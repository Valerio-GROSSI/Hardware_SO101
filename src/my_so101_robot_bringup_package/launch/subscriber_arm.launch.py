import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    set_gz_sim_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[os.environ.get('GZ_SIM_RESOURCE_PATH', ''), os.pathsep, os.path.join(get_package_prefix('so101_robot_description'),'share')])
    set_gazebo_classic_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[os.environ.get('GAZEBO_MODEL_PATH', ''), os.pathsep, os.path.join(get_package_prefix('so101_robot_description'),'share')])
    #export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix so101_robot_description)/share
    #export GAZEBO_MODEL_PATH="$(ros2 pkg prefix so101_robot_description)/share:${GAZEBO_MODEL_PATH}"

    use_sim = LaunchConfiguration("use_sim")
    use_sim_value = (LaunchConfiguration('use_sim').perform(context).lower() == "true")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_gazebo_value = (LaunchConfiguration('sim_gazebo').perform(context).lower() == "true")
    sim_gazebo_classic = LaunchConfiguration("sim_gazebo_classic")
    sim_gazebo_classic_value = (LaunchConfiguration('sim_gazebo_classic').perform(context).lower() == "true")

    if use_sim_value and sim_gazebo_value == sim_gazebo_classic_value:
        raise RuntimeError(
            "Lorsque use_sim=true, exactement un simulateur doit être sélectionné : "
            "sim_gazebo ou sim_gazebo_classic."
        )

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    usb_port = LaunchConfiguration("usb_port")
    recalibrate = LaunchConfiguration("recalibrate")

    arm = LaunchConfiguration("arm")
    arm_value = arm.perform(context)

    namespace = LaunchConfiguration("namespace")
    namespace_value = LaunchConfiguration("namespace").perform(context)

    use_moveit = LaunchConfiguration("use_moveit")
    use_moveit_value = (LaunchConfiguration("use_moveit").perform(context).lower() == "true")

    if use_moveit_value:
        namespace = ""
        namespace_value = ""

    activate_trajectory_controller = (LaunchConfiguration("activate_trajectory_controller").perform(context) == "true")

    use_rviz = LaunchConfiguration("use_rviz")
    headless_value = (LaunchConfiguration('headless').perform(context).lower() == "true")

    initial_positions_file = PathJoinSubstitution([FindPackageShare("so101_robot_description"), "urdf", "initial_positions.yaml"])
    controllers_config_file = PathJoinSubstitution([FindPackageShare("so101_robot_bringup"), "config", "sub_controllers.yaml"])

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                " ",
                PathJoinSubstitution([
                    FindPackageShare("so101_robot_description"),
                    "urdf",
                    "my_so101_robot.urdf.xacro", # Séparer cas Leader et Follower
                    ]),
                " ",
                " use_sim:=",
                use_sim,
                " sim_gazebo:=",
                sim_gazebo,
                " sim_gazebo_classic:=",
                sim_gazebo_classic,
                " use_mock_hardware:=",
                use_mock_hardware,
                " mock_sensor_commands:=",
                mock_sensor_commands,
                " initial_positions_file:=",
                initial_positions_file,
                " controller_config_file:=",
                controllers_config_file,
                " usb_port:=",
                usb_port,
                " recalibrate:=",
                recalibrate,
                " arm:=",
                arm,
                " namespace:=",
                namespace,
                " role:=follower", # subscriber_arm.launch.py donc le comportement hardware du bras est celui d'un follower (torque enclenché) et command_interfaces existants, que ce dernier soit un follower comme leader
            ]
        ),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_parameters = [robot_description]

    if namespace_value:
        robot_state_publisher_parameters.append({"frame_prefix": f"{namespace_value}/"})
        rviz_config = PathJoinSubstitution([FindPackageShare("so101_robot_description"), "rviz", f"ns_{arm_value}.rviz"])
    else:
        rviz_config = PathJoinSubstitution([FindPackageShare("so101_robot_description"), "rviz", f"{arm_value}.rviz"])

    rviz_config = PathJoinSubstitution([FindPackageShare("so101_robot_description"), "rviz", f"{arm_value}.rviz"])

    world_filename = ("empty_gz_sim.sdf" if sim_gazebo_value else "empty_gazebo_classic.sdf")
    world_config_value = PathJoinSubstitution([FindPackageShare("so101_robot_description"),"world",world_filename]).perform(context)
    gz_args = f"--headless-rendering -s -v 4 -r {world_config_value}" if headless_value else f"-r {world_config_value}"
    
    forward_position_controller_arguments = ["forward_position_controller"]
    joint_trajectory_controller_arguments = ["joint_trajectory_controller"]
    if activate_trajectory_controller:
        joint_trajectory_controller_arguments.append("--inactive")
    else:
        forward_position_controller_arguments.append("--inactive")

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace=namespace,
        name="robot_state_publisher",
        parameters=robot_state_publisher_parameters,
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        namespace=namespace,
        parameters=[controllers_config_file],
        remappings=[("~/robot_description", "robot_description")],
        condition=UnlessCondition(use_sim),
    )

    gz_sim = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                FindPackageShare("ros_gz_sim"),
                                "launch",
                                "gz_sim.launch.py",
                            ])
                        ),
                        launch_arguments={
                            "gz_args": gz_args,
                            "on_exit_shutdown": "True",
                        }.items(),
                        condition=IfCondition(str(use_sim_value and sim_gazebo_value).lower()),
                )

    gazebo_classic = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                FindPackageShare("gazebo_ros"),
                                "launch",
                                "gazebo.launch.py",
                            ])
                        ),
                        launch_arguments={
                            "world": world_config_value,
                            "gui": str(not headless_value).lower(),
                            "verbose": "true",
                        }.items(),
                        condition=IfCondition(str(use_sim_value and sim_gazebo_classic_value).lower())
                )

    gz_spawn_entity = Node(
            package="ros_gz_sim",
            executable="create",
            namespace=namespace,
            arguments=[
                "-name", f"so101_{arm_value}",
                "-topic", "robot_description",
                "-x", LaunchConfiguration("x", default="0.00"),
                "-y", LaunchConfiguration("y", default="0.00"),
                "-z", LaunchConfiguration("z", default="0.00"),
                "-R", LaunchConfiguration("roll", default="0.00"),
                "-P", LaunchConfiguration("pitch", default="0.00"),
                "-Y", LaunchConfiguration("yaw", default="0.00"),
            ],
            output="screen",
            condition=IfCondition(str(use_sim_value and sim_gazebo_value).lower()),
        )

    gazebo_classic_spawn_entity = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            namespace=namespace,
            arguments=[
                "-entity", f"so101_{arm_value}",
                "-topic", "robot_description",
                "-x", LaunchConfiguration("x", default="0.00"),
                "-y", LaunchConfiguration("y", default="0.00"),
                "-z", LaunchConfiguration("z", default="0.00"),
                "-R", LaunchConfiguration("roll", default="0.00"),
                "-P", LaunchConfiguration("pitch", default="0.00"),
                "-Y", LaunchConfiguration("yaw", default="0.00"),
            ],
            output="screen",
            condition=IfCondition(str(use_sim_value and sim_gazebo_classic_value).lower()),
        )

    gz_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        condition=IfCondition(str(use_sim_value and sim_gazebo_value).lower()),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster"], #"--controller-manager", "controller_manager"
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=forward_position_controller_arguments, 
    )
    
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=joint_trajectory_controller_arguments,
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["arm_controller"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["gripper_controller"],
    )

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    if use_sim_value:
        spawn_action = (
            gz_spawn_entity if sim_gazebo_value else gazebo_classic_spawn_entity
        )
        delay_joint_state_broadcaster_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_action,
                on_exit=[
                    TimerAction(
                        period=3.0,
                        actions=[joint_state_broadcaster_spawner],
                    )
                ],
            )
        )
    else:
        delay_joint_state_broadcaster_spawner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=ros2_control_node,
                on_start=[
                    TimerAction(
                        period=3.0,
                        actions=[joint_state_broadcaster_spawner],
                    )
                ],
            )
        )
    delay_forward_position_controller_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner],
        ),
        condition=UnlessCondition(use_moveit),
    )
    delay_joint_trajectory_controller_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=joint_state_broadcaster_spawner,
        on_exit=[joint_trajectory_controller_spawner],
        ),
        condition=UnlessCondition(use_moveit),
    )
    delay_arm_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        ),
        condition=IfCondition(use_moveit),
    )
    delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
        target_action=joint_state_broadcaster_spawner,
        on_exit=[gripper_controller_spawner],
        ),
        condition=IfCondition(use_moveit),
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=namespace,
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        namespace=namespace,
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return [
        set_gz_sim_resource_path,
        set_gazebo_classic_model_path,
        robot_state_pub_node,
        ros2_control_node,
        gz_sim,
        gazebo_classic,
        gz_spawn_entity,
        gazebo_classic_spawn_entity,
        gz_clock_bridge,
        delay_joint_state_broadcaster_spawner,
        delay_forward_position_controller_after_joint_state_broadcaster_spawner,
        delay_joint_trajectory_controller_after_joint_state_broadcaster_spawner,
        delay_arm_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_gripper_controller_spawner_after_joint_state_broadcaster_spawner,
        # joint_state_broadcaster_spawner,
        rviz_node,
    ]

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false')
    sim_gazebo_arg = DeclareLaunchArgument('sim_gazebo', default_value='true')
    sim_gazebo_classic_arg = DeclareLaunchArgument('sim_gazebo_classic', default_value='false')
    use_mock_hardware_arg = DeclareLaunchArgument("use_mock_hardware", default_value="false", description="Start robot with mock hardware mirroring command to its states.")
    mock_sensor_commands_arg = DeclareLaunchArgument("mock_sensor_commands", default_value="true", description="Start robot with mock sensor commands.")
    usb_port_arg = DeclareLaunchArgument("usb_port", default_value="/dev/ttyACM0")
    recalibrate_arg = DeclareLaunchArgument("recalibrate", default_value="false")
    arm_arg = DeclareLaunchArgument("arm", default_value="follower", choices=["leader", "follower"], description="Modèle du bras")
    namespace_arg = DeclareLaunchArgument("namespace", default_value="", description="ROS namespace; empty means root namespace",)
    activate_trajectory_controller_arg = DeclareLaunchArgument("activate_trajectory_controller", default_value="false")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")
    headless_arg = DeclareLaunchArgument("headless", default_value="false")
    use_moveit_arg = DeclareLaunchArgument("use_moveit", default_value="false") 

    return LaunchDescription([
        use_sim_arg,
        sim_gazebo_arg,
        sim_gazebo_classic_arg,
        use_mock_hardware_arg,
        mock_sensor_commands_arg,
        usb_port_arg,
        recalibrate_arg,
        arm_arg,
        namespace_arg,
        activate_trajectory_controller_arg,
        use_rviz_arg,
        headless_arg,
        use_moveit_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        OpaqueFunction(function = launch_setup),
    ])

if __name__ == "__main__":
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()