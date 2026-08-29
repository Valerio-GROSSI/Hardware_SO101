from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    usb_port = LaunchConfiguration("usb_port")
    recalibrate = LaunchConfiguration("recalibrate")

    arm = LaunchConfiguration("arm")
    arm_value = arm.perform(context)
    namespace = LaunchConfiguration("namespace")
    namespace_value = LaunchConfiguration("namespace").perform(context)    
    use_rviz = LaunchConfiguration("use_rviz")

    initial_positions_file = PathJoinSubstitution([FindPackageShare("so101_robot_description"), "urdf", "initial_positions.yaml"])
    controllers_config_file = PathJoinSubstitution([FindPackageShare("so101_robot_bringup"), "config", "pub_controllers.yaml"])

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
                " use_sim:=false", # publisher_arm.launch.py donc on n'utilise aucun environnement de simulation
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
                " role:=leader", # publisher_arm.launch.py donc le comportement hardware du bras est celui d'un leader (torque désenclenché) et command_interfaces non existants, que ce dernier soit un leader comme follower
            ]
        ),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    if namespace_value:
        rviz_config = PathJoinSubstitution([FindPackageShare("so101_robot_description"), "rviz", f"ns_{arm_value}.rviz"])
    else:
        rviz_config = PathJoinSubstitution([FindPackageShare("so101_robot_description"), "rviz", f"{arm_value}.rviz"])

    robot_state_publisher_parameters = [robot_description]
    if namespace_value:
        robot_state_publisher_parameters.append({"frame_prefix": f"{namespace_value}/"})

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
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
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
        robot_state_pub_node,
        ros2_control_node,
        delay_joint_state_broadcaster_spawner,
        # joint_state_publisher_gui_node,
        rviz_node,
    ]

def generate_launch_description():

    use_mock_hardware_arg = DeclareLaunchArgument("use_mock_hardware", default_value="false", description="Start robot with mock hardware mirroring command to its states.")
    mock_sensor_commands_arg = DeclareLaunchArgument("mock_sensor_commands", default_value="true", description="Start robot with mock sensor commands.")
    usb_port_arg = DeclareLaunchArgument("usb_port", default_value="/dev/ttyACM0")
    recalibrate_arg = DeclareLaunchArgument("recalibrate", default_value="false")
    arm_arg = DeclareLaunchArgument("arm", default_value="leader", choices=["leader", "follower"], description="Modèle du bras")
    namespace_arg = DeclareLaunchArgument("namespace", default_value="", description="ROS namespace; empty means root namespace",)
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")

    return LaunchDescription([
        use_mock_hardware_arg,
        mock_sensor_commands_arg,
        usb_port_arg,
        recalibrate_arg,
        arm_arg,
        namespace_arg,
        use_rviz_arg,
        OpaqueFunction(function = launch_setup),
    ])

if __name__ == "__main__":
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()