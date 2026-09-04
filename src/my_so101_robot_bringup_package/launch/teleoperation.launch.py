from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    publisher_usb_port = LaunchConfiguration("publisher_usb_port")
    subscriber_usb_port = LaunchConfiguration("subscriber_usb_port")
    publisher_arm = LaunchConfiguration("publisher_arm")
    subscriber_arm = LaunchConfiguration("subscriber_arm")
    publisher_namespace = LaunchConfiguration("publisher_namespace")
    subscriber_namespace = LaunchConfiguration("subscriber_namespace")
    controller = LaunchConfiguration("controller")
    use_rviz=LaunchConfiguration("use_rviz")
    pub_arm_joint_topic = PythonExpression(["'/' + '", 
                                            publisher_namespace, 
                                            "' + '/joint_states'"])
    sub_arm_jtc_topic = PythonExpression(["'/' + '",
                                         subscriber_namespace,
                                         "' + '/joint_trajectory_controller/joint_trajectory'"])
    sub_arm_fcc_topic = PythonExpression(["'/' + '",
                                         subscriber_namespace,
                                         "' + '/forward_position_controller/commands'"])

    publisher_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('so101_robot_bringup'),
                'launch',
                'publisher_arm.launch.py',
            ])),
            launch_arguments={
                                "usb_port": publisher_usb_port,
                                "arm": publisher_arm,
                                "namespace": publisher_namespace,
                                "use_rviz": use_rviz,
            }.items(),
        )

    subscriber_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('so101_robot_bringup'),
                'launch',
                'subscriber_arm.launch.py',
            ])),
            launch_arguments={
                                "usb_port": subscriber_usb_port,
                                "arm": subscriber_arm,
                                "namespace": subscriber_namespace,
                                "controller": controller,
                                "use_rviz": use_rviz,
            }.items(),
        )

    teleop = Node(package="so101_robot_bringup",
                  executable="teleop",
                  name="publisher_subscriber_relay",
                  output="screen",
                  parameters=[
                      {
                        "controller": controller,
                        "pub_arm_joint_topic": pub_arm_joint_topic,
                        "sub_arm_jtc_topic": sub_arm_jtc_topic,
                        "sub_arm_fcc_topic": sub_arm_fcc_topic,
                      },
                  ],
            )

    return [
        publisher_arm_launch,
        subscriber_arm_launch,
        teleop,
    ]

def generate_launch_description():
    publisher_usb_port_arg = DeclareLaunchArgument("publisher_usb_port", default_value="/dev/ttyACM0")
    subscriber_usb_port_arg = DeclareLaunchArgument("subscriber_usb_port", default_value="/dev/ttyACM1")
    publisher_arm_arg = DeclareLaunchArgument("publisher_arm", default_value="leader", choices=["leader", "follower"], description="Modèle du bras")
    subscriber_arm_arg = DeclareLaunchArgument("subscriber_arm", default_value="follower", choices=["leader", "follower"], description="Modèle du bras")
    publisher_namespace_arg = DeclareLaunchArgument("publisher_namespace", default_value="leader")
    subscriber_namespace_arg = DeclareLaunchArgument("subscriber_namespace", default_value="follower")
    controller_arg = DeclareLaunchArgument("controller", default_value="forward_position_controller", choices=["forward_position_controller", "joint_trajectory_controller"])
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")

    # use_mock_hardware_arg = DeclareLaunchArgument("use_mock_hardware", default_value="false", description="Start robot with mock hardware mirroring command to its states.")
    # mock_sensor_commands_arg = DeclareLaunchArgument("mock_sensor_commands", default_value="true", description="Start robot with mock sensor commands.")

    return LaunchDescription([
        publisher_usb_port_arg,
        subscriber_usb_port_arg,
        publisher_arm_arg,
        subscriber_arm_arg,
        publisher_namespace_arg,
        subscriber_namespace_arg,
        controller_arg,
        use_rviz_arg,

        # use_mock_hardware_arg,
        # mock_sensor_commands_arg,
        OpaqueFunction(function = launch_setup),
    ])

if __name__ =="__main__":
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()