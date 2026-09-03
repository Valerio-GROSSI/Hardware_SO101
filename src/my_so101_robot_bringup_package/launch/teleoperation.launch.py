from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    publisher_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('so101_robot_bringup'),
                'launch',
                'publisher_arm.launch.py',
            ]),
            launch_arguments={

            }.items(),
        ),
    )

    subscriber_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('so101_robot_bringup'),
                'launch',
                'subscriber_arm.launch.py',
            ]),
            launch_arguments={

            }.items(),
        ),
    )

    teleop = Node(package="so101_robot_bringup",
                  executable="teleop",
                  name="publisher_subscriber_relay",
                  output="screen",
                  parameters=[
                      {
                          
                      },
                  ],
            )

    return [
        publisher_arm_launch,
        subscriber_arm_launch,
        teleop,
    ]

def generate_launch_description():
    publisher_arm_arg = DeclareLaunchArgument("publisher_arm", default_value="leader", choices=["leader", "follower"], description="Modèle du bras")
    subscriber_arm_arg = DeclareLaunchArgument("subscriber_arm", default_value="follower", choices=["leader", "follower"], description="Modèle du bras")
    publisher_usb_port_arg = DeclareLaunchArgument("publisher_usb_port", default_value="/dev/ttyACM0")
    subscriber_usb_port_arg = DeclareLaunchArgument("subscriber_usb_port", default_value="/dev/ttyACM1")
    publisher_recalibrate_arg = DeclareLaunchArgument("publisher_recalibrate", default_value="false")
    subscriber_recalibrate_arg = DeclareLaunchArgument("subscriber_recalibrate", default_value="false")
    publisher_namespace_arg = DeclareLaunchArgument("publisher_namespace", default_value="leader")
    subscriber_namespace_arg = DeclareLaunchArgument("subscriber_namespace", default_value="follower")

    use_mock_hardware_arg = DeclareLaunchArgument("use_mock_hardware", default_value="false", description="Start robot with mock hardware mirroring command to its states.")
    mock_sensor_commands_arg = DeclareLaunchArgument("mock_sensor_commands", default_value="true", description="Start robot with mock sensor commands.")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")


    return LaunchDescription([
        publisher_arm_arg,
        subscriber_arm_arg,
        publisher_usb_port_arg,
        subscriber_usb_port_arg,
        publisher_recalibrate_arg,
        subscriber_recalibrate_arg,
        publisher_namespace_arg,
        subscriber_namespace_arg,
        use_mock_hardware_arg,
        mock_sensor_commands_arg,
        use_rviz_arg,
        OpaqueFunction(function = launch_setup),
    ])

if __name__ =="__main__":
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()