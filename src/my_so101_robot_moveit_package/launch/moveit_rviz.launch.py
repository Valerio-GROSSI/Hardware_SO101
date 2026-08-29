from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "so101",
            package_name="my_so101_robot_moveit_package",
        )
        .to_moveit_configs()
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("my_so101_robot_moveit_package"),
            "config",
            "moveit.rviz",
        ]
    )

    rviz_kinematics = PathJoinSubstitution(
        [
            FindPackageShare("my_so101_robot_moveit_package"),
            "config",
            "rviz_kinematics.yaml",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
            "--ros-args",
            "--params-file",
            rviz_kinematics,
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([rviz_node])