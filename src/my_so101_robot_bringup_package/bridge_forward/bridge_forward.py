#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class LeaderFollowerBridge(Node):

    def __init__(self):
        super().__init__("leader_follower_bridge")

        # Ordre exact attendu par le forward_position_controller
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            "/follower/forward_position_controller/commands",
            10,
        )

        self.joint_state_subscription = self.create_subscription(
            JointState,
            "/leader/joint_states",
            self.joint_state_callback,
            10,
        )

        self.get_logger().info(
            "Bridge: /leader/joint_states -> "
            "/follower/forward_position_controller/commands"
        )

    def joint_state_callback(self, msg: JointState):
        positions_by_name = dict(zip(msg.name, msg.position))

        missing_joints = [
            name
            for name in self.joint_names
            if name not in positions_by_name
        ]

        if missing_joints:
            self.get_logger().warning(
                f"Missing joints: {missing_joints}",
                throttle_duration_sec=2.0,
            )
            return

        command = Float64MultiArray()
        command.data = [
            positions_by_name[name]
            for name in self.joint_names
        ]

        self.command_publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()