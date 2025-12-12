#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os

class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker_node')

        self.is_found_sub = self.create_subscription(
            Bool,
            '/is_found',
            self.is_found_callback,
            10
        )

        self.is_located_sub = self.create_subscription(
            Bool,
            '/is_located',
            self.is_located_callback,
            10
        )

        self.get_logger().info("Speaker node started and waiting for object status...")

    def is_found_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Object FOUND! Playing sound...")
            os.system('aplay /home/projects/ros2_ws/projects/Found_it.wav')
        else:
            self.get_logger().warn("Received /is_found = False (unknown object status)")

    def is_located_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Object LOCATED! Playing sound...")
            os.system('aplay /home/projects/ros2_ws/projects/Located_it.wav')
        else:
            self.get_logger().warn("Received /is_located = False (unknown object status)")


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
