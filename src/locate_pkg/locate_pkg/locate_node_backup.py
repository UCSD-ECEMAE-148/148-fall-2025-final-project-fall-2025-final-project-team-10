#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist


class LocateNode(Node):
    def __init__(self):
        super().__init__("locate_node")

        # State variables
        self.is_found = False       # comes from /is_found
        self.obj_depth = None       # comes from /obj_depth
        self.current_angle = 0.5    # default center (normalized 0-1)

        # -------------------- SUBSCRIPTIONS --------------------
        self.create_subscription(Bool, "/is_found", self.found_callback, 10)
        self.create_subscription(Float32, "/angle", self.angle_callback, 10)
        self.create_subscription(Float32, "/obj_depth", self.depth_callback, 10)

        # -------------------- PUBLISHERS --------------------
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.is_found_pub = self.create_publisher(Bool, "/is_found", 10)
        self.is_located_pub = self.create_publisher(Bool, "/is_located", 10)

        self.get_logger().info("Locate node started — standing by for object detection.")

    # ============================================================
    # CALLBACKS
    # ============================================================

    def found_callback(self, msg: Bool):
        self.is_found = msg.data

        if self.is_found:
            self.get_logger().info("Object FOUND — beginning locate behavior.")
        else:
            self.get_logger().info("Locate disabled — /is_found set to False.")

    def angle_callback(self, msg: Float32):
        self.current_angle = msg.data  # normalized 0 → 1

        # If the object is found and not yet located, steer toward it
        if self.is_found and not self.is_object_located():
            self.publish_steering_and_motion()

    def depth_callback(self, msg: Float32):
        self.obj_depth = msg.data

        # If object is located (within 1–7 inches)
        if self.obj_depth is not None and 25.0 <= self.obj_depth <= 178.0:
            self.handle_object_located()

    # ============================================================
    # CONTROL LOGIC
    # ============================================================

    def publish_steering_and_motion(self):
        """Send Twist commands to steer toward the object."""
        twist = Twist()
        twist.linear.x = 0.1  # very slow forward movement

        # Clamp angle to [0, 1]
        a = max(0.0, min(1.0, self.current_angle))

        # Convert [0, 1] → [-1, 1]
        twist.angular.z = (a - 0.5) * 2.0

        self.cmd_vel_pub.publish(twist)

    def is_object_located(self):
        """Check if object is within 25 - 178 mm (1 - 7 inches)."""
        return self.obj_depth is not None and 25.0 <= self.obj_depth <= 178.0

    def handle_object_located(self):
        """Stop the robot and update /is_located and /is_found."""
        # Stop the robot
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_twist)

        # Publish updates
        self.is_located_pub.publish(Bool(data=True))
        self.is_found_pub.publish(Bool(data=False))

        self.is_found = False  # internal flag

        self.get_logger().info("Object LOCATED (1–7 inches). Stopping and ending locate behavior.")

# ============================================================
# MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = LocateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
