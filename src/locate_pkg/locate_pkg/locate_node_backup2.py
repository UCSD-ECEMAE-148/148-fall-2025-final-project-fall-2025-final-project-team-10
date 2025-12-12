#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist


class LocateNode(Node):
    def __init__(self):
        super().__init__("locate_node")

        # State variables
        self.is_found = False        # comes from /is_found
        self.obj_area = 0.0          # comes from /obj_area (fraction 0-1)
        self.current_angle = 0.5     # default center (normalized 0-1)

        self.last_found_state = False
        self.last_located_state = False

        # -------------------- SUBSCRIPTIONS --------------------
        self.create_subscription(Bool, "/is_found", self.found_callback, 10)
        self.create_subscription(Float32, "/angle", self.angle_callback, 10)
        self.create_subscription(Float32, "/obj_area", self.area_callback, 10)  # changed topic and callback

        # -------------------- PUBLISHERS --------------------
        self.cmd_vel_pub = self.create_publisher(Twist, "/locate/cmd_vel", 10)
        self.is_found_pub = self.create_publisher(Bool, "/is_found", 10)
        self.is_located_pub = self.create_publisher(Bool, "/is_located", 10)

        self.get_logger().info("Locate node started — standing by for object detection.")

    # ============================================================
    # CALLBACKS
    # ============================================================

    def found_callback(self, msg: Bool):
        self.is_found = msg.data
        if self.is_found != self.last_found_state:
            if self.is_found:
                self.get_logger().info("Object FOUND — beginning locate behavior.")
            else:
                self.get_logger().info("Locate disabled — /is_found set to False.")

            self.last_found_state = self.is_found
    def angle_callback(self, msg: Float32):
        self.current_angle = msg.data  # normalized 0 → 1

        # If the object is found and not yet located, steer toward it
        if self.is_found and not self.is_object_located():
            self.publish_steering_and_motion()

    def area_callback(self, msg: Float32):
        self.obj_area = msg.data

        # If object is located (area fraction above 0.5)
        if self.obj_area > 0.5:
            self.handle_object_located()

    # ============================================================
    # CONTROL LOGIC
    # ============================================================

    def publish_steering_and_motion(self):
        """Send Twist commands to steer toward the object."""
        self.last_located_state = False
        twist = Twist()
        twist.linear.x = 0.1  # very slow forward movement

        # Clamp angle to [0, 1]
        a = max(0.0, min(1.0, self.current_angle))

        # Convert [0, 1] → [-1, 1]
        twist.angular.z = (a - 0.5) * 2.0

        self.cmd_vel_pub.publish(twist)

    def is_object_located(self):
        """Check if object area fraction is above 0.5."""
        return self.obj_area > 0.5

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
        if not self.last_located_state:
            self.get_logger().info("Object LOCATED (area > 0.5). Stopping and ending locate behavior.")
            self.last_located_state = True
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
