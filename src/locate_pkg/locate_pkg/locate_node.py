#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Empty
from geometry_msgs.msg import Twist

class LocateNode(Node):
    def __init__(self):
        super().__init__("locate_node")

        # --- STATE VARIABLES ---
        self.is_found = False
        self.obj_area = 0.0
        self.current_angle = 0.5
        
        # LOGGING STATE
        self.last_found_state = False 
        self.last_located_state = False
        
        # MISSION STATE (The Fix)
        # False = "Armed but waiting". True = "Authorized to drive".
        self.mission_active = False 

        # --- SUBSCRIPTIONS ---
        self.create_subscription(Bool, "/is_found", self.found_callback, 10)
        self.create_subscription(Float32, "/angle", self.angle_callback, 10)
        self.create_subscription(Float32, "/obj_area", self.area_callback, 10)
        
        # New Subscription: Listen for the "GO" command
        self.create_subscription(Empty, "/mission/go", self.go_callback, 10)

        # --- PUBLISHERS ---
        self.cmd_vel_pub = self.create_publisher(Twist, "/locate/cmd_vel", 10)
        self.is_found_pub = self.create_publisher(Bool, "/is_found", 10)
        self.is_located_pub = self.create_publisher(Bool, "/is_located", 10)

        self.get_logger().info("Locate Node Initialized. Waiting for Object + 'GO' command.")

    # ============================================================
    # CALLBACKS
    # ============================================================

    def go_callback(self, msg):
        """Enable movement when 'G' is pressed"""
        if not self.mission_active:
            self.get_logger().info(">>> MISSION START: Locate System Authorized to Move.")
            self.mission_active = True

    def found_callback(self, msg: Bool):
        self.is_found = msg.data

        if self.is_found != self.last_found_state:
            if self.is_found:
                self.get_logger().info("Object Detected.")
            else:
                self.get_logger().info("Target Lost.")
            self.last_found_state = self.is_found

    def angle_callback(self, msg: Float32):
        self.current_angle = msg.data
        # ONLY move if we found the object AND mission is active
        if self.is_found and not self.is_object_located() and self.mission_active:
            self.publish_steering_and_motion()

    def area_callback(self, msg: Float32):
        self.obj_area = msg.data
        if self.obj_area > 0.5:
            self.handle_object_located()

    # ============================================================
    # CONTROL LOGIC
    # ============================================================

    def publish_steering_and_motion(self):
        self.last_located_state = False
        
        twist = Twist()
        twist.linear.x = 0.1
        a = max(0.0, min(1.0, self.current_angle))
        twist.angular.z = (a - 0.5) * 2.0
        self.cmd_vel_pub.publish(twist)

    def is_object_located(self):
        return self.obj_area > 0.5

    def handle_object_located(self):
        # Stop the robot
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_twist)

        # Publish updates
        self.is_located_pub.publish(Bool(data=True))
        self.is_found_pub.publish(Bool(data=False))
        self.is_found = False 

        if not self.last_located_state:
            self.get_logger().info("OBJECT FOUND (Arrived). Stopping.")
            self.last_located_state = True

def main(args=None):
    rclpy.init(args=args)
    node = LocateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
