#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class RightWallFollower(Node):
    def __init__(self):
        super().__init__('right_wall_follower')

        # Subscribe to LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Publish to cmd_vel (temporary — later connect to VESC)
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.get_logger().info("Wall follower node started!")

        # Desired distance from right wall (meters)
        self.target_dist = 0.40

        # PID simple proportional gain
        self.kp = 1.2

    def scan_callback(self, scan: LaserScan):
        # Debug print (fixed variable name)
        self.get_logger().info(f"Received scan with {len(scan.ranges)} points")

        ranges = list(scan.ranges)
        n = len(ranges)

        # Convert angle sector into indices safely
        # We want right side = 270° ± 20° → sector = [250° → 290°]
        def deg_to_idx(deg):
            # Convert degrees to index using Scan Angular Resolution
            angle = math.radians(deg)
            idx = int((angle - scan.angle_min) / scan.angle_increment)
            return max(0, min(n-1, idx))

        right_start = deg_to_idx(250)
        right_end   = deg_to_idx(290)

        right_distances = [
            r for r in ranges[right_start:right_end+1]
            if not math.isinf(r) and not math.isnan(r)
        ]

        if len(right_distances) == 0:
            # No valid readings
            self.get_logger().warn("No right wall detected — moving forward")
            self.drive(0.1, 0.0)
            return

        # The closest right-side wall distance
        right_dist = min(right_distances)
        error = self.target_dist - right_dist

        # Positive error → too far → turn right
        # Negative error → too close → turn left
        turn = self.kp * error
        turn = max(min(turn, 0.6), -0.6)

	if turn < 0:
            self.get_logger().info(f"Too close! Turning Left.")
	else:
            self.get_logger().info(f"Too far! Turning Right.")

        forward = 0.15
        self.drive(forward, turn)

    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RightWallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
