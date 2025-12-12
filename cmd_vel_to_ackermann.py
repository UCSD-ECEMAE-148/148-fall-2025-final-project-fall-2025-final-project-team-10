import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time

class TwistToRaw(Node):
    def __init__(self):
        super().__init__('twist_to_raw')
        
        # --- CONFIGURATION ---
        self.speed_scaler = 0.5        # Power Multiplier
        self.min_power = 0.15          # Minimum power to overcome friction
        
        self.steering_center = 0.5     # Servo Straight
        self.steering_gain = 0.5       # Turn sensitivity
        self.timeout_sec = 0.2         # Time before "Safety Stop"
        # ---------------------

        self.last_cmd_time = self.get_clock().now().nanoseconds
        self.cmd_active = False

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.motor_pub = self.create_publisher(Float64, '/commands/motor/duty_cycle', 10)
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.create_timer(0.05, self.timer_callback)
            
        self.get_logger().info('Bridge: Friction Comp + Turn-in-Place Hack ACTIVE')

    def listener_callback(self, msg):
        self.last_cmd_time = self.get_clock().now().nanoseconds
        self.cmd_active = True
        
        # 1. Handle Steering
        # Invert logic: (Turn Rate * Gain) * -1
        servo_offset = (msg.angular.z * self.steering_gain)
        servo_val = self.steering_center + servo_offset
        servo_val = max(min(servo_val, 0.85), 0.15) # Clamp

        # 2. Handle Speed (FIXED)
        target_speed = msg.linear.x
        
        if abs(target_speed < 0.01) and abs(msg.angular.z) > 0.01:
            self.get_logger().warn("converting pure turn to move forward")
            target_speed = 0.1
        servo_offset = (msg.angular.z * self.steering_gain) * -1
        servo_val = self.steering_center + servo_offset
        servo_val = max(min(servo_val, 0.85), 0.15) #
        # Check ABSOLUTE speed to see if we should move (allows negative/reverse)
        if abs(target_speed) > 0.01:
            # A. Calculate Base Power

            # B. Apply Friction Help (The "Kick")
            # We must add friction help in the direction of travel
            if target_speed > 0: # Forward
                duty_val = 0.05

            else: # Reverse
                duty_val = -.05
                
        else:
            duty_val = 0.0

        self.get_logger().info(f'Received: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}')
        self.get_logger().info(f'Sending: duty={duty_val:.3f}, servo={servo_val:.3f}')
        # 3. Publish
        self.publish_command(duty_val, servo_val)
    def timer_callback(self):
        now = self.get_clock().now().nanoseconds
        elapsed = (now - self.last_cmd_time) / 1e9

        if elapsed > self.timeout_sec:
            self.publish_command(0.0, self.steering_center)
            self.cmd_active = False

    def publish_command(self, motor, servo):
        m_msg = Float64()
        m_msg.data = float(motor)
        self.motor_pub.publish(m_msg)
        s_msg = Float64()
        s_msg.data = float(servo)
        self.servo_pub.publish(s_msg)

def main():
    rclpy.init()
    node = TwistToRaw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
