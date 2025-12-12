#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smbus2 import SMBus
#from std_msgs.msg import Bool , Float32
import time

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        # Create I2C bus (works in Docker)
        self.bus = SMBus(1)

        # PCA9685 address
        self.address = 0x40

        # Initialize PCA9685
        self.pca9685_init()

        # Track whether the servo should stop
 #       self.is_found_sub = True

        # Move servo every second (example loop)
        self.timer = self.create_timer(1.0, self.move_servo)

        # Subscribe to the object detection topic
  #      self.is_found_sub = self.create_subscription(
   #         Bool,
    #        '/is_found',   # Replace with the topic your sensor publishes
     #       self.sensor_callback,
      #      10  # QoS queue size
       # )

        #Publisher to the angle the servo is at to send to the vesc
#        self.angle_pub = self.create_publisher(
 #       Float32,
  #      '/angle'
   #     10
    #    )


        self.position = 0


    def write_reg(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def set_pwm(self, channel, on, off):
        self.write_reg(0x06 + 4 * channel, on & 0xFF)
        self.write_reg(0x07 + 4 * channel, on >> 8)
        self.write_reg(0x08 + 4 * channel, off & 0xFF)
        self.write_reg(0x09 + 4 * channel, off >> 8)

    def set_pwm_freq(self, freq):
        prescale = int(25000000.0 / (4096 * freq) - 1)
        oldmode = self.bus.read_byte_data(self.address, 0x00)
        self.write_reg(0x00, (oldmode & 0x7F) | 0x10)
        self.write_reg(0xFE, prescale)
        self.write_reg(0x00, oldmode)
        time.sleep(0.005)
        self.write_reg(0x00, oldmode | 0xa1)

    def pca9685_init(self):
        self.write_reg(0x00, 0x00)  # MODE1 reset
        self.set_pwm_freq(50)      # 50Hz for servo motors

   # def move_servo(self):
     #   if self.is_found_sub:
      #      self.getlogger().info("Object detected, stopping servo")
       #     return

        # 0° = ~150, 90° = ~375, 180° = ~600  110 = ~425
      #  for angle 
        positions = [150, 170, 190, 210, 230, 250, 270, 290, 310, 330, 350, 330, 310, 290, 270, 250, 230, 210, 190, 170]
        pulse = positions[self.position]
        delay = 1

        self.get_logger().info(f"Moving servo to: {pulse}")

        self.set_pwm(0, 0, pulse)  # channel 0
        time.sleep(delay)

        self.position = (self.position + 1) % len(positions)

#    def publish_heading(self):
        #send the current servo angle to /servo_heading
 #       angle = self.position * self.angle_step #integrate the angle conversion here
  #      msg = Float32()
   #     msg.data = float(angle)
    #    self.heading_pub.publish(msg)
     #   self.get_logger().info(f"Published servo heading: {angle:.1f}°")

#    def sensor_callback(self, msg):
       #Update whether servo should stop
 #       self.is_found_sub = msg.data
  #      if msg.data:
   #         self.get_logger().info("Object detected --> servo sending heading.")
    #    else:
     #       self.get_logger().info("Object NOT detected yet --> resuming scan.")


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
