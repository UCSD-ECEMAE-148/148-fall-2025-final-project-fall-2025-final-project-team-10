#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from smbus2 import SMBus
import time

class ServoSweep(Node):
    def __init__(self):
        super().__init__('servo_sweep')

        # PCA9685 default I2C address
        self.address = 0x40
        self.bus = SMBus(1)

        # Frequency for servo (50Hz)
        self.pca9685_init()

        # Servo on channel 0
        self.channel = 0

        # Begin sweeping
        self.sweep_servo()

    def pca9685_init(self):
        # Reset PCA9685
        self.bus.write_byte_data(self.address, 0x00, 0x00)
        time.sleep(0.01)

        # Set PWM frequency to 50Hz
        freq = 50
        prescale = int(25000000.0 / (4096 * freq) - 1)
        self.bus.write_byte_data(self.address, 0x00, 0x10)   # sleep
        self.bus.write_byte_data(self.address, 0xFE, prescale)
        self.bus.write_byte_data(self.address, 0x00, 0x00)   # wake
        time.sleep(0.01)

    def set_servo_angle(self, channel, angle):
        # Constrain angle
        angle = max(0, min(180, angle))

        # Map angle → pulse length
        pulse_min = 150  # 0°
        pulse_max = 600  # 180°
        pulse = int(pulse_min + (angle / 180) * (pulse_max - pulse_min))

        # Write to PCA9685 registers
        reg = 0x06 + 4 * channel
        self.bus.write_byte_data(self.address, reg, 0)
        self.bus.write_byte_data(self.address, reg + 1, 0)
        self.bus.write_byte_data(self.address, reg + 2, pulse & 0xFF)
        self.bus.write_byte_data(self.address, reg + 3, pulse >> 8)

    def sweep_servo(self):
        # Sweep up
        for angle in range(0, 181, 20):
            self.get_logger().info(f"Angle: {angle}")
            self.set_servo_angle(self.channel, angle)
            time.sleep(0.5)

        # Sweep down
        for angle in range(180, -1, -20):
            self.get_logger().info(f"Angle: {angle}")
            self.set_servo_angle(self.channel, angle)
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = ServoSweep()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
