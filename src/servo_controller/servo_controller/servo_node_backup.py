import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import time
from smbus2 import SMBus

# PCA9685 constants
PCA9685_ADDRESS = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

FREQUENCY = 50  # 50 Hz for servos

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        self.deadzone = 5.0  # degrees, adjustable

        self.bus = SMBus(1)  # /dev/i2c-1
        self.address = PCA9685_ADDRESS

        self.set_pwm_freq(FREQUENCY)

        self.servo_channel = 0
        self.servo_angle = 90.0  # start centered

        self.is_found = False
        self.obj_pos = 0.0

        self.scan_direction = 1
        self.scan_speed = 1.0  # degrees per tick

        self.create_subscription(Bool, '/is_found', self.is_found_callback, 10)
        self.create_subscription(Float32, '/obj_pos', self.obj_pos_callback, 10)

        self.angle_pub = self.create_publisher(Float32, '/angle', 10)

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def write_byte(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read_byte(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq_hz):
        # Calculate prescale value for freq_hz
        prescaleval = int(round(25000000.0 / (4096 * freq_hz)) - 1)
        old_mode = self.read_byte(MODE1)
        new_mode = (old_mode & 0x7F) | 0x10  # sleep
        self.write_byte(MODE1, new_mode)
        self.write_byte(PRESCALE, prescaleval)
        self.write_byte(MODE1, old_mode)
        time.sleep(0.005)
        self.write_byte(MODE1, old_mode | 0x80)  # restart

    def set_pwm(self, channel, on, off):
        reg = LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.address, reg, on & 0xFF)
        self.bus.write_byte_data(self.address, reg + 1, on >> 8)
        self.bus.write_byte_data(self.address, reg + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, reg + 3, off >> 8)

    def angle_to_pwm(self, angle):
        # Convert angle (0-180) to PWM pulse (approx 1ms-2ms pulse width)
        pulse_min = 205   # 1ms pulse
        pulse_max = 410   # 2ms pulse

        pulse = int(pulse_min + (pulse_max - pulse_min) * angle / 180.0)
        return pulse

    def is_found_callback(self, msg):
        self.is_found = msg.data

    def obj_pos_callback(self, msg):
        self.obj_pos = msg.data

    def control_loop(self):
        if not self.is_found:
            self.servo_angle += self.scan_direction * self.scan_speed
            if self.servo_angle >= 180:
                self.servo_angle = 180
                self.scan_direction = -1
            elif self.servo_angle <= 0:
                self.servo_angle = 0
                self.scan_direction = 1
        else:
            target_angle = self.servo_angle - self.obj_pos
            target_angle = max(0, min(180, target_angle))
            error = target_angle - self.servo_angle
            if abs(error) > self.deadzone:
                self.servo_angle += 5.0 if error > 0 else -5.0
                self.servo_angle = max(0, min(180, self.servo_angle))

        pulse = self.angle_to_pwm(self.servo_angle)
        self.set_pwm(self.servo_channel, 0, pulse)

        if self.servo_angle <= 45:
            norm_angle = 0.0
        elif self.servo_angle >= 135:
            norm_angle = 1.0
        else:
            norm_angle = (self.servo_angle - 45) / 90.0

        msg = Float32()
        msg.data = norm_angle
        self.angle_pub.publish(msg)

        self.get_logger().info(f"Servo angle: {self.servo_angle:.1f}°, normalized: {norm_angle:.3f}")

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
