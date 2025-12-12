import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class StopPublisher(Node):
    def __init__(self):
        super().__init__('stop_publisher')
        self.pub = self.create_publisher(Twist, 'stop/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_stop)

    def publish_stop(self):
        msg = Twist()  # zero velocity
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StopPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
