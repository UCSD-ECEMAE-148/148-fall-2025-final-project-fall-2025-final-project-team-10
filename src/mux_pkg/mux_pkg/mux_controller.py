import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from twist_mux_msgs.srv import EnableInput


class TwistMuxController(Node):
    def __init__(self):
        super().__init__('twist_mux_controller')

        self.locked_on = False
        self.is_found = False

        self.locked_on_sub = self.create_subscription(Bool, '/locked_on', self.locked_on_cb, 10)
        self.is_found_sub = self.create_subscription(Bool, '/is_found', self.is_found_cb, 10)

        # Wait for twist_mux enable_input services
        self.client_stop = self.create_client(EnableInput, 'twist_mux/enable_input/stop')
        self.client_locate = self.create_client(EnableInput, 'twist_mux/enable_input/locate')
        self.client_explore = self.create_client(EnableInput, 'twist_mux/enable_input/explore')

        self.get_logger().info("Waiting for twist_mux enable_input services...")
        self.client_stop.wait_for_service()
        self.client_locate.wait_for_service()
        self.client_explore.wait_for_service()
        self.get_logger().info("Services available. Ready.")

        self.update_mux_inputs()

    def locked_on_cb(self, msg):
        self.locked_on = msg.data
        self.update_mux_inputs()

    def is_found_cb(self, msg):
        self.is_found = msg.data
        self.update_mux_inputs()

    def update_mux_inputs(self):
        if not self.locked_on:
            self.set_input('stop', True)
            self.set_input('locate', False)
            self.set_input('explore', False)
        elif self.locked_on and not self.is_found:
            self.set_input('stop', False)
            self.set_input('locate', False)
            self.set_input('explore', True)
        elif self.is_found:
            self.set_input('stop', False)
            self.set_input('locate', True)
            self.set_input('explore', False)

    def set_input(self, name, enable):
        client = {
            'stop': self.client_stop,
            'locate': self.client_locate,
            'explore': self.client_explore,
        }[name]

        req = EnableInput.Request()
        req.enable = enable
        future = client.call_async(req)
        future.add_done_callback(lambda f: self.done_cb(f, name, enable))

    def done_cb(self, future, name, enable):
        try:
            res = future.result()
            self.get_logger().info(f"{'Enabled' if enable else 'Disabled'} {name}: success={res.success}")
        except Exception as e:
            self.get_logger().error(f"Service call failed for {name}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TwistMuxController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
