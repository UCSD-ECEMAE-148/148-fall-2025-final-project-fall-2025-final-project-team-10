#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import threading
import sys

class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        
        # Publishers
        self.lock_pub = self.create_publisher(Empty, '/reid/lock', 10)
        self.go_pub = self.create_publisher(Empty, '/mission/go', 10)
        
        # Mux Client (To enable exploration)
        self.param_client = self.create_client(SetParameters, '/twist_mux/set_parameters')
        
        self.get_logger().info("Mission Control Initialized.")

    def send_lock(self):
        print("\n[CMD] >> LOCKING... (Car will hold position)")
        self.lock_pub.publish(Empty())

    def send_go(self):
        print("\n[CMD] >> GO! Authorizing movement...")
        
        # 1. Tell LocateNode it's okay to move
        self.go_pub.publish(Empty())
        
        # 2. Tell Mux to enable Explore (if it was disabled)
        if self.param_client.service_is_ready():
            req = SetParameters.Request()
            p1 = Parameter()
            p1.name = 'topics.explore.enable'
            p1.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
            req.parameters = [p1]
            self.param_client.call_async(req)
        else:
            print("[WARN] Mux service not ready, but Locate signal sent.")

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = MissionControl()
    spinner = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spinner.start()

    print("""
    =========================================
    MISSION CONTROL
    -----------------------------------------
    1. Show Object -> 'l' -> Enter
    2. Hide Object
    3. 'g' -> Enter (Car Starts)
    =========================================
    """)

    try:
        while True:
            cmd = input("Command [l/g/q]: ").strip().lower()
            if cmd == 'l':
                node.send_lock()
            elif cmd == 'g':
                node.send_go()
            elif cmd == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()
