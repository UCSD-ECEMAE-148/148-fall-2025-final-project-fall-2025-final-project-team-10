#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.publisher_ = self.create_publisher(Image, '/camera/raw', 10)
        self.bridge = CvBridge()

        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(640, 480)
        cam.setInterleaved(False)
        cam.setFps(30)

        xout = self.pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("rgb")
        cam.preview.link(xout.input)

        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS
        self.get_logger().info("CameraPublisher node started")

    def timer_callback(self):
        in_rgb = self.q_rgb.tryGet()
        if in_rgb is not None:
            frame = in_rgb.getCvFrame()
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_frame"
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
