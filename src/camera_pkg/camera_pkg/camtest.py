#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2
import time
import numpy as np

class DepthAIDebugPublisher(Node):
    def __init__(self):
        super().__init__('depthai_debug_pub')

        self.publisher_ = self.create_publisher(Image, '/camera/raw', 10)
        self.bridge = CvBridge()

        # --- Build minimal working pipeline ---
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(640, 480)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        xout = self.pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("video")
        cam.preview.link(xout.input)

        # --- Try connecting to the DepthAI device ---
        self.device = None
        for speed in [dai.UsbSpeed.SUPER, dai.UsbSpeed.HIGH]:
            try:
                self.get_logger().info(f"Trying DepthAI connection at {speed.name}...")
                self.device = dai.Device(self.pipeline, maxUsbSpeed=speed)
                break
            except Exception as e:
                self.get_logger().warn(f"Failed to connect at {speed.name}: {e}")
                time.sleep(1)

        if self.device is None:
            self.get_logger().error("❌ Could not connect to DepthAI device at any USB speed.")
            return

        # --- Diagnostics: print all known info ---
        connected = self.device.getConnectedCameras()
        mxid = self.device.getMxId()
        usb_speed = self.device.getUsbSpeed()
        available_streams = self.device.getOutputQueueNames()

        self.get_logger().info(f"✅ Connected to OAK-D device {mxid}")
        self.get_logger().info(f"Connected cameras: {connected}")
        self.get_logger().info(f"USB speed: {usb_speed}")
        self.get_logger().info(f"Available output streams: {available_streams}")

        # --- Get queue ---
        if "video" not in available_streams:
            self.get_logger().error("⚠️ 'video' stream not found in pipeline output names!")
        self.q = self.device.getOutputQueue("video", 4, False)

        # Give the pipeline a moment to spin up
        time.sleep(0.5)

        self.timer = self.create_timer(0.05, self.publish_frame)
        self.frame_count = 0
        self.start_time = time.time()
        self.get_logger().info("DepthAI debug publisher node started")

    def publish_frame(self):
        if not hasattr(self, 'q') or self.q is None:
            self.get_logger().warn("Queue not initialized.")
            return

        frame_packet = self.q.tryGet()
        if frame_packet is None:
            self.get_logger().warn("No RGB frame received", throttle_duration_sec=5.0)
            return

        frame = frame_packet.getCvFrame()
        if frame is None or frame.size == 0:
            self.get_logger().warn("Empty frame received")
            return

        # Count FPS
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            elapsed = time.time() - self.start_time
            fps = self.frame_count / elapsed
            self.get_logger().info(f"📸 Receiving frames at ~{fps:.2f} FPS")

        # Publish to ROS
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthAIDebugPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
