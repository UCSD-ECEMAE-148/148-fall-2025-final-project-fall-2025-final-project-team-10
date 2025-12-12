#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Import your depthai and model-related stuff here (YOLO, embedding, etc.)

class ProcessingNode(Node):
    def __init__(self):
        super().__init__('processing_node')
        self.subscriber_ = self.create_subscription(Image, '/camera/raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Image, '/camera/reid_output', 10)
        self.bridge = CvBridge()

        # Initialize your model, blobs, etc here
        # For now, we just draw a dummy rectangle to test

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CVBridge convert error: {e}")
            return

        # Dummy processing: Draw a green rectangle in center
        h, w = frame.shape[:2]
        cv2.rectangle(frame, (w//4, h//4), (3*w//4, 3*h//4), (0,255,0), 3)

        # TODO: Add your YOLO detection and embedding code here,
        # modify frame accordingly

        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = "camera_frame"
            self.publisher_.publish(out_msg)
            self.get_logger().debug("Published processed frame")
        except Exception as e:
            self.get_logger().error(f"Failed to publish processed frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
