#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2
import numpy as np
from pathlib import Path
import time


class ReidNode(Node):
    def __init__(self):
        super().__init__('reid_node')

        # --- Paths ---
        self.WORKSPACE_PATH = "/home/projects/ros2_ws"
        self.BASE_PKG_PATH = f"{self.WORKSPACE_PATH}/src/camera_pkg"
        self.YOLO_BLOB_PATH = f"{self.BASE_PKG_PATH}/blobs/yolov8n_openvino_2022.1_6shave.blob"

        # --- Config ---
        self.INPUT_SIZE = (416, 416)
        self.LABELS = [
            "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train",
            "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter",
            "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear",
            "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase",
            "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
            "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
            "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut",
            "cake", "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet",
            "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
            "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock",
            "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        ]

        # --- ROS Setup ---
        self.pub_img = self.create_publisher(Image, '/camera/reid_output', 10)
        self.pub_found = self.create_publisher(Bool, '/is_found', 10)
        self.pub_pos = self.create_publisher(Float32, '/obj_pos', 10)

        self.create_subscription(Empty, '/reid/lock', self.lock_callback, 10)
        self.create_subscription(Empty, '/reid/reset', self.reset_callback, 10)
        self.bridge = CvBridge()

        # --- Verify YOLO blob ---
        if not Path(self.YOLO_BLOB_PATH).exists():
            raise FileNotFoundError(f"YOLO blob not found: {self.YOLO_BLOB_PATH}")

        # --- Build DepthAI pipeline ---
        self.pipeline = self.create_pipeline()
        try:
            self.device = dai.Device(self.pipeline, maxUsbSpeed=dai.UsbSpeed.SUPER)
        except Exception as e:
            self.get_logger().error(f"Failed to create DepthAI device: {e}")
            self.device = None

        if self.device:
            self.q_rgb = self.device.getOutputQueue("rgb", 1, False)
            self.q_det = self.device.getOutputQueue("det", 1, False)
        else:
            self.q_rgb = self.q_det = None

        # --- Internal state ---
        self.locked_label = None
        self.capturing = True
        self.trigger_lock = False
        self.waiting_after_lock = False
        self.lock_time = None
        self.lock_wait = 5.0  # seconds pause after locking

        # --- Timer @ 5 FPS ---
        self.timer = self.create_timer(0.2, self.run_loop)

        self.get_logger().info("ReID Node ready (5 FPS, no depth)")

    def create_pipeline(self):
        pipeline = dai.Pipeline()

        # Color camera — 1080p sensor but 416×416 preview for performance
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setPreviewSize(*self.INPUT_SIZE)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(5)

        # YOLO detection network
        detect = pipeline.create(dai.node.YoloDetectionNetwork)
        detect.setBlobPath(self.YOLO_BLOB_PATH)
        detect.setConfidenceThreshold(0.5)
        detect.setNumClasses(80)
        detect.setCoordinateSize(4)
        detect.setAnchors([
            10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119,
            116, 90, 156, 198, 373, 326
        ])
        detect.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})

        cam.preview.link(detect.input)

        # Output links
        x_rgb = pipeline.create(dai.node.XLinkOut)
        x_rgb.setStreamName("rgb")
        cam.preview.link(x_rgb.input)

        x_det = pipeline.create(dai.node.XLinkOut)
        x_det.setStreamName("det")
        detect.out.link(x_det.input)

        return pipeline

    def get_latest(self, queue):
        if queue is None:
            return None
        pkt = queue.tryGet()
        while queue.has():
            pkt = queue.tryGet()
        return pkt

    def lock_callback(self, _):
        self.get_logger().info("LOCK REQUESTED")
        self.trigger_lock = True

    def reset_callback(self, _):
        self.get_logger().info("RESET REQUESTED")
        self.locked_label = None
        self.capturing = True
        self.waiting_after_lock = False
        self.lock_time = None

    def run_loop(self):
        if not self.device:
            self.get_logger().warn("No DepthAI device available")
            return

        in_rgb = self.get_latest(self.q_rgb)
        in_det = self.get_latest(self.q_det)
        if not in_rgb or not in_det:
            return

        frame = in_rgb.getCvFrame()
        h, w = frame.shape[:2]
        cx_mid = w // 2

        # Find detection closest to center
        best_rect, best_label, min_dist = None, None, float('inf')
        for d in in_det.detections:
            if d.confidence < 0.5:
                continue
            x1, y1 = int(d.xmin * w), int(d.ymin * h)
            x2, y2 = int(d.xmax * w), int(d.ymax * h)
            c_x = (x1 + x2) // 2
            dist = abs(c_x - cx_mid)
            if dist < min_dist:
                min_dist = dist
                best_rect = (x1, y1, x2, y2)
                best_label = d.label

        found = Bool()
        pos_msg = Float32()

        # Handle locking
        if self.trigger_lock and best_rect:
            self.locked_label = best_label
            self.capturing = False
            self.trigger_lock = False
            self.waiting_after_lock = True
            self.lock_time = time.time()
            label_name = self.LABELS[self.locked_label] if self.locked_label < len(self.LABELS) else str(self.locked_label)
            self.get_logger().info(f"LOCKED ON: {label_name}")
            found.data = False
            self.pub_found.publish(found)
            return

        if self.waiting_after_lock and time.time() - self.lock_time >= self.lock_wait:
            self.waiting_after_lock = False
            self.get_logger().info("Now watching for same object again...")

        # Publish detections
        if not self.capturing and not self.waiting_after_lock and best_rect:
            x1, y1, x2, y2 = best_rect
            if best_label == self.locked_label:
                found.data = True
                self.pub_found.publish(found)

                bbox_cx = (x1 + x2) / 2
                # flipped horizontal direction: left = positive, right = negative
                obj_pos = -(((bbox_cx / w) - 0.5) * 40.0)
                pos_msg.data = obj_pos
                self.pub_pos.publish(pos_msg)

                # draw green box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label_text = self.LABELS[best_label] if best_label < len(self.LABELS) else str(best_label)
                cv2.putText(frame, f"{label_text}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                found.data = False
                self.pub_found.publish(found)
        else:
            found.data = False
            self.pub_found.publish(found)

        # Publish image preview
        frame_small = cv2.resize(frame, (320, 240))
        try:
            msg = self.bridge.cv2_to_imgmsg(frame_small, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.pub_img.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Image publish error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ReidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
