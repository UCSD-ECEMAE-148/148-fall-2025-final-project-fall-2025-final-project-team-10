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

        # --- Configuration ---
        self.WORKSPACE_PATH = "/home/projects/ros2_ws"
        self.BASE_PKG_PATH = f"{self.WORKSPACE_PATH}/src/camera_pkg"
        self.YOLO_BLOB_PATH = f"{self.BASE_PKG_PATH}/blobs/yolov8n_openvino_2022.1_6shave.blob"
        self.EMBEDDER_BLOB_PATH = f"{self.BASE_PKG_PATH}/blobs/mobilenet-v2-1.0-224_openvino_2022.1_4shave.blob"
        self.MATCH_THRESHOLD = 0.60
        self.INPUT_SIZE = (416, 416)
        self.EMBEDDER_SIZE = (224, 224)
        self.LABELS = [  # COCO classes
            "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
            "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
            "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
            "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
            "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
            "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa",
            "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote",
            "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
            "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        ]

        # --- ROS Publishers and Subscribers ---
        self.publisher_ = self.create_publisher(Image, '/camera/reid_output', 10)
        self.is_found_pub = self.create_publisher(Bool, '/is_found', 10)
        self.obj_pos_pub = self.create_publisher(Float32, '/obj_pos', 10)
        self.obj_area_pub = self.create_publisher(Float32, '/obj_area', 10)  # <-- obj_area publisher

        self.sub_lock = self.create_subscription(Empty, '/reid/lock', self.lock_callback, 10)
        self.bridge = CvBridge()

        # Check blobs
        if not Path(self.YOLO_BLOB_PATH).exists():
            self.get_logger().error(f"YOLO MISSING: {self.YOLO_BLOB_PATH}")
            raise FileNotFoundError(f"YOLO blob not found: {self.YOLO_BLOB_PATH}")
        if not Path(self.EMBEDDER_BLOB_PATH).exists():
            self.get_logger().error(f"EMBEDDER MISSING: {self.EMBEDDER_BLOB_PATH}")
            raise FileNotFoundError(f"Embedder blob not found: {self.EMBEDDER_BLOB_PATH}")

        # Setup pipeline
        self.pipeline = self.create_pipeline()
        try:
            self.device = dai.Device(self.pipeline, maxUsbSpeed=dai.UsbSpeed.SUPER)
        except Exception as e:
            self.get_logger().error(f"Failed to create DepthAI device: {e}")
            self.device = None

        if self.device:
            self.q_rgb = self.device.getOutputQueue("rgb", maxSize=1, blocking=False)
            self.q_det = self.device.getOutputQueue("det", maxSize=1, blocking=False)
        else:
            self.q_rgb = None
            self.q_det = None

        self.locked_label = None
        self.capturing = True
        self.trigger_lock = False

        self.timer = self.create_timer(0.05, self.run_loop)  # 20 Hz approx
        self.get_logger().info("ReID Node ready (no depth tracking)")

    def create_pipeline(self):
        pipeline = dai.Pipeline()

        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(*self.INPUT_SIZE)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(20)

        detect = pipeline.create(dai.node.YoloDetectionNetwork)
        detect.setBlobPath(self.YOLO_BLOB_PATH)
        detect.setConfidenceThreshold(0.5)
        detect.setNumClasses(80)
        detect.setCoordinateSize(4)
        detect.setAnchors([10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326])
        detect.setAnchorMasks({"side26": [1,2,3], "side13": [3,4,5]})

        x_rgb = pipeline.create(dai.node.XLinkOut)
        x_rgb.setStreamName("rgb")

        x_det = pipeline.create(dai.node.XLinkOut)
        x_det.setStreamName("det")

        cam.preview.link(detect.input)
        cam.preview.link(x_rgb.input)
        detect.out.link(x_det.input)

        return pipeline

    def get_latest(self, queue):
        if queue is None:
            return None
        pkt = queue.tryGet()
        if pkt is not None:
            while True:
                next_pkt = queue.tryGet()
                if next_pkt is None:
                    break
                pkt = next_pkt
        return pkt

    def lock_callback(self, msg):
        self.get_logger().info("LOCK REQUESTED")
        self.trigger_lock = True

    def run_loop(self):
        if self.device is None:
            self.get_logger().warn("No DepthAI device available, publishing dummy image.")
            dummy_img = np.zeros((240, 320, 3), dtype=np.uint8)
            try:
                msg = self.bridge.cv2_to_imgmsg(dummy_img, "bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_frame"
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish dummy image: {e}")
            return

        in_rgb = self.get_latest(self.q_rgb)
        in_det = self.get_latest(self.q_det)

        if in_rgb is None:
            self.get_logger().warn("No RGB frame received")
            return

        frame = in_rgb.getCvFrame()
        h, w = frame.shape[:2]

        best_rect = None
        best_label = None
        min_dist = float('inf')
        center_x, center_y = w // 2, h // 2

        if in_det is not None:
            for d in in_det.detections:
                if d.confidence < 0.5:
                    continue
                # Only consider detections with locked label when not capturing
                if not self.capturing and d.label != self.locked_label:
                    continue

                x1, y1, x2, y2 = int(d.xmin * w), int(d.ymin * h), int(d.xmax * w), int(d.ymax * h)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                dist = ((cx - center_x) ** 2 + (cy - center_y) ** 2) ** 0.5

                if dist < min_dist:
                    min_dist = dist
                    best_rect = (max(0, x1), max(0, y1), min(w, x2), min(h, y2))
                    best_label = d.label

        if self.trigger_lock and best_rect is not None:
            label_txt = self.LABELS[best_label] if best_label < len(self.LABELS) else str(best_label)
            self.get_logger().info(f"LOCKED: {label_txt}")
            self.locked_label = best_label
            self.capturing = False
            self.trigger_lock = False

        # Draw and publish
        if best_rect is not None:
            x1, y1, x2, y2 = best_rect
            label_txt = self.LABELS[best_label] if best_label < len(self.LABELS) else str(best_label)

            # Set color based on capture and locked label status
            color = (0, 255, 0) if (not self.capturing and best_label == self.locked_label) else (0, 255, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
            cv2.putText(frame, f"{label_txt}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Publish obj_area only if locked and label matches locked label
            if not self.capturing and best_label == self.locked_label:
                bbox_area = (x2 - x1) * (y2 - y1)
                total_area = w * h
                area_fraction = bbox_area / total_area if total_area > 0 else 0.0
                area_msg = Float32()
                area_msg.data = float(area_fraction)
                self.obj_area_pub.publish(area_msg)
            else:
                # Not locked or label mismatch — publish zero area
                area_msg = Float32()
                area_msg.data = 0.0
                self.obj_area_pub.publish(area_msg)

            if not self.capturing and best_label == self.locked_label:
                is_found_msg = Bool()
                is_found_msg.data = True
                self.is_found_pub.publish(is_found_msg)

                bbox_center_x = (x1 + x2) / 2
                norm_pos = ((bbox_center_x / w) - 0.5) * 40  # -20 to 20
                pos_msg = Float32()
                pos_msg.data = float(norm_pos)
                self.obj_pos_pub.publish(pos_msg)
            else:
                is_found_msg = Bool()
                is_found_msg.data = False
                self.is_found_pub.publish(is_found_msg)

        else:
            # No detections at all, publish False and zero area
            is_found_msg = Bool()
            is_found_msg.data = False
            self.is_found_pub.publish(is_found_msg)

            area_msg = Float32()
            area_msg.data = 0.0
            self.obj_area_pub.publish(area_msg)

        # Resize for publishing preview (to reduce bandwidth)
        frame_small = cv2.resize(frame, (320, 240))
        try:
            msg = self.bridge.cv2_to_imgmsg(frame_small, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Publish Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ReidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
