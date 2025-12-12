import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Bool, String
from cv_bridge import CvBridge
import depthai as dai
import cv2
import numpy as np
from pathlib import Path

class LockLabelDetectNode(Node):
    def __init__(self):
        super().__init__('lock_label_detect_node')

        # Paths to blobs - adjust to your setup
        self.WORKSPACE_PATH = "/home/projects/ros2_ws"
        self.BASE_PKG_PATH = f"{self.WORKSPACE_PATH}/src/camera_pkg"
        self.YOLO_BLOB_PATH = f"{self.BASE_PKG_PATH}/blobs/yolov8n_openvino_2022.1_6shave.blob"

        self.INPUT_SIZE = (416, 416)
        self.LABELS = [ "person", "bicycle", "car", "motorbike", "aeroplane", "bus",
                        "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign",
                        "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
                        "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
                        "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
                        "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
                        "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
                        "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
                        "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
                        "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor",
                        "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
                        "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
                        "scissors", "teddy bear", "hair drier", "toothbrush" ]

        # ROS publishers and subscribers
        self.publisher_img = self.create_publisher(Image, '/camera/reid_output', 10)
        self.publisher_label = self.create_publisher(String, '/detected_label', 10)
        self.publisher_is_found = self.create_publisher(Bool, '/is_found', 10)

        self.sub_lock = self.create_subscription(Empty, '/reid/lock', self.lock_callback, 10)
        self.sub_reset = self.create_subscription(Empty, '/reid/reset', self.reset_callback, 10)

        self.bridge = CvBridge()

        # Blob file check
        if not Path(self.YOLO_BLOB_PATH).exists():
            self.get_logger().error(f"YOLO blob missing: {self.YOLO_BLOB_PATH}")
            raise FileNotFoundError(f"YOLO blob not found: {self.YOLO_BLOB_PATH}")

        # Setup DepthAI pipeline
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

        # Variables for locking
        self.locked_label = None  # Stores locked label index (int)
        self.locked_label_name = None  # Stores locked label string
        self.is_found = False

        # Run loop timer
        self.timer = self.create_timer(0.1, self.run_loop)  # 10 Hz
        self.get_logger().info("LockLabelDetectNode ready.")

    def create_pipeline(self):
        pipeline = dai.Pipeline()
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(*self.INPUT_SIZE)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(10)

        detect = pipeline.create(dai.node.YoloDetectionNetwork)
        detect.setBlobPath(self.YOLO_BLOB_PATH)
        detect.setConfidenceThreshold(0.5)
        detect.setNumClasses(80)
        detect.setCoordinateSize(4)
        detect.setAnchors([10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326])
        detect.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})

        x_rgb = pipeline.create(dai.node.XLinkOut)
        x_rgb.setStreamName("rgb")
        cam.preview.link(detect.input)
        cam.preview.link(x_rgb.input)

        x_det = pipeline.create(dai.node.XLinkOut)
        x_det.setStreamName("det")
        detect.out.link(x_det.input)

        return pipeline

    def get_latest(self, queue):
        if queue is None:
            return None
        pkt = queue.tryGet()
        while True:
            next_pkt = queue.tryGet()
            if next_pkt is None:
                break
            pkt = next_pkt
        return pkt

    def lock_callback(self, msg):
        # When lock message received, lock onto the detection closest to center
        self.get_logger().info("LOCK REQUESTED")
        if self.latest_detections:
            # Find closest detection to center
            h, w = self.latest_frame.shape[:2]
            center_x, center_y = w // 2, h // 2
            min_dist = float('inf')
            lock_det = None
            for d in self.latest_detections:
                x1, y1 = int(d.xmin * w), int(d.ymin * h)
                x2, y2 = int(d.xmax * w), int(d.ymax * h)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                dist = ((cx - center_x)**2 + (cy - center_y)**2)**0.5
                if dist < min_dist:
                    min_dist = dist
                    lock_det = d
            if lock_det is not None:
                self.locked_label = lock_det.label
                self.locked_label_name = self.LABELS[lock_det.label] if lock_det.label < len(self.LABELS) else str(lock_det.label)
                self.get_logger().info(f"LOCKED on label: {self.locked_label_name}")
            else:
                self.get_logger().warn("No detections to lock onto")
        else:
            self.get_logger().warn("No detections available to lock onto")

    def reset_callback(self, msg):
        self.get_logger().info("RESET REQUESTED")
        self.locked_label = None
        self.locked_label_name = None
        self.is_found = False

    def run_loop(self):
        if self.device is None:
            self.get_logger().warn("No DepthAI device available, skipping frame.")
            return

        in_rgb = self.get_latest(self.q_rgb)
        in_det = self.get_latest(self.q_det)
        if in_rgb is None or in_det is None:
            self.get_logger().warn("Missing frame or detection, skipping.")
            return

        frame = in_rgb.getCvFrame()
        h, w = frame.shape[:2]

        self.latest_frame = frame
        self.latest_detections = in_det.detections

        # Annotate detections on frame
        for d in in_det.detections:
            if d.confidence < 0.5:
                continue
            x1, y1 = int(d.xmin * w), int(d.ymin * h)
            x2, y2 = int(d.xmax * w), int(d.ymax * h)
            label_str = self.LABELS[d.label] if d.label < len(self.LABELS) else str(d.label)

            # Draw bounding box and label
            color = (0, 255, 255)
            # If locked and this detection matches locked label, use green box
            if self.locked_label is not None and d.label == self.locked_label:
                color = (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{label_str}: {d.confidence:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Publish detected_label: locked label or empty string if none
        label_msg = String()
        label_msg.data = self.locked_label_name if self.locked_label_name is not None else ""
        self.publisher_label.publish(label_msg)

        # Check if locked label currently seen
        found = False
        if self.locked_label is not None:
            for d in in_det.detections:
                if d.confidence >= 0.5 and d.label == self.locked_label:
                    found = True
                    break

        # Publish is_found bool
        is_found_msg = Bool()
        is_found_msg.data = found
        self.publisher_is_found.publish(is_found_msg)

        # Publish annotated frame
        frame_small = cv2.resize(frame, (320, 240))
        try:
            msg_img = self.bridge.cv2_to_imgmsg(frame_small, encoding="bgr8")
            msg_img.header.stamp = self.get_clock().now().to_msg()
            msg_img.header.frame_id = "camera_frame"
            self.publisher_img.publish(msg_img)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LockLabelDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
