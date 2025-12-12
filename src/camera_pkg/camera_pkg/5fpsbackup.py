#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String
from cv_bridge import CvBridge
import depthai as dai
import cv2
import numpy as np
from pathlib import Path


class ReidNode(Node):
    def __init__(self):
        super().__init__('reid_node')

        # --- CONFIG ---
        self.WORKSPACE_PATH = "/home/projects/ros2_ws"
        self.BASE_PKG_PATH = f"{self.WORKSPACE_PATH}/src/camera_pkg"
        self.YOLO_BLOB_PATH = f"{self.BASE_PKG_PATH}/blobs/yolov8n_openvino_2022.1_6shave.blob"
        self.EMBEDDER_BLOB_PATH = f"{self.BASE_PKG_PATH}/blobs/mobilenet-v2-1.0-224_openvino_2022.1_4shave.blob"
        self.MATCH_THRESHOLD = 0.60
        self.INPUT_SIZE = (416, 416)
        self.EMBEDDER_SIZE = (224, 224)

        self.LABELS = [
            "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck",
            "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
            "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
            "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
            "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
            "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
            "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa",
            "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
            "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
            "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
            "toothbrush"
        ]


        # --- ROS2 SETUP ---
        self.pub_image = self.create_publisher(Image, '/camera/reid_output', 10)
        self.pub_label = self.create_publisher(String, '/camera/reid_label', 10)
        self.sub_lock = self.create_subscription(Empty, '/reid/lock', self.lock_callback, 10)
        self.sub_reset = self.create_subscription(Empty, '/reid/reset', self.reset_callback, 10)
        self.bridge = CvBridge()

        # --- CHECK BLOBS ---
        for path in [self.YOLO_BLOB_PATH, self.EMBEDDER_BLOB_PATH]:
            if not Path(path).exists():
                self.get_logger().error(f"Missing blob: {path}")
                raise FileNotFoundError(f"Blob not found: {path}")

        # --- PIPELINE ---
        self.pipeline = self.create_pipeline()
        try:
            self.device = dai.Device(self.pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)
        except Exception as e:
            self.get_logger().error(f"Failed to create DepthAI device: {e}")
            self.device = None

        if self.device:
            self.q_rgb = self.device.getOutputQueue("rgb", maxSize=1, blocking=False)
            self.q_det = self.device.getOutputQueue("det", maxSize=1, blocking=False)
            self.q_emb_in = self.device.getInputQueue("embedder_in")
            self.q_emb_out = self.device.getOutputQueue("embed", maxSize=1, blocking=False)
        else:
            self.q_rgb = self.q_det = self.q_emb_in = self.q_emb_out = None

        # --- STATE ---
        self.ref_emb = None
        self.ref_img = None
        self.ref_label = None
        self.capturing = True
        self.trigger_lock = False

        self.timer = self.create_timer(0.05, self.run_loop)
        self.get_logger().info("ReID Node Ready (5 Hz, with label publishing)")

    # ---------------- PIPELINE ----------------
    def create_pipeline(self):
        pipeline = dai.Pipeline()

        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(*self.INPUT_SIZE)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(5)

        manip = pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setResize(*self.INPUT_SIZE)
        manip.setMaxOutputFrameSize(self.INPUT_SIZE[0] * self.INPUT_SIZE[1] * 3)
        manip.setWaitForConfigInput(False)
        cam.preview.link(manip.inputImage)

        detect = pipeline.create(dai.node.YoloDetectionNetwork)
        detect.setBlobPath(self.YOLO_BLOB_PATH)
        detect.setConfidenceThreshold(0.5)
        detect.setNumClasses(80)
        detect.setCoordinateSize(4)
        detect.setAnchors([10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119,
                           116, 90, 156, 198, 373, 326])
        detect.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})
        detect.input.setBlocking(False)
        detect.input.setQueueSize(1)
        manip.out.link(detect.input)

        embedder_in = pipeline.create(dai.node.XLinkIn)
        embedder_in.setStreamName("embedder_in")
        embedder_nn = pipeline.create(dai.node.NeuralNetwork)
        embedder_nn.setBlobPath(self.EMBEDDER_BLOB_PATH)
        embedder_in.out.link(embedder_nn.input)

        x_rgb = pipeline.create(dai.node.XLinkOut)
        x_rgb.setStreamName("rgb")
        cam.video.link(x_rgb.input)

        x_det = pipeline.create(dai.node.XLinkOut)
        x_det.setStreamName("det")
        detect.out.link(x_det.input)

        x_embed = pipeline.create(dai.node.XLinkOut)
        x_embed.setStreamName("embed")
        embedder_nn.out.link(x_embed.input)

        return pipeline

    # ---------------- HELPERS ----------------
    def get_embedding(self, crop):
        if crop.size == 0:
            return None
        resized = cv2.resize(crop, self.EMBEDDER_SIZE)
        img_data = dai.ImgFrame()
        img_data.setData(resized.transpose(2, 0, 1).flatten())
        img_data.setWidth(self.EMBEDDER_SIZE[0])
        img_data.setHeight(self.EMBEDDER_SIZE[1])
        img_data.setType(dai.ImgFrame.Type.BGR888p)
        self.q_emb_in.send(img_data)
        in_nn = self.q_emb_out.tryGet()
        if in_nn is None:
            return None
        return np.array(in_nn.getFirstLayerFp16())

    def get_latest(self, queue):
        if queue is None:
            return None
        pkt = queue.tryGet()
        while pkt:
            nxt = queue.tryGet()
            if not nxt:
                break
            pkt = nxt
        return pkt

    def cosine_similarity(self, a, b):
        return np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-6)

    # ---------------- CALLBACKS ----------------
    def lock_callback(self, _):
        self.get_logger().info("LOCK REQUESTED")
        self.trigger_lock = True

    def reset_callback(self, _):
        self.get_logger().info("RESET REQUESTED")
        self.capturing = True
        self.ref_emb = None

    # ---------------- MAIN LOOP ----------------
    def run_loop(self):
        if self.device is None:
            return

        in_rgb = self.get_latest(self.q_rgb)
        in_det = self.get_latest(self.q_det)
        if in_rgb is None:
            return

        frame = in_rgb.getCvFrame()
        h, w = frame.shape[:2]
        best_rect = None
        best_label = None

        # --- Detection selection ---
        if in_det:
            center_x, center_y = w // 2, h // 2
            min_dist = float('inf')
            for d in in_det.detections:
                if d.confidence < 0.5:
                    continue
                x1, y1 = int(d.xmin * w), int(d.ymin * h)
                x2, y2 = int(d.xmax * w), int(d.ymax * h)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                dist = ((cx - center_x)**2 + (cy - center_y)**2)**0.5
                if dist < min_dist:
                    min_dist = dist
                    best_rect = (x1, y1, x2, y2)
                    best_label = d.label

        if best_rect and best_label is not None:
            label_txt = self.LABELS[best_label] if best_label < len(self.LABELS) else str(best_label)
            x1, y1, x2, y2 = best_rect
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 3)
            cv2.putText(frame, label_txt, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # --- Publish label ---
            label_msg = String()
            label_msg.data = label_txt
            self.pub_label.publish(label_msg)

        # --- Publish image ---
        frame_small = cv2.resize(frame, (320, 240))
        msg = self.bridge.cv2_to_imgmsg(frame_small, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        self.pub_image.publish(msg)


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


if __name__ == '__main__':
    main()
