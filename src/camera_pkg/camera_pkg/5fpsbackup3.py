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
        self.LABELS = [
            "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train",
            "truck", "boat", "traffic light", "fire hydrant", "stop sign",
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
            "scissors", "teddy bear", "hair drier", "toothbrush"
        ]

        # --- ROS Setup ---
        self.publisher_ = self.create_publisher(Image, '/camera/reid_output', 10)
        self.publisher_label = self.create_publisher(Bool, '/is_found', 10)  # Publish True when locked label seen
        self.publisher_obj_pos = self.create_publisher(Float32, '/obj_pos', 10)  # NEW: horizontal position
        self.publisher_obj_depth = self.create_publisher(Float32, '/obj_depth', 10)  # NEW: depth at bbox center

        self.sub_lock = self.create_subscription(Empty, '/reid/lock', self.lock_callback, 10)
        self.sub_reset = self.create_subscription(Empty, '/reid/reset', self.reset_callback, 10)

        self.bridge = CvBridge()

        # Check blob files
        if not Path(self.YOLO_BLOB_PATH).exists():
            self.get_logger().error(f"YOLO blob not found: {self.YOLO_BLOB_PATH}")
            raise FileNotFoundError(f"YOLO blob not found: {self.YOLO_BLOB_PATH}")

        if not Path(self.EMBEDDER_BLOB_PATH).exists():
            self.get_logger().error(f"Embedder blob not found: {self.EMBEDDER_BLOB_PATH}")
            raise FileNotFoundError(f"Embedder blob not found: {self.EMBEDDER_BLOB_PATH}")

        # Create pipeline with depth node (MODIFIED)
        self.pipeline = self.create_pipeline()

        try:
            self.device = dai.Device(self.pipeline, maxUsbSpeed=dai.UsbSpeed.SUPER)
        except Exception as e:
            self.get_logger().error(f"Failed to create DepthAI device: {e}")
            self.device = None

        if self.device:
            self.q_rgb = self.device.getOutputQueue("rgb", maxSize=1, blocking=False)
            self.q_det = self.device.getOutputQueue("det", maxSize=1, blocking=False)
            self.q_emb_in = self.device.getInputQueue("embedder_in")
            self.q_emb_out = self.device.getOutputQueue("embed", maxSize=1, blocking=True)
            self.q_depth = self.device.getOutputQueue("depth", maxSize=1, blocking=False)  # NEW
        else:
            self.q_rgb = None
            self.q_det = None
            self.q_emb_in = None
            self.q_emb_out = None
            self.q_depth = None

        self.ref_label = None
        self.capturing = True
        self.trigger_lock = False

        self.timer = self.create_timer(0.01, self.run_loop)
        self.get_logger().info("ReID Node ready (with depth and obj_pos publishing)")

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
        detect.setAnchors([10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59,
                           119, 116, 90, 156, 198, 373, 326])
        detect.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})

        embedder_in = pipeline.create(dai.node.XLinkIn)
        embedder_in.setStreamName("embedder_in")

        embedder_nn = pipeline.create(dai.node.NeuralNetwork)
        embedder_nn.setBlobPath(self.EMBEDDER_BLOB_PATH)
        embedder_in.out.link(embedder_nn.input)

        x_rgb = pipeline.create(dai.node.XLinkOut)
        x_rgb.setStreamName("rgb")

        x_det = pipeline.create(dai.node.XLinkOut)
        x_det.setStreamName("det")

        x_embed = pipeline.create(dai.node.XLinkOut)
        x_embed.setStreamName("embed")

        # Depth node for depth map (NEW)
        stereo = pipeline.create(dai.node.StereoDepth)
        left = pipeline.create(dai.node.MonoCamera)
        right = pipeline.create(dai.node.MonoCamera)
        left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        stereo.setConfidenceThreshold(200)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setOutputDepth(True)
        stereo.setOutputRectified(False)

        left.out.link(stereo.left)
        right.out.link(stereo.right)

        x_depth = pipeline.create(dai.node.XLinkOut)
        x_depth.setStreamName("depth")

        stereo.depth.link(x_depth.input)

        # Link nodes
        cam.preview.link(detect.input)
        cam.preview.link(x_rgb.input)
        detect.out.link(x_det.input)
        embedder_nn.out.link(x_embed.input)

        return pipeline

    def get_embedding(self, crop):
        if crop.size == 0:
            return None
        resized = cv2.resize(crop, self.EMBEDDER_SIZE)
        img_data = dai.ImgFrame()
        # The NN expects interleaved BGR888 data, flatten with transpose
        img_data.setData(resized.transpose(2, 0, 1).flatten())
        img_data.setWidth(self.EMBEDDER_SIZE[0])
        img_data.setHeight(self.EMBEDDER_SIZE[1])
        # Using interleaved type
        img_data.setType(dai.ImgFrame.Type.BGR888)
        self.q_emb_in.send(img_data)
        in_nn = self.q_emb_out.get()
        return np.array(in_nn.getFirstLayerFp16())

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

    def cosine_similarity(self, a, b):
        return np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-6)

    def lock_callback(self, msg):
        self.get_logger().info("LOCK REQUESTED")
        self.trigger_lock = True

    def reset_callback(self, msg):
        self.get_logger().info("RESET REQUESTED")
        self.capturing = True
        self.ref_label = None

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
        in_depth = self.get_latest(self.q_depth)  # NEW depth frame

        if in_rgb is None:
            self.get_logger().warn("No RGB frame received")
            return

        frame = in_rgb.getCvFrame()
        h, w = frame.shape[:2]

        best_rect = None
        best_label = None

        found = False  # NEW: whether locked label is found this frame

        if in_det is not None:
            center_x, center_y = w // 2, h // 2
            min_dist = float('inf')
            for d in in_det.detections:
                if d.confidence < 0.5:
                    continue
                if not self.capturing and d.label != self.ref_label:
                    continue
                x1, y1 = int(d.xmin * w), int(d.ymin * h)
                x2, y2 = int(d.xmax * w), int(d.ymax * h)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                dist = ((cx - center_x) ** 2 + (cy - center_y) ** 2) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    best_rect = (max(0, x1), max(0, y1), min(w, x2), min(h, y2))
                    best_label = d.label

            if best_rect is not None:
                bx1, by1, bx2, by2 = best_rect
                label_txt = self.LABELS[best_label] if best_label < len(self.LABELS) else str(best_label)
                cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 255, 255), 3)
                cv2.putText(frame, f"TARGET: {label_txt}", (bx1, by1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                if self.trigger_lock:
                    self.get_logger().info(f"LOCKED: {label_txt}")
                    self.ref_label = best_label
                    self.capturing = False
                    self.trigger_lock = False

                # NEW: If locked on label, publish is_found = True and obj_pos, obj_depth
                if not self.capturing and best_label == self.ref_label:
                    found = True

                    # Calculate horizontal position scaled to [-20, 20]
                    bbox_cx = (bx1 + bx2) / 2
                    pos_norm = (bbox_cx - (w / 2)) / (w / 2)  # range approx [-1,1]
                    pos_scaled = pos_norm * 20  # scale to [-20, 20]

                    # Get depth at bbox center from depth frame (if available)
                    depth_value = -1.0  # default if no depth
                    if in_depth is not None:
                        depth_frame = in_depth.getFrame()  # depth_frame is uint16, depth in mm
                        depth_x = int(bbox_cx * depth_frame.shape[1] / w)
                        bbox_cy = (by1 + by2) / 2
                        depth_y = int(bbox_cy * depth_frame.shape[0] / h)
                        # Check bounds
                        if 0 <= depth_x < depth_frame.shape[1] and 0 <= depth_y < depth_frame.shape[0]:
                            depth_mm = depth_frame[depth_y, depth_x]
                            if depth_mm != 0:
                                depth_value = float(depth_mm) / 1000.0  # convert mm to meters

                    # Publish horizontal position
                    msg_pos = Float32()
                    msg_pos.data = float(pos_scaled)
                    self.publisher_obj_pos.publish(msg_pos)

                    # Publish depth
                    msg_depth = Float32()
                    msg_depth.data = depth_value
                    self.publisher_obj_depth.publish(msg_depth)

        # Publish /is_found boolean
        msg_found = Bool()
        msg_found.data = found
        self.publisher_label.publish(msg_found)

        # Publish output image resized for Foxglove or WiFi use
        frame_small = cv2.resize(frame, (320, 240))
        try:
            msg = self.bridge.cv2_to_imgmsg(frame_small, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.publisher_.publish(msg)
            self.get_logger().debug("Published processed frame")
        except Exception as e:
            self.get_logger().error(f"Publish Error: {e}")

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
