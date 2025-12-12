#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import depthai as dai
import cv2
import numpy as np
from pathlib import Path

class ReidNode(Node):
    def __init__(self):
        super().__init__('reid_node')
        
        # --- 1. CONFIGURATION ---
        self.WORKSPACE_PATH = "/home/projects/ros2_ws"
        self.BASE_PKG_PATH = f"{self.WORKSPACE_PATH}/src/camera_pkg"
        
        self.YOLO_BLOB_PATH = f"{self.BASE_PKG_PATH}/blobs/yolov8n_openvino_2022.1_6shave.blob"
        self.EMBEDDER_BLOB_PATH = f"{self.BASE_PKG_PATH}/blobs/mobilenet-v2-1.0-224_openvino_2022.1_4shave.blob"
        
        self.MATCH_THRESHOLD = 0.60
        self.INPUT_SIZE = (416, 416)
        self.EMBEDDER_SIZE = (224, 224)
        
        self.LABELS = [
            "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light",
            "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
            "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
            "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa",
            "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard",
            "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
            "scissors", "teddy bear", "hair drier", "toothbrush"
        ]

        # --- 2. ROS SETUP ---
        self.publisher_ = self.create_publisher(Image, '/camera/reid_output', 10)
        self.sub_lock = self.create_subscription(Empty, '/reid/lock', self.lock_callback, 10)
        self.sub_reset = self.create_subscription(Empty, '/reid/reset', self.reset_callback, 10)
        
        self.bridge = CvBridge()

        # Check blob files exist — fail early if missing
        if not Path(self.YOLO_BLOB_PATH).exists():
            self.get_logger().error(f"YOLO MISSING: {self.YOLO_BLOB_PATH}")
            raise FileNotFoundError(f"YOLO blob not found: {self.YOLO_BLOB_PATH}")
        if not Path(self.EMBEDDER_BLOB_PATH).exists():
            self.get_logger().error(f"EMBEDDER MISSING: {self.EMBEDDER_BLOB_PATH}")
            raise FileNotFoundError(f"Embedder blob not found: {self.EMBEDDER_BLOB_PATH}")

        # --- 3. OAK-D PIPELINE ---
        self.pipeline = self.create_pipeline()
        
        try:
            # *** USB 3.0 mode (SUPER) for best performance ***
            self.device = dai.Device(self.pipeline, maxUsbSpeed=dai.UsbSpeed.SUPER)
        except Exception as e:
            self.get_logger().error(f"Failed to create DepthAI device: {e}")
            self.device = None

        if self.device:
            self.q_rgb = self.device.getOutputQueue("rgb", maxSize=1, blocking=False)
            self.q_det = self.device.getOutputQueue("det", maxSize=1, blocking=False)
            self.q_emb_in = self.device.getInputQueue("embedder_in")
            self.q_emb_out = self.device.getOutputQueue("embed", maxSize=1, blocking=False)
        else:
            self.q_rgb = None
            self.q_det = None
            self.q_emb_in = None
            self.q_emb_out = None

        self.ref_emb = None
        self.ref_img = None
        self.ref_label = None
        self.capturing = True 
        self.trigger_lock = False

        self.timer = self.create_timer(0.01, self.run_loop)
        self.get_logger().info("ReID Node Ready (USB 3.0 Mode).")

    def create_pipeline(self):
        pipeline = dai.Pipeline()
        
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(*self.INPUT_SIZE)
       # cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(5)
        
        detect = pipeline.create(dai.node.YoloDetectionNetwork)
        detect.setBlobPath(self.YOLO_BLOB_PATH)
        detect.setConfidenceThreshold(0.5)
        detect.setNumClasses(80)
        detect.setCoordinateSize(4)
        detect.setAnchors([10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326])
        detect.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})
        
        embedder_in = pipeline.create(dai.node.XLinkIn)
        embedder_in.setStreamName("embedder_in")
        
        embedder_nn = pipeline.create(dai.node.NeuralNetwork)
        embedder_nn.setBlobPath(self.EMBEDDER_BLOB_PATH)
        embedder_in.out.link(embedder_nn.input)
        
        x_rgb = pipeline.create(dai.node.XLinkOut)
        x_rgb.setStreamName("rgb")
        cam.preview.link(detect.input)
        cam.preview.link(x_rgb.input)
        
        x_det = pipeline.create(dai.node.XLinkOut)
        x_det.setStreamName("det")
        detect.out.link(x_det.input)
        
        x_embed = pipeline.create(dai.node.XLinkOut)
        x_embed.setStreamName("embed")
        embedder_nn.out.link(x_embed.input)
        
        return pipeline

    def get_embedding(self, crop):
        if crop.size == 0: 
            return None
        resized = cv2.resize(crop, self.EMBEDDER_SIZE)
        img_data = dai.ImgFrame()
        # Use numpy transpose here instead of cv2.transpose
        img_data.setData(resized.transpose(2, 0, 1).flatten())
        img_data.setWidth(self.EMBEDDER_SIZE[0])
        img_data.setHeight(self.EMBEDDER_SIZE[1])
        img_data.setType(dai.ImgFrame.Type.BGR888p) 
        
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
        self.ref_emb = None

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

        if in_det is None:
            self.get_logger().warn("No detection frame received")

        frame = in_rgb.getCvFrame()
        h, w = frame.shape[:2]
        
        best_rect = None
        best_crop = None
        best_label = None

        if in_det is not None:
            center_x, center_y = w // 2, h // 2
            min_dist = float('inf')

            for d in in_det.detections:
                if d.confidence < 0.5:
                    continue
                if not self.capturing and d.label != self.ref_label:
                    continue

                x1, y1, x2, y2 = int(d.xmin * w), int(d.ymin * h), int(d.xmax * w), int(d.ymax * h)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                dist = ((cx - center_x) ** 2 + (cy - center_y) ** 2) ** 0.5
                
                if dist < min_dist:
                    min_dist = dist
                    best_rect = (max(0, x1), max(0, y1), min(w, x2), min(h, y2))
                    best_label = d.label
                    
                    bx1, by1, bx2, by2 = best_rect
                    if bx2-bx1 > 10 and by2-by1 > 10:
                        best_crop = frame[by1:by2, bx1:bx2]

        if self.capturing:
            if best_rect is not None:
                bx1, by1, bx2, by2 = best_rect
                label_txt = self.LABELS[best_label] if best_label < len(self.LABELS) else str(best_label)
                cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 255, 255), 3)
                cv2.putText(frame, f"TARGET: {label_txt}", (bx1, by1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                if self.trigger_lock:
                    self.get_logger().info(f"LOCKED: {label_txt}")
                    self.ref_emb = self.get_embedding(best_crop)
                    self.ref_img = cv2.resize(best_crop, (100, 100))
                    self.ref_label = best_label
                    self.capturing = False
                    self.trigger_lock = False
        else:
            if best_crop is not None:
                curr_emb = self.get_embedding(best_crop)
                if curr_emb is not None and self.ref_emb is not None:
                    score = self.cosine_similarity(self.ref_emb, curr_emb)
                    color = (0, 255, 0) if score > self.MATCH_THRESHOLD else (0, 0, 255)
                    x1, y1, x2, y2 = best_rect
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                    cv2.putText(frame, f"{score:.2f}", (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            if self.ref_img is not None:
                try:
                    frame[10:110, 10:110] = self.ref_img 
                except Exception as e:
                    self.get_logger().warn(f"Failed to overlay ref_img: {e}")

        # *** CRITICAL FIX: RESIZE FOR FOXGLOVE/WIFI ***
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
