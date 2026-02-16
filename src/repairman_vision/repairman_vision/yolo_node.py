import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import cv2

# Try to import YOLO, fallback to dummy mode if not available
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("WARNING: ultralytics not installed. Using dummy crack detection mode.")
    print("Install with: pip install ultralytics")


class YoloCrackDetector(Node):
    def __init__(self):
        super().__init__("crack_detector_node")

        self.declare_parameter("weights_path", "/home/dogru/repairman_ws/weights/best.pt")
        self.declare_parameter("conf", 0.4)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("executed_topic", "/repair/executed")
        self.declare_parameter("mask_after_topic", "/damage/mask_after")
        self.declare_parameter("use_dummy_mode", not YOLO_AVAILABLE)

        self.weights_path = self.get_parameter("weights_path").value
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.camera_topic = self.get_parameter("camera_topic").value
        self.executed_topic = self.get_parameter("executed_topic").value
        self.mask_after_topic = self.get_parameter("mask_after_topic").value
        self.use_dummy_mode = bool(self.get_parameter("use_dummy_mode").value)
        self.capture_after_on_next_frame = False

        self.bridge = CvBridge()
        
        if YOLO_AVAILABLE and not self.use_dummy_mode:
            self.model = YOLO(self.weights_path)
            self.get_logger().info(f"Loaded YOLO weights: {self.weights_path}")
        else:
            self.model = None
            self.get_logger().warn("Using DUMMY crack detection mode (no YOLO). Generating synthetic mask.")

        self.sub = self.create_subscription(Image, self.camera_topic, self.cb, 10)
        self.sub_executed = self.create_subscription(Bool, self.executed_topic, self.on_executed, 10)
        self.pub_mask = self.create_publisher(Image, "/damage/mask", 10)
        self.pub_annot = self.create_publisher(Image, "/damage/annotated", 10)
        self.pub_mask_after = self.create_publisher(Image, self.mask_after_topic, 10)

        self.get_logger().info(
            f"YoloCrackDetector listening {self.camera_topic}, after-trigger from {self.executed_topic}"
        )

    def on_executed(self, msg: Bool):
        """Schedule a one-shot publish to /damage/mask_after after a repair execution completes."""
        if msg.data:
            self.capture_after_on_next_frame = True
            self.get_logger().info("Received /repair/executed=True. Next mask will be published as /damage/mask_after.")

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]

        if self.model is not None:
            # YOLO mode
            results = self.model.predict(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)
            r0 = results[0]

            annotated = r0.plot()
            self.pub_annot.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))

            mask = np.zeros((h, w), dtype=np.uint8)
            if r0.boxes is not None and len(r0.boxes) > 0:
                for b in r0.boxes.xyxy.cpu().numpy():
                    x1, y1, x2, y2 = b.astype(int)
                    cv2.rectangle(mask, (x1, y1), (x2, y2), 255, thickness=-1)
        else:
            # Dummy mode: synthetic crack detection (for testing without YOLO)
            # Create a synthetic vertical crack in the middle of the image
            annotated = frame.copy()
            mask = np.zeros((h, w), dtype=np.uint8)
            
            # Draw a synthetic vertical crack/crack region
            crack_x_start = w // 3
            crack_x_end = 2 * w // 3
            crack_y_start = h // 4
            crack_y_end = 3 * h // 4
            
            # Create mask (white region = damage)
            cv2.rectangle(mask, (crack_x_start, crack_y_start), (crack_x_end, crack_y_end), 255, thickness=-1)
            
            # Draw on annotated image for visualization
            cv2.rectangle(annotated, (crack_x_start, crack_y_start), (crack_x_end, crack_y_end), (0, 255, 0), thickness=2)
            cv2.putText(annotated, "DUMMY MODE (no YOLO)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            self.pub_annot.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_msg.header = msg.header
        self.pub_mask.publish(mask_msg)

        if self.capture_after_on_next_frame:
            self.pub_mask_after.publish(mask_msg)
            self.capture_after_on_next_frame = False
            self.get_logger().info(f"Published post-repair mask to {self.mask_after_topic}.")

def main():
    rclpy.init()
    node = YoloCrackDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
