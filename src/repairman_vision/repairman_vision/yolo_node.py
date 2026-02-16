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
        self.declare_parameter("bbox_refine_crack", True)
        self.declare_parameter("dummy_use_image_crack_extractor", True)
        self.declare_parameter("dummy_blackhat_kernel", 17)
        self.declare_parameter("dummy_adaptive_block_size", 31)
        self.declare_parameter("dummy_adaptive_c", 2.0)
        self.declare_parameter("dummy_open_kernel", 3)
        self.declare_parameter("dummy_close_kernel", 5)
        self.declare_parameter("dummy_min_component_area", 80)
        self.declare_parameter("dummy_keep_top_components", 1)
        self.declare_parameter("dummy_component_score_mode", "perimeter_x_aspect")

        self.weights_path = self.get_parameter("weights_path").value
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.camera_topic = self.get_parameter("camera_topic").value
        self.executed_topic = self.get_parameter("executed_topic").value
        self.mask_after_topic = self.get_parameter("mask_after_topic").value
        self.use_dummy_mode = bool(self.get_parameter("use_dummy_mode").value)
        self.bbox_refine_crack = bool(self.get_parameter("bbox_refine_crack").value)
        self.dummy_use_image_crack_extractor = bool(
            self.get_parameter("dummy_use_image_crack_extractor").value
        )
        self.dummy_blackhat_kernel = self.to_odd_int(
            self.get_parameter("dummy_blackhat_kernel").value, minimum=3
        )
        self.dummy_adaptive_block_size = self.to_odd_int(
            self.get_parameter("dummy_adaptive_block_size").value, minimum=3
        )
        self.dummy_adaptive_c = float(self.get_parameter("dummy_adaptive_c").value)
        self.dummy_open_kernel = self.to_odd_int(
            self.get_parameter("dummy_open_kernel").value, minimum=1
        )
        self.dummy_close_kernel = self.to_odd_int(
            self.get_parameter("dummy_close_kernel").value, minimum=1
        )
        self.dummy_min_component_area = int(
            self.get_parameter("dummy_min_component_area").value
        )
        self.dummy_keep_top_components = max(
            1, int(self.get_parameter("dummy_keep_top_components").value)
        )
        self.dummy_component_score_mode = str(
            self.get_parameter("dummy_component_score_mode").value
        )
        self.capture_after_on_next_frame = False

        self.bridge = CvBridge()
        
        if YOLO_AVAILABLE and not self.use_dummy_mode:
            self.model = YOLO(self.weights_path)
            self.get_logger().info(f"Loaded YOLO weights: {self.weights_path}")
        else:
            self.model = None
            self.get_logger().warn(
                "Using DUMMY crack detection mode (no YOLO). "
                "Fallback chain: image crack extractor -> synthetic mask."
            )

        self.sub = self.create_subscription(Image, self.camera_topic, self.cb, 10)
        self.sub_executed = self.create_subscription(Bool, self.executed_topic, self.on_executed, 10)
        self.pub_mask = self.create_publisher(Image, "/damage/mask", 10)
        self.pub_annot = self.create_publisher(Image, "/damage/annotated", 10)
        self.pub_mask_after = self.create_publisher(Image, self.mask_after_topic, 10)

        self.get_logger().info(
            f"YoloCrackDetector listening {self.camera_topic}, after-trigger from {self.executed_topic}"
        )
        if self.model is not None:
            self.get_logger().info(
                "Mask generation priority: segmentation masks -> refined bbox cracks -> bbox fill."
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
            used_segmentation = self.fill_mask_from_segmentation(r0, mask, h, w)
            if not used_segmentation:
                self.fill_mask_from_boxes(r0, frame, mask, h, w)
        else:
            # Dummy mode: image-driven crack extraction fallback (no YOLO).
            annotated = frame.copy()
            mask = np.zeros((h, w), dtype=np.uint8)

            if self.dummy_use_image_crack_extractor:
                mask = self.extract_crack_from_image(frame)
            if cv2.countNonZero(mask) == 0:
                # Last-resort fallback if extraction fails completely.
                xs = np.linspace(int(0.15 * w), int(0.85 * w), 160)
                ys = (
                    0.50 * h
                    + 0.16 * h * np.sin((xs / max(w, 1)) * 2.2 * np.pi)
                    + 0.05 * h * np.sin((xs / max(w, 1)) * 6.0 * np.pi)
                )
                curve = np.stack([xs, ys], axis=1).astype(np.int32).reshape((-1, 1, 2))
                thickness = max(8, int(0.03 * min(h, w)))
                cv2.polylines(mask, [curve], isClosed=False, color=255, thickness=thickness)
                mask = cv2.morphologyEx(
                    mask,
                    cv2.MORPH_CLOSE,
                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
                )

            # Draw extracted contour(s) on annotated image for visualization.
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(annotated, contours, -1, (0, 255, 0), 2)
            cv2.putText(annotated, "DUMMY MODE (no YOLO)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            self.pub_annot.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_msg.header = msg.header
        self.pub_mask.publish(mask_msg)

        if self.capture_after_on_next_frame:
            self.pub_mask_after.publish(mask_msg)
            self.capture_after_on_next_frame = False
            self.get_logger().info(f"Published post-repair mask to {self.mask_after_topic}.")

    def fill_mask_from_segmentation(self, result, out_mask, h, w):
        """Use segmentation polygons when available for true crack shape."""
        if result.masks is None or not hasattr(result.masks, "xy"):
            return False

        polygons = result.masks.xy
        if polygons is None or len(polygons) == 0:
            return False

        used = False
        for poly in polygons:
            if poly is None or len(poly) < 3:
                continue
            pts = np.round(poly).astype(np.int32)
            pts[:, 0] = np.clip(pts[:, 0], 0, w - 1)
            pts[:, 1] = np.clip(pts[:, 1], 0, h - 1)
            cv2.fillPoly(out_mask, [pts], 255)
            used = True

        if used:
            self.get_logger().debug("Using YOLO segmentation masks for crack toolpath generation.")
        return used

    def fill_mask_from_boxes(self, result, frame, out_mask, h, w):
        """Fallback when segmentation masks are unavailable."""
        if result.boxes is None or len(result.boxes) == 0:
            return

        for b in result.boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = b.astype(int)
            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(0, min(w, x2))
            y2 = max(0, min(h, y2))
            if x2 <= x1 or y2 <= y1:
                continue

            if self.bbox_refine_crack:
                roi = frame[y1:y2, x1:x2]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (5, 5), 0)
                crack = cv2.adaptiveThreshold(
                    blur,
                    255,
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY_INV,
                    31,
                    5,
                )
                crack = cv2.morphologyEx(
                    crack, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                )
                crack = cv2.medianBlur(crack, 3)
                out_mask[y1:y2, x1:x2] = cv2.bitwise_or(out_mask[y1:y2, x1:x2], crack)
            else:
                cv2.rectangle(out_mask, (x1, y1), (x2, y2), 255, thickness=-1)

        if self.bbox_refine_crack:
            self.get_logger().warn(
                "YOLO segmentation unavailable, using refined bbox crack mask. "
                "For best centerline quality use a segmentation model."
            )

    def extract_crack_from_image(self, frame):
        """Classical CV fallback to approximate crack mask from raw image."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_eq = clahe.apply(gray)

        # Dark thin defects are emphasized with blackhat.
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.dummy_blackhat_kernel, self.dummy_blackhat_kernel)
        )
        blackhat = cv2.morphologyEx(gray_eq, cv2.MORPH_BLACKHAT, kernel)
        blackhat = cv2.normalize(blackhat, None, 0, 255, cv2.NORM_MINMAX)

        _, otsu = cv2.threshold(blackhat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        adaptive = cv2.adaptiveThreshold(
            blackhat,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            self.dummy_adaptive_block_size,
            self.dummy_adaptive_c,
        )

        mask = cv2.bitwise_and(adaptive, otsu)
        if self.dummy_open_kernel > 1:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE,
                    (self.dummy_open_kernel, self.dummy_open_kernel),
                ),
            )
        if self.dummy_close_kernel > 1:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE,
                    (self.dummy_close_kernel, self.dummy_close_kernel),
                ),
            )

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if num_labels <= 1:
            return np.zeros_like(mask)

        candidates = []
        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.dummy_min_component_area:
                continue
            component = np.where(labels == label, 255, 0).astype(np.uint8)
            score = self.score_component(component, area)
            candidates.append((score, label, area))

        if not candidates:
            return np.zeros_like(mask)

        candidates.sort(key=lambda it: it[0], reverse=True)
        keep_n = min(self.dummy_keep_top_components, len(candidates))
        selected_labels = {label for _, label, _ in candidates[:keep_n]}
        out = np.where(np.isin(labels, list(selected_labels)), 255, 0).astype(np.uint8)
        return out

    def score_component(self, component, area):
        """Score a connected component for crack-likeness in fallback mode."""
        mode = self.dummy_component_score_mode.lower().strip()
        if mode == "largest_area":
            return float(area)

        contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return float(area)
        contour = max(contours, key=cv2.contourArea)
        perimeter = float(cv2.arcLength(contour, True))
        _, _, w, h = cv2.boundingRect(contour)
        short_side = float(max(1, min(w, h)))
        long_side = float(max(w, h))
        aspect = long_side / short_side
        return perimeter * max(1.0, aspect) * np.log1p(float(area))

    @staticmethod
    def to_odd_int(value, minimum=1):
        out = max(minimum, int(value))
        if out % 2 == 0:
            out += 1
        return out

def main():
    rclpy.init()
    node = YoloCrackDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
