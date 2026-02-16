"""
Verification Node: Computes repair quality score.

Subscribes to:
  - /damage/mask (before image)
  - /damage/mask_after (after image)

Publishes:
  - /repair/quality_score (std_msgs/Float32): Repair quality in range [0.0, 1.0]

Quality metric:
  quality = clamp((before_area - after_area) / max(before_area, eps), 0, 1)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque


class VerifyNode(Node):
    def __init__(self):
        super().__init__("verify_node")

        self.declare_parameter("mask_before_topic", "/damage/mask")
        self.declare_parameter("mask_after_topic", "/damage/mask_after")
        self.declare_parameter("quality_topic", "/repair/quality_score")
        self.declare_parameter("executed_topic", "/repair/executed")
        self.declare_parameter("compute_rate_hz", 2.0)
        self.declare_parameter("use_latest_as_after", False)  # If True, use latest /damage/mask as "after"

        self.mask_before_topic = self.get_parameter("mask_before_topic").value
        self.mask_after_topic = self.get_parameter("mask_after_topic").value
        self.quality_topic = self.get_parameter("quality_topic").value
        self.executed_topic = self.get_parameter("executed_topic").value
        compute_rate = float(self.get_parameter("compute_rate_hz").value)
        self.use_latest_as_after = bool(self.get_parameter("use_latest_as_after").value)

        self.bridge = CvBridge()

        # For MVP, we can subscribe to both topics, or if use_latest_as_after=True,
        # we just track the latest /damage/mask as "after"
        if self.use_latest_as_after:
            self.sub_before = self.create_subscription(Image, self.mask_before_topic, self.on_mask_before, 10)
            self.mask_after = None
            self.mode = "latest"
            self.get_logger().info("Using latest /damage/mask as 'after' image (use_latest_as_after=True)")
        else:
            self.sub_before = self.create_subscription(Image, self.mask_before_topic, self.on_mask_before, 10)
            self.sub_after = self.create_subscription(Image, self.mask_after_topic, self.on_mask_after, 10)
            self.sub_executed = self.create_subscription(Bool, self.executed_topic, self.on_executed, 10)
            self.mask_after = None
            self.mode = "paired"
            self.get_logger().info(
                f"Waiting for paired images: {self.mask_before_topic} + {self.mask_after_topic} "
                f"(triggered by {self.executed_topic})"
            )

        self.pub_quality = self.create_publisher(Float32, self.quality_topic, 10)

        self.mask_before = None  # frozen "before" snapshot for current cycle
        self.mask_after_ready = False
        self.awaiting_after = False
        self.quality_score_cache = deque(maxlen=5)

        # Timer to periodically compute quality (for paired mode)
        self.compute_timer = self.create_timer(1.0 / compute_rate, self.compute_quality)

        self.get_logger().info(f"VerifyNode initialized: publishes {self.quality_topic}")

    def on_mask_before(self, msg: Image):
        """Receive 'before' mask (or latest mask in latest mode)."""
        try:
            if self.use_latest_as_after:
                self.mask_before = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                # In "latest" mode, also use this as the "after" for quality computation
                self.mask_after = self.mask_before
                return

            # Paired mode:
            # Keep the first snapshot as baseline until execution completes and after-mask is consumed.
            if self.mask_before is None and not self.awaiting_after:
                self.mask_before = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                self.get_logger().info("Captured before-mask snapshot for current repair cycle.")
        except Exception as e:
            self.get_logger().error(f"Error converting mask_before: {e}")

    def on_mask_after(self, msg: Image):
        """Receive 'after' mask (paired mode only)."""
        if not self.use_latest_as_after:
            try:
                self.mask_after = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                self.mask_after_ready = True
            except Exception as e:
                self.get_logger().error(f"Error converting mask_after: {e}")

    def on_executed(self, msg: Bool):
        """Mark that execution completed and the next after-mask should be evaluated."""
        if self.use_latest_as_after:
            return
        if msg.data:
            self.awaiting_after = True

    def compute_quality(self):
        """Compute repair quality score."""
        if self.mask_before is None:
            return

        if self.mask_after is None:
            # Not ready yet
            return
        if not self.use_latest_as_after and not self.mask_after_ready:
            return
        if not self.use_latest_as_after and not self.awaiting_after:
            return

        # Count non-zero pixels (damage area)
        before_area = cv2.countNonZero(self.mask_before)
        after_area = cv2.countNonZero(self.mask_after)

        # Quality metric: how much damage was removed
        eps = 1e-5
        if before_area < eps:
            # No damage initially
            quality = 0.0 if after_area > before_area else 1.0
        else:
            quality = (before_area - after_area) / float(before_area)

        # Clamp to [0, 1]
        quality = max(0.0, min(1.0, quality))

        self.quality_score_cache.append(quality)

        # Publish (optionally smooth with average)
        smoothed_quality = np.mean(list(self.quality_score_cache))
        score_msg = Float32(data=float(smoothed_quality))
        self.pub_quality.publish(score_msg)
        self.get_logger().info(f"Published quality score: {smoothed_quality:.3f}")

        self.get_logger().debug(
            f"Quality: before_area={before_area}, after_area={after_area}, score={smoothed_quality:.3f}"
        )

        if not self.use_latest_as_after:
            # Consume paired sample once to avoid reusing stale /damage/mask_after.
            self.mask_before = None
            self.mask_after_ready = False
            self.mask_after = None
            self.awaiting_after = False


def main():
    rclpy.init()
    node = VerifyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
