import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from geometry_msgs.msg import Polygon, Point32


class ToolpathNode(Node):
    def __init__(self):
        super().__init__("toolpath_node")

        self.declare_parameter("mask_topic", "/damage/mask")
        self.declare_parameter("toolpath_topic", "/repair/toolpath")
        self.declare_parameter("min_area", 300)  # px^2 threshold to ignore tiny blobs

        mask_topic = self.get_parameter("mask_topic").value
        toolpath_topic = self.get_parameter("toolpath_topic").value
        self.min_area = int(self.get_parameter("min_area").value)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, mask_topic, self.cb, 10)
        self.pub = self.create_publisher(Polygon, toolpath_topic, 10)

        self.get_logger().info(f"ToolpathNode subscribed: {mask_topic} -> publishes: {toolpath_topic}")

    def cb(self, msg: Image):
        # Expect mono8 mask (0/255)
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        # Clean noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return

        # Pick largest contour
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area < self.min_area:
            return

        # Simple MVP centerline approximation:
        # Use contour points downsampled (every N points) as a "path".
        # (Later: skeletonization -> true centerline)
        step = max(1, len(cnt) // 80)  # keep ~80 points
        poly = Polygon()
        for i in range(0, len(cnt), step):
            x, y = cnt[i][0]
            poly.points.append(Point32(x=float(x), y=float(y), z=0.0))

        self.pub.publish(poly)


def main():
    rclpy.init()
    node = ToolpathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
