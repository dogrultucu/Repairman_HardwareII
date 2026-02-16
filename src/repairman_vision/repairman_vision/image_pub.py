import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("test_image_publisher")

        self.declare_parameter("image_path", "/home/dogru/repairman_ws/data/test.png")
        self.declare_parameter("topic", "/camera/image_raw")
        self.declare_parameter("hz", 5.0)

        image_path = self.get_parameter("image_path").value
        topic = self.get_parameter("topic").value
        hz = float(self.get_parameter("hz").value)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, 10)

        img = cv2.imread(image_path)
        if img is None:
            raise RuntimeError(f"Could not read image: {image_path}")

        self.msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.timer = self.create_timer(1.0 / hz, self.tick)

        self.get_logger().info(f"Publishing {image_path} -> {topic} @ {hz} Hz")

    def tick(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()