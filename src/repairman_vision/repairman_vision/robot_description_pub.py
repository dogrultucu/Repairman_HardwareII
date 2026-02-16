import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__("robot_description_pub")

        self.declare_parameter("topic", "/robot_description")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("publish_rate_hz", 0.5)

        topic = self.get_parameter("topic").value
        self.robot_description = self.get_parameter("robot_description").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        publish_rate_hz = max(0.1, publish_rate_hz)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(String, topic, qos)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_description)

        if not self.robot_description:
            self.get_logger().warn("robot_description is empty; RobotModel may not render.")
        else:
            self.get_logger().info(f"Publishing robot description on {topic} (transient_local)")

        self.publish_description()

    def publish_description(self):
        if not self.robot_description:
            return
        msg = String()
        msg.data = self.robot_description
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RobotDescriptionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
