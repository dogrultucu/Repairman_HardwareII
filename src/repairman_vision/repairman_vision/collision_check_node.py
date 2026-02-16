import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

try:
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity

    MOVEIT_MSGS_AVAILABLE = True
except ImportError:
    RobotState = None
    GetStateValidity = None
    MOVEIT_MSGS_AVAILABLE = False


class CollisionCheckNode(Node):
    """Query MoveIt /check_state_validity and publish collision status."""

    def __init__(self):
        super().__init__("collision_check_node")

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("collision_topic", "/repair/in_collision")
        self.declare_parameter("status_topic", "/repair/collision_status")
        self.declare_parameter("marker_topic", "/repair/collision_marker")
        self.declare_parameter("state_validity_service", "/check_state_validity")
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("check_rate_hz", 5.0)
        self.declare_parameter("marker_frame", "repairman_map")
        self.declare_parameter("marker_pose", [0.15, 0.15, 0.35])
        self.declare_parameter("marker_scale", 0.12)

        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.collision_topic = str(self.get_parameter("collision_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.service_name = str(self.get_parameter("state_validity_service").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.check_rate_hz = max(0.2, float(self.get_parameter("check_rate_hz").value))
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.marker_pose = [float(v) for v in self.get_parameter("marker_pose").value]
        self.marker_scale = float(self.get_parameter("marker_scale").value)

        self.latest_joint_state = None
        self.request_pending = False
        self.last_in_collision = None
        self.last_status_log_ns = 0

        self.sub_joint = self.create_subscription(
            JointState, self.joint_state_topic, self.on_joint_state, 20
        )
        self.pub_collision = self.create_publisher(Bool, self.collision_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.pub_marker = self.create_publisher(Marker, self.marker_topic, 10)

        self.enabled = MOVEIT_MSGS_AVAILABLE
        if not self.enabled:
            self.get_logger().error(
                "moveit_msgs not found. Install MoveIt packages and relaunch collision_check_node."
            )
            self.publish_status(False, "MoveIt not available")
            return

        self.client = self.create_client(GetStateValidity, self.service_name)
        self.timer = self.create_timer(1.0 / self.check_rate_hz, self.tick)

        self.get_logger().info(
            f"CollisionCheckNode monitoring {self.joint_state_topic} via {self.service_name} "
            f"for group '{self.group_name}'"
        )
        self.publish_status(False, "Waiting for MoveIt state validity service")

    def on_joint_state(self, msg: JointState):
        if not msg.name:
            return
        self.latest_joint_state = copy.deepcopy(msg)

    def tick(self):
        if not self.enabled:
            return

        if self.latest_joint_state is None:
            self.publish_status(False, "Waiting for /joint_states")
            return

        if not self.client.service_is_ready():
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_status_log_ns > int(5e9):
                self.get_logger().warn(
                    f"Service {self.service_name} not available yet. "
                    "Run move_group with the robot MoveIt config."
                )
                self.last_status_log_ns = now_ns
            self.publish_status(False, "Waiting for /check_state_validity")
            return

        if self.request_pending:
            return

        req = GetStateValidity.Request()
        req.group_name = self.group_name

        robot_state = RobotState()
        robot_state.joint_state = self.latest_joint_state
        req.robot_state = robot_state

        future = self.client.call_async(req)
        self.request_pending = True
        future.add_done_callback(self.on_response)

    def on_response(self, future):
        self.request_pending = False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"State validity service call failed: {exc}")
            self.publish_status(False, "State validity call failed")
            return

        in_collision = not bool(response.valid)
        contacts = len(response.contacts)

        status = "IN COLLISION" if in_collision else "Collision-free"
        if contacts > 0:
            status += f" ({contacts} contacts)"

        self.publish_status(in_collision, status)

        if self.last_in_collision is None or self.last_in_collision != in_collision:
            self.get_logger().info(status)
            self.last_in_collision = in_collision

    def publish_status(self, in_collision: bool, text: str):
        bool_msg = Bool()
        bool_msg.data = bool(in_collision)
        self.pub_collision.publish(bool_msg)

        status_msg = String()
        status_msg.data = text
        self.pub_status.publish(status_msg)

        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "repair_collision_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.marker_pose[0]
        marker.pose.position.y = self.marker_pose[1]
        marker.pose.position.z = self.marker_pose[2]
        marker.pose.orientation.w = 1.0
        marker.scale.z = self.marker_scale

        if in_collision:
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
        else:
            marker.color.r = 0.1
            marker.color.g = 0.95
            marker.color.b = 0.2
        marker.color.a = 1.0
        marker.text = text

        self.pub_marker.publish(marker)


def main():
    rclpy.init()
    node = CollisionCheckNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
