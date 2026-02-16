"""
Execution Node: Simulates robot execution of a toolpath.

Subscribes to /repair/toolpath (geometry_msgs/Polygon)
Publishes:
  - /repair/executed (std_msgs/Bool): True when toolpath execution completes
  - /repair/executed_path (nav_msgs/Path): Executed path for visualization
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Polygon, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import math


class ExecuteNode(Node):
    def __init__(self):
        super().__init__("execute_node")

        self.declare_parameter("toolpath_topic", "/repair/toolpath")
        self.declare_parameter("executed_topic", "/repair/executed")
        self.declare_parameter("executed_path_topic", "/repair/executed_path")
        self.declare_parameter("execute_rate_hz", 10.0)
        self.declare_parameter("workspace_width", 100.0)
        self.declare_parameter("workspace_height", 100.0)
        self.declare_parameter("path_frame", "repairman_map")
        self.declare_parameter("accept_updates_while_executing", False)
        self.declare_parameter("path_publish_stride", 5)
        self.declare_parameter("publish_progress_path", False)

        toolpath_topic = self.get_parameter("toolpath_topic").value
        self.executed_topic = self.get_parameter("executed_topic").value
        self.executed_path_topic = self.get_parameter("executed_path_topic").value
        execute_rate = float(self.get_parameter("execute_rate_hz").value)
        self.workspace_width = float(self.get_parameter("workspace_width").value)
        self.workspace_height = float(self.get_parameter("workspace_height").value)
        self.path_frame = self.get_parameter("path_frame").value
        self.accept_updates_while_executing = bool(
            self.get_parameter("accept_updates_while_executing").value
        )
        self.path_publish_stride = max(1, int(self.get_parameter("path_publish_stride").value))
        self.publish_progress_path = bool(self.get_parameter("publish_progress_path").value)

        self.sub = self.create_subscription(Polygon, toolpath_topic, self.on_toolpath, 10)
        self.pub_executed = self.create_publisher(Bool, self.executed_topic, 10)
        self.pub_executed_path = self.create_publisher(Path, self.executed_path_topic, 10)

        # Execution state
        self.current_toolpath = None
        self.execution_in_progress = False
        self.current_point_index = 0

        # Timer for simulated execution
        self.execution_timer = self.create_timer(1.0 / execute_rate, self.execute_step)

        self.get_logger().info(
            f"ExecuteNode initialized: {toolpath_topic} -> {self.executed_topic}, {self.executed_path_topic} "
            f"(frame={self.path_frame})"
        )

    def on_toolpath(self, msg: Polygon):
        """Receive a new toolpath and start execution."""
        if len(msg.points) == 0:
            self.get_logger().warn("Received empty toolpath, ignoring.")
            return
        if self.execution_in_progress and not self.accept_updates_while_executing:
            self.get_logger().debug("Execution already in progress; ignoring toolpath update.")
            return

        self.current_toolpath = msg
        self.execution_in_progress = True
        self.current_point_index = 0
        self.get_logger().info(f"Received new toolpath with {len(msg.points)} points. Starting execution...")

    def execute_step(self):
        """Simulate executing one step of the toolpath."""
        if not self.execution_in_progress or self.current_toolpath is None:
            return

        # Move to next point
        if self.current_point_index < len(self.current_toolpath.points):
            self.current_point_index += 1
            if self.publish_progress_path:
                should_publish = (
                    self.current_point_index % self.path_publish_stride == 0
                    or self.current_point_index == len(self.current_toolpath.points)
                )
                if should_publish:
                    self.publish_executed_path_progress()
        else:
            if not self.publish_progress_path:
                # Publish final executed path once to keep RViz load low.
                self.current_point_index = len(self.current_toolpath.points)
                self.publish_executed_path_progress()
            # Execution complete
            self.execution_in_progress = False
            self.current_toolpath = None
            self.current_point_index = 0

            # Publish success
            result = Bool(data=True)
            self.pub_executed.publish(result)
            self.get_logger().info("Toolpath execution complete. Published /repair/executed=True")

    def publish_executed_path_progress(self):
        """Convert current execution progress to a Path message."""
        path = Path()
        path.header.frame_id = self.path_frame
        path.header.stamp = Time()

        for i in range(self.current_point_index):
            pt = self.current_toolpath.points[i]
            # Normalize point to workspace
            norm_x, norm_y = self.normalize_point(pt.x, pt.y, self.current_toolpath)

            pose = PoseStamped()
            pose.header.frame_id = self.path_frame
            pose.header.stamp = Time()
            pose.pose.position.x = norm_x
            pose.pose.position.y = norm_y
            pose.pose.position.z = 0.0
            # Simple orientation: direction towards next point
            if i < len(self.current_toolpath.points) - 1:
                next_pt = self.current_toolpath.points[i + 1]
                dx = next_pt.x - pt.x
                dy = next_pt.y - pt.y
                angle = math.atan2(dy, dx)
                # Convert angle to quaternion (yaw only)
                pose.pose.orientation.z = math.sin(angle / 2)
                pose.pose.orientation.w = math.cos(angle / 2)
            else:
                pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        self.pub_executed_path.publish(path)

    def normalize_point(self, x, y, polygon):
        """
        Normalize polygon points to workspace bounds.

        Simple scaling: map bounding box to workspace_width x workspace_height.
        """
        if len(polygon.points) == 0:
            return x, y

        xs = [p.x for p in polygon.points]
        ys = [p.y for p in polygon.points]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        # Avoid division by zero
        width = max(max_x - min_x, 1e-6)
        height = max(max_y - min_y, 1e-6)

        # Scale to workspace
        norm_x = ((x - min_x) / width) * self.workspace_width
        norm_y = ((y - min_y) / height) * self.workspace_height

        return norm_x, norm_y


def main():
    rclpy.init()
    node = ExecuteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
