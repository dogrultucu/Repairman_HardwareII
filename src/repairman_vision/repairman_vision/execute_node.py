"""
Execution Node: Simulates robot execution of a toolpath.

Subscribes to /repair/toolpath (geometry_msgs/Polygon)
Publishes:
  - /repair/executed (std_msgs/Bool): True when toolpath execution completes
  - /repair/executed_path (nav_msgs/Path): Executed path for visualization
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Polygon, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
import math


class ExecuteNode(Node):
    def __init__(self):
        super().__init__("execute_node")

        self.declare_parameter("toolpath_topic", "/repair/toolpath")
        self.declare_parameter("state_topic", "/repair/state")
        self.declare_parameter("executed_topic", "/repair/executed")
        self.declare_parameter("executed_path_topic", "/repair/executed_path")
        self.declare_parameter("require_repair_state", True)
        self.declare_parameter("execute_rate_hz", 10.0)
        self.declare_parameter("workspace_width", 100.0)
        self.declare_parameter("workspace_height", 100.0)
        self.declare_parameter("path_frame", "repairman_map")
        self.declare_parameter("accept_updates_while_executing", False)
        self.declare_parameter("path_publish_stride", 5)
        self.declare_parameter("publish_progress_path", False)
        self.declare_parameter("simulate_extrusion", True)
        self.declare_parameter("extrusion_markers_topic", "/repair/extrusion_markers")
        self.declare_parameter("extrusion_line_width", 0.02)
        self.declare_parameter("extrusion_height", 0.01)
        self.declare_parameter("toolhead_size", 0.04)
        self.declare_parameter("extrusion_color_r", 1.0)
        self.declare_parameter("extrusion_color_g", 0.45)
        self.declare_parameter("extrusion_color_b", 0.0)
        self.declare_parameter("extrusion_color_a", 1.0)

        toolpath_topic = self.get_parameter("toolpath_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.executed_topic = self.get_parameter("executed_topic").value
        self.executed_path_topic = self.get_parameter("executed_path_topic").value
        self.require_repair_state = bool(self.get_parameter("require_repair_state").value)
        self.extrusion_markers_topic = self.get_parameter("extrusion_markers_topic").value
        execute_rate = float(self.get_parameter("execute_rate_hz").value)
        self.workspace_width = float(self.get_parameter("workspace_width").value)
        self.workspace_height = float(self.get_parameter("workspace_height").value)
        self.path_frame = self.get_parameter("path_frame").value
        self.accept_updates_while_executing = bool(
            self.get_parameter("accept_updates_while_executing").value
        )
        self.path_publish_stride = max(1, int(self.get_parameter("path_publish_stride").value))
        self.publish_progress_path = bool(self.get_parameter("publish_progress_path").value)
        self.simulate_extrusion = bool(self.get_parameter("simulate_extrusion").value)
        self.extrusion_line_width = float(self.get_parameter("extrusion_line_width").value)
        self.extrusion_height = float(self.get_parameter("extrusion_height").value)
        self.toolhead_size = float(self.get_parameter("toolhead_size").value)
        self.extrusion_color = (
            float(self.get_parameter("extrusion_color_r").value),
            float(self.get_parameter("extrusion_color_g").value),
            float(self.get_parameter("extrusion_color_b").value),
            float(self.get_parameter("extrusion_color_a").value),
        )

        self.sub = self.create_subscription(Polygon, toolpath_topic, self.on_toolpath, 10)
        self.sub_state = self.create_subscription(String, self.state_topic, self.on_state, 10)
        self.pub_executed = self.create_publisher(Bool, self.executed_topic, 10)
        self.pub_executed_path = self.create_publisher(Path, self.executed_path_topic, 10)
        self.pub_extrusion_markers = self.create_publisher(
            MarkerArray, self.extrusion_markers_topic, 10
        )

        # Execution state
        self.current_toolpath = None
        self.execution_in_progress = False
        self.current_point_index = 0
        self.executed_world_points = []
        self.input_already_normalized = False
        self.pending_toolpath = None
        self.pipeline_state = "IDLE"

        # Timer for simulated execution
        self.execution_timer = self.create_timer(1.0 / execute_rate, self.execute_step)

        self.get_logger().info(
            f"ExecuteNode initialized: {toolpath_topic} -> {self.executed_topic}, {self.executed_path_topic} "
            f"(frame={self.path_frame})"
        )
        if self.simulate_extrusion:
            self.get_logger().info(
                f"Extrusion simulation enabled -> {self.extrusion_markers_topic}"
            )

    def on_toolpath(self, msg: Polygon):
        """Receive a new toolpath and start execution."""
        if len(msg.points) == 0:
            self.get_logger().warn("Received empty toolpath, ignoring.")
            return
        if self.execution_in_progress and not self.accept_updates_while_executing:
            self.get_logger().debug("Execution already in progress; ignoring toolpath update.")
            return

        if self.require_repair_state and self.pipeline_state != "REPAIR":
            self.pending_toolpath = msg
            self.get_logger().info(
                f"Toolpath received ({len(msg.points)} points), waiting for REPAIR state "
                f"(current={self.pipeline_state})."
            )
            return

        self.start_execution(msg)

    def on_state(self, msg: String):
        self.pipeline_state = str(msg.data).strip().upper() if msg.data else "IDLE"
        if (
            self.require_repair_state
            and not self.execution_in_progress
            and self.pending_toolpath is not None
            and self.pipeline_state == "REPAIR"
        ):
            pending = self.pending_toolpath
            self.pending_toolpath = None
            self.get_logger().info("REPAIR state active, starting pending toolpath execution.")
            self.start_execution(pending)

    def start_execution(self, msg: Polygon):
        self.current_toolpath = msg
        self.input_already_normalized = self.is_workspace_polygon(msg)
        self.execution_in_progress = True
        self.current_point_index = 0
        self.executed_world_points = []
        self.get_logger().info(f"Received new toolpath with {len(msg.points)} points. Starting execution...")
        if self.input_already_normalized:
            self.get_logger().info("Toolpath interpreted as workspace/world coordinates.")
        else:
            self.get_logger().warn(
                "Toolpath appears to be image/local coordinates; applying bbox normalization."
            )
        if self.simulate_extrusion:
            self.publish_clear_extrusion_markers()

    def execute_step(self):
        """Simulate executing one step of the toolpath."""
        if not self.execution_in_progress or self.current_toolpath is None:
            return
        if self.require_repair_state and self.pipeline_state != "REPAIR":
            return

        # Move to next point
        if self.current_point_index < len(self.current_toolpath.points):
            self.current_point_index += 1
            self.update_extrusion_state()
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
            if self.simulate_extrusion:
                self.publish_extrusion_markers(final=True)

    def publish_executed_path_progress(self):
        """Convert current execution progress to a Path message."""
        path = Path()
        path.header.frame_id = self.path_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(self.current_point_index):
            pt = self.current_toolpath.points[i]
            # Normalize point to workspace
            norm_x, norm_y = self.normalize_point(pt.x, pt.y, self.current_toolpath)

            pose = PoseStamped()
            pose.header.frame_id = self.path_frame
            pose.header.stamp = path.header.stamp
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

    def update_extrusion_state(self):
        """Append current tool tip position and publish extrusion markers."""
        if not self.simulate_extrusion or self.current_point_index <= 0:
            return

        pt = self.current_toolpath.points[self.current_point_index - 1]
        norm_x, norm_y = self.normalize_point(pt.x, pt.y, self.current_toolpath)
        current = (norm_x, norm_y, self.extrusion_height)

        if not self.executed_world_points or self.executed_world_points[-1] != current:
            self.executed_world_points.append(current)
        self.publish_extrusion_markers(final=False)

    def publish_clear_extrusion_markers(self):
        """Clear previous extrusion markers in RViz."""
        clear = Marker()
        clear.header.frame_id = self.path_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        clear.ns = "repair_extrusion"
        clear.id = 0
        arr = MarkerArray()
        arr.markers.append(clear)
        self.pub_extrusion_markers.publish(arr)

    def publish_extrusion_markers(self, final=False):
        """Publish line-strip extrusion trace and toolhead marker."""
        if not self.simulate_extrusion:
            return

        now = self.get_clock().now().to_msg()
        color_r, color_g, color_b, color_a = self.extrusion_color

        line = Marker()
        line.header.frame_id = self.path_frame
        line.header.stamp = now
        line.ns = "repair_extrusion"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = self.extrusion_line_width
        line.color.r = color_r
        line.color.g = color_g
        line.color.b = color_b
        line.color.a = color_a
        line.pose.orientation.w = 1.0
        line.lifetime.sec = 0

        for x, y, z in self.executed_world_points:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = float(z)
            line.points.append(p)

        tool = Marker()
        tool.header.frame_id = self.path_frame
        tool.header.stamp = now
        tool.ns = "repair_extrusion"
        tool.id = 1
        tool.type = Marker.SPHERE
        tool.action = Marker.ADD
        tool.scale.x = self.toolhead_size
        tool.scale.y = self.toolhead_size
        tool.scale.z = self.toolhead_size
        tool.color.r = 0.0
        tool.color.g = 1.0
        tool.color.b = 1.0
        tool.color.a = 0.9
        tool.pose.orientation.w = 1.0
        tool.lifetime.sec = 0

        if self.executed_world_points:
            x, y, z = self.executed_world_points[-1]
            tool.pose.position.x = float(x)
            tool.pose.position.y = float(y)
            tool.pose.position.z = float(z) + (0.0 if final else 0.01)
        else:
            tool.pose.position.x = 0.0
            tool.pose.position.y = 0.0
            tool.pose.position.z = 0.0

        arr = MarkerArray()
        arr.markers.append(line)
        arr.markers.append(tool)
        self.pub_extrusion_markers.publish(arr)

    def normalize_point(self, x, y, polygon):
        """
        Normalize polygon points to workspace bounds.

        Simple scaling: map bounding box to workspace_width x workspace_height.
        """
        if len(polygon.points) == 0:
            return x, y

        if self.input_already_normalized:
            norm_x = self.clamp(float(x), 0.0, self.workspace_width)
            norm_y = self.clamp(float(y), 0.0, self.workspace_height)
            return norm_x, norm_y

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

    def is_workspace_polygon(self, polygon):
        if len(polygon.points) < 2:
            return False
        xs = [p.x for p in polygon.points]
        ys = [p.y for p in polygon.points]
        tol = 0.05 * max(self.workspace_width, self.workspace_height)
        return (
            min(xs) >= -tol
            and min(ys) >= -tol
            and max(xs) <= self.workspace_width + tol
            and max(ys) <= self.workspace_height + tol
        )

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))


def main():
    rclpy.init()
    node = ExecuteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
