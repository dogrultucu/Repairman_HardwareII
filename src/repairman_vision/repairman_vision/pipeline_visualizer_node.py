import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray


class PipelineVisualizerNode(Node):
    """Realtime visualization for pipeline phase and progress."""

    ORDERED_STAGES = ["SCAN", "DETECT", "PLAN", "REPAIR", "RESCAN", "EVALUATE", "PASS"]
    EXPECTED_DURATIONS = {
        "SCAN": 2.0,
        "DETECT": 3.0,
        "PLAN": 3.0,
        "REPAIR": 10.0,
        "RESCAN": 5.0,
        "EVALUATE": 5.0,
        "PASS": 1.0,
        "REPAIR_AGAIN": 1.0,
        "IDLE": 1.0,
    }

    def __init__(self):
        super().__init__("pipeline_visualizer_node")

        self.declare_parameter("state_topic", "/repair/state")
        self.declare_parameter("marker_topic", "/repair/pipeline_markers")
        self.declare_parameter("status_topic", "/repair/pipeline_status")
        self.declare_parameter("stage_progress_topic", "/repair/stage_progress")
        self.declare_parameter("cycle_progress_topic", "/repair/cycle_progress")
        self.declare_parameter("publish_rate_hz", 5.0)

        self.declare_parameter("frame_id", "repairman_map")
        self.declare_parameter("anchor_x", 0.15)
        self.declare_parameter("anchor_y", 0.88)
        self.declare_parameter("anchor_z", 0.38)
        self.declare_parameter("bar_width", 0.70)
        self.declare_parameter("bar_height", 0.02)
        self.declare_parameter("stage_dot_size", 0.03)
        self.declare_parameter("text_size", 0.06)

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.stage_progress_topic = str(self.get_parameter("stage_progress_topic").value)
        self.cycle_progress_topic = str(self.get_parameter("cycle_progress_topic").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.anchor_x = float(self.get_parameter("anchor_x").value)
        self.anchor_y = float(self.get_parameter("anchor_y").value)
        self.anchor_z = float(self.get_parameter("anchor_z").value)
        self.bar_width = float(self.get_parameter("bar_width").value)
        self.bar_height = float(self.get_parameter("bar_height").value)
        self.stage_dot_size = float(self.get_parameter("stage_dot_size").value)
        self.text_size = float(self.get_parameter("text_size").value)

        self.current_state = "IDLE"
        self.last_state = "IDLE"
        self.state_start_time = time.time()
        self.cycle_count = 0

        self.sub_state = self.create_subscription(String, self.state_topic, self.on_state, 10)
        self.pub_markers = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.pub_stage_progress = self.create_publisher(Float32, self.stage_progress_topic, 10)
        self.pub_cycle_progress = self.create_publisher(Float32, self.cycle_progress_topic, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        self.get_logger().info(
            f"Pipeline visualizer started: state={self.state_topic} markers={self.marker_topic}"
        )

    def on_state(self, msg: String):
        new_state = str(msg.data).strip().upper() if msg.data else "IDLE"
        if new_state == self.current_state:
            return

        self.last_state = self.current_state
        self.current_state = new_state
        self.state_start_time = time.time()

        if self.last_state == "IDLE" and self.current_state == "SCAN":
            self.cycle_count += 1

    def on_timer(self):
        stage_progress = self.compute_stage_progress()
        cycle_progress = self.compute_cycle_progress(stage_progress)

        status_text = self.format_status_text(stage_progress, cycle_progress)
        self.publish_status_topics(stage_progress, cycle_progress, status_text)
        self.publish_markers(stage_progress, cycle_progress, status_text)

    def compute_stage_progress(self):
        elapsed = max(0.0, time.time() - self.state_start_time)
        expected = self.EXPECTED_DURATIONS.get(self.current_state, 3.0)
        if expected <= 1e-6:
            return 1.0
        return max(0.0, min(1.0, elapsed / expected))

    def compute_cycle_progress(self, stage_progress):
        if self.current_state in self.ORDERED_STAGES:
            idx = self.ORDERED_STAGES.index(self.current_state)
            return max(0.0, min(1.0, (idx + stage_progress) / float(len(self.ORDERED_STAGES))))

        if self.current_state == "REPAIR_AGAIN":
            return 0.92

        if self.current_state == "IDLE":
            return 0.0

        return 0.0

    def format_status_text(self, stage_progress, cycle_progress):
        stage_label = self.state_label(self.current_state)
        return (
            f"Cycle #{self.cycle_count} | Stage: {stage_label} "
            f"{stage_progress * 100.0:5.1f}% | Overall: {cycle_progress * 100.0:5.1f}%"
        )

    def publish_status_topics(self, stage_progress, cycle_progress, status_text):
        status_msg = String()
        status_msg.data = status_text
        self.pub_status.publish(status_msg)

        stage_msg = Float32()
        stage_msg.data = float(stage_progress * 100.0)
        self.pub_stage_progress.publish(stage_msg)

        cycle_msg = Float32()
        cycle_msg.data = float(cycle_progress * 100.0)
        self.pub_cycle_progress.publish(cycle_msg)

    def publish_markers(self, stage_progress, cycle_progress, status_text):
        now = self.get_clock().now().to_msg()
        markers = []

        # Text header
        text = Marker()
        text.header.frame_id = self.frame_id
        text.header.stamp = now
        text.ns = "pipeline"
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.anchor_x
        text.pose.position.y = self.anchor_y
        text.pose.position.z = self.anchor_z
        text.pose.orientation.w = 1.0
        text.scale.z = self.text_size
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = status_text
        markers.append(text)

        # Bar background
        bg = Marker()
        bg.header.frame_id = self.frame_id
        bg.header.stamp = now
        bg.ns = "pipeline"
        bg.id = 1
        bg.type = Marker.CUBE
        bg.action = Marker.ADD
        bg.pose.position.x = self.anchor_x
        bg.pose.position.y = self.anchor_y
        bg.pose.position.z = self.anchor_z - 0.05
        bg.pose.orientation.w = 1.0
        bg.scale.x = self.bar_width
        bg.scale.y = self.bar_height
        bg.scale.z = self.bar_height
        bg.color.r = 0.2
        bg.color.g = 0.2
        bg.color.b = 0.2
        bg.color.a = 0.8
        markers.append(bg)

        # Bar fill
        fill_width = max(0.001, self.bar_width * cycle_progress)
        fill = Marker()
        fill.header.frame_id = self.frame_id
        fill.header.stamp = now
        fill.ns = "pipeline"
        fill.id = 2
        fill.type = Marker.CUBE
        fill.action = Marker.ADD
        fill.pose.position.x = self.anchor_x - self.bar_width / 2.0 + fill_width / 2.0
        fill.pose.position.y = self.anchor_y
        fill.pose.position.z = self.anchor_z - 0.05
        fill.pose.orientation.w = 1.0
        fill.scale.x = fill_width
        fill.scale.y = self.bar_height
        fill.scale.z = self.bar_height
        fill.color.r = 0.1
        fill.color.g = 0.85
        fill.color.b = 0.25
        fill.color.a = 0.95
        markers.append(fill)

        active_index = self.active_stage_index()
        for idx, stage in enumerate(self.ORDERED_STAGES):
            dot = Marker()
            dot.header.frame_id = self.frame_id
            dot.header.stamp = now
            dot.ns = "pipeline_stages"
            dot.id = 100 + idx
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x = self.stage_x(idx)
            dot.pose.position.y = self.anchor_y
            dot.pose.position.z = self.anchor_z - 0.095
            dot.pose.orientation.w = 1.0
            dot.scale.x = self.stage_dot_size
            dot.scale.y = self.stage_dot_size
            dot.scale.z = self.stage_dot_size

            if idx < active_index:
                dot.color.r = 0.1
                dot.color.g = 0.85
                dot.color.b = 0.25
                dot.color.a = 1.0
            elif idx == active_index:
                pulse = 0.5 + 0.5 * math.sin(time.time() * 4.0)
                dot.color.r = 1.0
                dot.color.g = 0.75 + 0.25 * pulse
                dot.color.b = 0.1
                dot.color.a = 1.0
            else:
                dot.color.r = 0.45
                dot.color.g = 0.45
                dot.color.b = 0.45
                dot.color.a = 0.95

            markers.append(dot)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = now
            label.ns = "pipeline_stage_labels"
            label.id = 200 + idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = self.stage_x(idx)
            label.pose.position.y = self.anchor_y
            label.pose.position.z = self.anchor_z - 0.13
            label.pose.orientation.w = 1.0
            label.scale.z = self.text_size * 0.50
            label.color.r = 0.9
            label.color.g = 0.9
            label.color.b = 0.9
            label.color.a = 0.95
            label.text = stage
            markers.append(label)

        arr = MarkerArray()
        arr.markers = markers
        self.pub_markers.publish(arr)

    def active_stage_index(self):
        if self.current_state in self.ORDERED_STAGES:
            return self.ORDERED_STAGES.index(self.current_state)
        if self.current_state == "REPAIR_AGAIN":
            return self.ORDERED_STAGES.index("EVALUATE")
        return 0

    def stage_x(self, idx):
        if len(self.ORDERED_STAGES) <= 1:
            return self.anchor_x
        left = self.anchor_x - self.bar_width / 2.0
        step = self.bar_width / float(len(self.ORDERED_STAGES) - 1)
        return left + idx * step

    @staticmethod
    def state_label(state):
        if state == "REPAIR_AGAIN":
            return "RETRY"
        return state


def main():
    rclpy.init()
    node = PipelineVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
