import math
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


class ArmSimNode(Node):
    """Simple 6-axis arm controller for RViz simulation.

    The node tracks the latest point from /repair/executed_path and publishes
    /joint_states using a lightweight IK solver (3 positional joints + 3 wrist
    joints for orientation shaping).
    """

    def __init__(self):
        super().__init__("arm_sim_node")

        self.declare_parameter("path_topic", "/repair/executed_path")
        self.declare_parameter("executed_topic", "/repair/executed")
        self.declare_parameter("state_topic", "/repair/state")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("use_state_driven_motion", True)
        self.declare_parameter("path_stale_timeout_sec", 1.0)
        self.declare_parameter("scan_cycle_sec", 4.0)
        self.declare_parameter("scan_sweep_joint1_deg", 22.0)
        self.declare_parameter("scan_sweep_joint4_deg", 16.0)
        self.declare_parameter("wait_pose", [0.35, -1.20, 1.85, 0.0, -0.65, 0.0])
        self.declare_parameter("scan_pose", [0.20, -1.05, 1.70, 0.0, -0.80, 0.0])
        self.declare_parameter("tracking_alpha", 0.2)
        self.declare_parameter("target_smoothing_alpha", 0.35)
        self.declare_parameter("yaw_smoothing_alpha", 0.15)
        self.declare_parameter("max_joint_speed_rad_s", 1.0)
        self.declare_parameter("singularity_margin", 0.08)
        self.declare_parameter("safe_min_radius", 0.26)
        self.declare_parameter("safe_max_radius", 0.44)
        self.declare_parameter("safe_work_z", 0.11)
        self.declare_parameter("tool_tcp_offset", 0.23)
        self.declare_parameter("collision_topic", "/repair/in_collision")
        self.declare_parameter("use_collision_guard", True)
        self.declare_parameter("retreat_on_collision", False)
        self.declare_parameter("elbow_branch_switch_penalty", 0.25)

        self.declare_parameter("base_x", 0.5)
        self.declare_parameter("base_y", 0.5)
        self.declare_parameter("shoulder_z", 0.17)
        self.declare_parameter("work_z", 0.01)

        # Match URDF arm geometry.
        self.declare_parameter("link_2", 0.28)  # shoulder -> elbow
        self.declare_parameter("link_3", 0.24)  # elbow -> wrist

        self.path_topic = self.get_parameter("path_topic").value
        self.executed_topic = self.get_parameter("executed_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.use_state_driven_motion = bool(self.get_parameter("use_state_driven_motion").value)
        self.path_stale_timeout_sec = float(self.get_parameter("path_stale_timeout_sec").value)
        self.scan_cycle_sec = max(1.0, float(self.get_parameter("scan_cycle_sec").value))
        self.scan_sweep_joint1_rad = math.radians(
            float(self.get_parameter("scan_sweep_joint1_deg").value)
        )
        self.scan_sweep_joint4_rad = math.radians(
            float(self.get_parameter("scan_sweep_joint4_deg").value)
        )
        self.wait_pose = self.read_pose_param("wait_pose", [0.35, -1.20, 1.85, 0.0, -0.65, 0.0])
        self.scan_pose = self.read_pose_param("scan_pose", [0.20, -1.05, 1.70, 0.0, -0.80, 0.0])
        self.alpha = float(self.get_parameter("tracking_alpha").value)
        self.target_alpha = float(self.get_parameter("target_smoothing_alpha").value)
        self.yaw_alpha = float(self.get_parameter("yaw_smoothing_alpha").value)
        self.max_joint_speed = float(self.get_parameter("max_joint_speed_rad_s").value)
        self.singularity_margin = float(self.get_parameter("singularity_margin").value)
        self.safe_min_radius = float(self.get_parameter("safe_min_radius").value)
        self.safe_max_radius = float(self.get_parameter("safe_max_radius").value)
        self.safe_work_z = float(self.get_parameter("safe_work_z").value)
        self.tool_tcp_offset = float(self.get_parameter("tool_tcp_offset").value)
        self.collision_topic = str(self.get_parameter("collision_topic").value)
        self.use_collision_guard = bool(self.get_parameter("use_collision_guard").value)
        self.retreat_on_collision = bool(self.get_parameter("retreat_on_collision").value)
        self.elbow_branch_switch_penalty = float(
            self.get_parameter("elbow_branch_switch_penalty").value
        )

        self.base_x = float(self.get_parameter("base_x").value)
        self.base_y = float(self.get_parameter("base_y").value)
        self.shoulder_z = float(self.get_parameter("shoulder_z").value)
        self.work_z = float(self.get_parameter("work_z").value)
        self.l2 = float(self.get_parameter("link_2").value)
        self.l3 = float(self.get_parameter("link_3").value)

        self.target_xy: Optional[Tuple[float, float]] = None
        self.target_yaw = 0.0
        self.pipeline_state = "IDLE"
        self.state_entry_time = time.time()
        self.last_path_update_time = 0.0
        self.home_mode = True
        self.in_collision = False

        self.joint_names = [f"joint_{i}" for i in range(1, 7)]
        self.current = self.wait_pose.copy()
        self.home = self.wait_pose.copy()
        self.last_singularity_scale = 1.0
        self.last_elbow_sign = 1.0

        self.joint_lower = np.array(
            [-math.pi, -2.6, -2.8, -math.pi, -2.6, -2.0 * math.pi], dtype=float
        )
        self.joint_upper = np.array(
            [math.pi, 2.6, 2.8, math.pi, 2.6, 2.0 * math.pi], dtype=float
        )

        self.sub_path = self.create_subscription(Path, self.path_topic, self.on_path, 10)
        self.sub_executed = self.create_subscription(Bool, self.executed_topic, self.on_executed, 10)
        self.sub_state = self.create_subscription(String, self.state_topic, self.on_state, 10)
        if self.use_collision_guard:
            self.sub_collision = self.create_subscription(
                Bool, self.collision_topic, self.on_collision, 10
            )
        self.pub_joint_states = self.create_publisher(JointState, self.joint_state_topic, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.tick)

        self.get_logger().info(
            f"ArmSimNode tracking {self.path_topic} -> publishing {self.joint_state_topic} "
            f"(state_driven={self.use_state_driven_motion})"
        )

    def on_path(self, msg: Path):
        if not msg.poses:
            return
        self.last_path_update_time = time.time()
        if self.use_state_driven_motion and self.pipeline_state != "REPAIR":
            return

        tip = msg.poses[-1].pose.position
        new_target = (float(tip.x), float(tip.y))
        if self.target_xy is None:
            self.target_xy = new_target
        else:
            self.target_xy = (
                (1.0 - self.target_alpha) * self.target_xy[0] + self.target_alpha * new_target[0],
                (1.0 - self.target_alpha) * self.target_xy[1] + self.target_alpha * new_target[1],
            )
        self.home_mode = False

        if len(msg.poses) >= 2:
            p0 = msg.poses[-2].pose.position
            p1 = msg.poses[-1].pose.position
            measured_yaw = math.atan2(p1.y - p0.y, p1.x - p0.x)
            yaw_delta = self.wrap_pi(measured_yaw - self.target_yaw)
            yaw_alpha = self.clamp(self.yaw_alpha, 0.01, 1.0)
            self.target_yaw = self.wrap_pi(self.target_yaw + yaw_alpha * yaw_delta)

    def on_executed(self, msg: Bool):
        if msg.data:
            # Return to a relaxed home pose after execution.
            self.home_mode = True
            self.target_xy = None

    def on_collision(self, msg: Bool):
        self.in_collision = bool(msg.data)
        if self.in_collision and self.retreat_on_collision:
            self.home_mode = True

    def on_state(self, msg: String):
        new_state = str(msg.data).strip().upper() if msg.data else "IDLE"
        if new_state == self.pipeline_state:
            return
        self.pipeline_state = new_state
        self.state_entry_time = time.time()
        if self.use_state_driven_motion and self.pipeline_state != "REPAIR":
            self.target_xy = None

    def tick(self):
        use_ik = False
        if self.use_state_driven_motion:
            desired, use_ik = self.desired_from_state()
        elif self.in_collision and self.use_collision_guard and not self.retreat_on_collision:
            desired = self.current.copy()
        else:
            desired = self.home.copy() if self.home_mode else self.solve_ik()
            use_ik = not self.home_mode

        if not use_ik:
            self.last_singularity_scale = 1.0

        desired = self.clamp_joint_array(desired)
        blended = self.current + self.alpha * (desired - self.current)

        # Velocity limiter to avoid jerky/singular jumps.
        max_step = max(1e-4, self.max_joint_speed / max(self.publish_rate_hz, 1e-3))
        max_step *= self.last_singularity_scale
        delta = blended - self.current
        delta = np.clip(delta, -max_step, max_step)
        self.current += delta
        self.current = self.clamp_joint_array(self.current)
        self.publish_joint_state(self.current)

    def desired_from_state(self):
        if self.in_collision and self.use_collision_guard:
            if self.retreat_on_collision:
                return self.wait_pose.copy(), False
            return self.current.copy(), False

        if self.pipeline_state in ("SCAN", "RESCAN"):
            return self.scan_sweep_pose(), False

        if self.pipeline_state == "REPAIR":
            if (
                self.target_xy is not None
                and (time.time() - self.last_path_update_time) <= self.path_stale_timeout_sec
            ):
                return self.solve_ik(), True
            return self.wait_pose.copy(), False

        return self.wait_pose.copy(), False

    def scan_sweep_pose(self):
        q = self.scan_pose.copy()
        phase = (time.time() - self.state_entry_time) * (2.0 * math.pi / self.scan_cycle_sec)
        q[0] += self.scan_sweep_joint1_rad * math.sin(phase)
        q[3] += self.scan_sweep_joint4_rad * math.sin(phase + math.pi / 2.0)
        return q

    def solve_ik(self) -> np.ndarray:
        if self.target_xy is None:
            return self.home.copy()

        tx, ty = self.target_xy
        # Compensate tool TCP extension so the visible tool tip follows the path.
        tx -= self.tool_tcp_offset * math.cos(self.target_yaw)
        ty -= self.tool_tcp_offset * math.sin(self.target_yaw)
        dx = tx - self.base_x
        dy = ty - self.base_y
        q1_raw = math.atan2(dy, dx)
        q1 = self.current[0] + self.wrap_pi(q1_raw - self.current[0])
        q1 = self.clamp(q1, -2.6, 2.6)

        r = math.hypot(dx, dy)
        r = self.clamp(r, self.safe_min_radius, self.safe_max_radius)
        z_target = max(self.work_z, self.safe_work_z)
        z = z_target - self.shoulder_z

        # Clamp target to reachable annulus.
        dist = math.hypot(r, z)
        r_min = abs(self.l2 - self.l3) + 1e-4
        r_max = self.l2 + self.l3 - 1e-4
        if dist < r_min:
            scale = r_min / max(dist, 1e-6)
            r *= scale
            z *= scale
        elif dist > r_max:
            scale = r_max / dist
            r *= scale
            z *= scale

        c3 = (r * r + z * z - self.l2 * self.l2 - self.l3 * self.l3) / (2.0 * self.l2 * self.l3)
        # Keep away from kinematic singularity at |c3| ~= 1.
        c3 = self.clamp(c3, -1.0 + self.singularity_margin, 1.0 - self.singularity_margin)
        s3_mag = math.sqrt(max(0.0, 1.0 - c3 * c3))

        # Two branches: choose the one closest to current and away from limits.
        branch_candidates = []
        for s3 in (-s3_mag, s3_mag):
            branch_sign = 1.0 if s3 >= 0.0 else -1.0
            q3_c = math.atan2(s3, c3)
            q2_c = math.atan2(z, r) - math.atan2(self.l3 * s3, self.l2 + self.l3 * c3)
            cost = (
                abs(self.wrap_pi(q2_c - self.current[1]))
                + abs(self.wrap_pi(q3_c - self.current[2]))
                + 0.6 * self.joint_limit_penalty(1, q2_c)
                + 0.6 * self.joint_limit_penalty(2, q3_c)
            )
            if branch_sign != self.last_elbow_sign:
                cost += self.elbow_branch_switch_penalty
            branch_candidates.append((cost, q2_c, q3_c, branch_sign))

        _, q2, q3, chosen_branch_sign = min(branch_candidates, key=lambda x: x[0])
        self.last_elbow_sign = chosen_branch_sign

        # Wrist shaping to keep tool roughly downward and aligned with path heading.
        q4 = self.current[3] + self.wrap_pi((self.target_yaw - q1) - self.current[3])
        q5 = self.clamp(-(q2 + q3), -2.2, 2.2)
        q6 = self.current[5] + self.wrap_pi((-0.5 * q4) - self.current[5])

        # Slow down when near singularity.
        self.last_singularity_scale = self.clamp(s3_mag / max(self.singularity_margin, 1e-3), 0.2, 1.0)

        desired = np.array([q1, q2, q3, q4, q5, q6], dtype=float)
        return self.clamp_joint_array(desired)

    def publish_joint_state(self, joints: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(v) for v in joints]
        self.pub_joint_states.publish(msg)

    @staticmethod
    def wrap_pi(x: float) -> float:
        while x > math.pi:
            x -= 2.0 * math.pi
        while x < -math.pi:
            x += 2.0 * math.pi
        return x

    @staticmethod
    def clamp(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def read_pose_param(self, name, fallback):
        raw = self.get_parameter(name).value
        try:
            vals = [float(v) for v in raw]
        except Exception:
            vals = list(fallback)
        if len(vals) != 6:
            self.get_logger().warn(f"{name} must contain 6 values, using fallback.")
            vals = list(fallback)
        return np.array(vals, dtype=float)

    def clamp_joint_array(self, q: np.ndarray) -> np.ndarray:
        return np.minimum(np.maximum(q, self.joint_lower), self.joint_upper)

    def joint_limit_penalty(self, idx: int, q: float) -> float:
        """Penalty increases when a joint candidate is close to limits."""
        lo = self.joint_lower[idx]
        hi = self.joint_upper[idx]
        mid = 0.5 * (lo + hi)
        half = 0.5 * (hi - lo)
        if half <= 1e-6:
            return 0.0
        norm = abs((q - mid) / half)
        return norm * norm


def main():
    rclpy.init()
    node = ArmSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
