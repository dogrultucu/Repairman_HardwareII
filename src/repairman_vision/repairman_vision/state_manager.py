"""
State Manager: Orchestrates the closed-loop repair pipeline.

States:
  IDLE       -> waiting for external trigger or automatic cycle start
  SCAN       -> acquire image (wait for image_pub)
  DETECT     -> run YOLO inference (wait for yolo_node)
  PLAN       -> generate toolpath (wait for toolpath_node)
  REPAIR     -> execute toolpath (wait for execute_node)
  RESCAN     -> wait for re-scan image
  EVALUATE   -> compute quality score
  PASS       -> repair successful, return to IDLE
  REPAIR_AGAIN -> quality < threshold, trigger another detect/plan/repair cycle

Transitions:
  IDLE -> SCAN (on timer or external trigger)
  SCAN -> DETECT (on image received)
  DETECT -> PLAN (on mask received)
  PLAN -> REPAIR (on toolpath received)
  REPAIR -> RESCAN (on execution done)
  RESCAN -> EVALUATE (on re-scan image)
  EVALUATE -> PASS or REPAIR_AGAIN (based on quality score)
  PASS -> IDLE
  REPAIR_AGAIN -> SCAN
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon
from std_msgs.msg import Bool, Float32, String
from enum import Enum
import time


class State(Enum):
    IDLE = 0
    SCAN = 1
    DETECT = 2
    PLAN = 3
    REPAIR = 4
    RESCAN = 5
    EVALUATE = 6
    PASS = 7
    REPAIR_AGAIN = 8


class StateManager(Node):
    def __init__(self):
        super().__init__("state_manager")

        self.declare_parameter("cycle_period_sec", 30.0)  # Auto-trigger new cycle every N seconds
        self.declare_parameter("quality_threshold", 0.5)  # If quality < this, request second pass
        self.declare_parameter("rescan_delay_sec", 2.0)  # Wait after repair before re-scanning
        self.declare_parameter("max_repair_passes", 2)  # Max number of consecutive repair attempts
        self.declare_parameter("auto_start", True)  # If True, automatically start cycles
        self.declare_parameter("verbose", True)

        self.cycle_period = float(self.get_parameter("cycle_period_sec").value)
        self.quality_threshold = float(self.get_parameter("quality_threshold").value)
        self.rescan_delay = float(self.get_parameter("rescan_delay_sec").value)
        self.max_repair_passes = int(self.get_parameter("max_repair_passes").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.verbose = bool(self.get_parameter("verbose").value)

        # Subscriptions
        self.sub_image = self.create_subscription(Image, "/camera/image_raw", self.on_image, 10)
        self.sub_mask = self.create_subscription(Image, "/damage/mask", self.on_mask, 10)
        self.sub_toolpath = self.create_subscription(Polygon, "/repair/toolpath", self.on_toolpath, 10)
        self.sub_executed = self.create_subscription(Bool, "/repair/executed", self.on_executed, 10)
        self.sub_quality = self.create_subscription(Float32, "/repair/quality_score", self.on_quality, 10)

        # Publishers for diagnostic/control
        self.pub_start_trigger = self.create_publisher(Bool, "/repair/start_cycle", 10)
        self.pub_state = self.create_publisher(String, "/repair/state", 10)

        # State machine
        self.state = State.IDLE
        self.last_image_time = 0
        self.last_mask_time = 0
        self.last_toolpath_time = 0
        self.last_executed_time = 0
        self.last_quality_time = 0
        self.repair_pass_count = 0

        # Timeouts to prevent hanging in one state
        self.state_entry_time = time.time()
        self.state_timeout = 60.0  # seconds

        # Timer for auto-cycling and state machine updates
        self.cycle_timer = self.create_timer(1.0, self.update_state_machine)
        self.pub_state.publish(String(data=self.state.name))

        self.log(f"StateManager initialized. Auto-start={self.auto_start}, quality_threshold={self.quality_threshold}")

    def log(self, msg):
        if self.verbose:
            self.get_logger().info(msg)
        else:
            self.get_logger().debug(msg)

    def transition_to(self, new_state):
        """Transition to a new state."""
        old_state = self.state
        self.state = new_state
        self.state_entry_time = time.time()
        self.log(f"[STATE] {old_state.name} -> {new_state.name}")
        self.pub_state.publish(String(data=new_state.name))

    def is_state_timeout(self):
        """Check if current state has exceeded timeout."""
        elapsed = time.time() - self.state_entry_time
        return elapsed > self.state_timeout

    def on_image(self, msg: Image):
        """Handle incoming image messages."""
        self.last_image_time = time.time()
        if self.state == State.SCAN:
            self.transition_to(State.DETECT)

    def on_mask(self, msg: Image):
        """Handle incoming mask messages."""
        self.last_mask_time = time.time()
        if self.state == State.DETECT:
            self.transition_to(State.PLAN)
        elif self.state == State.RESCAN:
            # Re-scan mask acquired, move to evaluation
            self.transition_to(State.EVALUATE)

    def on_toolpath(self, msg: Polygon):
        """Handle incoming toolpath messages."""
        self.last_toolpath_time = time.time()
        if self.state == State.PLAN:
            self.transition_to(State.REPAIR)

    def on_executed(self, msg: Bool):
        """Handle execution completion messages."""
        if msg.data:  # True means execution successful
            self.last_executed_time = time.time()
            if self.state == State.REPAIR:
                self.transition_to(State.RESCAN)

    def on_quality(self, msg: Float32):
        """Handle incoming quality scores."""
        self.last_quality_time = time.time()
        if self.state == State.EVALUATE:
            quality = msg.data
            self.log(f"[QUALITY] Received score: {quality:.3f}")

            if quality >= self.quality_threshold:
                self.log(f"[QUALITY] PASS: quality {quality:.3f} >= threshold {self.quality_threshold}")
                self.transition_to(State.PASS)
            else:
                self.request_additional_pass(
                    f"quality {quality:.3f} < threshold {self.quality_threshold}"
                )

    def request_additional_pass(self, reason: str):
        """Request another repair pass if budget remains; otherwise finish the cycle."""
        if self.repair_pass_count < self.max_repair_passes:
            self.repair_pass_count += 1
            self.log(
                f"[RETRY] {reason}. "
                f"Requesting pass {self.repair_pass_count}/{self.max_repair_passes}."
            )
            self.transition_to(State.REPAIR_AGAIN)
        else:
            self.log(
                f"[RETRY] {reason}. "
                f"Max repair passes ({self.max_repair_passes}) reached. Accepting current quality."
            )
            self.transition_to(State.PASS)

    def update_state_machine(self):
        """Update the finite-state machine."""
        # Check for timeout
        if self.is_state_timeout():
            self.log(f"[TIMEOUT] State {self.state.name} exceeded {self.state_timeout}s, resetting to IDLE")
            self.transition_to(State.IDLE)

        # State-specific logic
        if self.state == State.IDLE:
            # Wait for auto-trigger or external command
            if self.auto_start:
                elapsed_since_idle = time.time() - self.state_entry_time
                if elapsed_since_idle > self.cycle_period:
                    self.log("[IDLE] Auto-triggering new cycle")
                    self.repair_pass_count = 0
                    self.transition_to(State.SCAN)

        elif self.state == State.SCAN:
            # Waiting for image
            elapsed = time.time() - self.state_entry_time
            if elapsed > 2.0:  # 2 second timeout
                self.log("[SCAN] No image received, retrying...")
                self.state_entry_time = time.time()

        elif self.state == State.DETECT:
            # Waiting for mask
            elapsed = time.time() - self.state_entry_time
            if elapsed > 3.0:
                self.log("[DETECT] No mask received, moving to PLAN anyway")
                self.transition_to(State.PLAN)

        elif self.state == State.PLAN:
            # Waiting for toolpath
            elapsed = time.time() - self.state_entry_time
            if elapsed > 3.0:
                self.log("[PLAN] No toolpath received, moving to REPAIR anyway")
                self.transition_to(State.REPAIR)

        elif self.state == State.REPAIR:
            # Waiting for execution done signal (/repair/executed)
            elapsed = time.time() - self.state_entry_time
            if elapsed > 10.0:
                self.log("[REPAIR] No execution completion signal, timing out")
                self.transition_to(State.RESCAN)

        elif self.state == State.RESCAN:
            # Wait a bit for material to cure, then expect a new mask
            elapsed = time.time() - self.state_entry_time
            if elapsed < self.rescan_delay:
                pass  # Still waiting
            else:
                # Timeout after rescan_delay + additional wait
                if elapsed > self.rescan_delay + 3.0:
                    self.log("[RESCAN] Timeout waiting for re-scan mask, moving to EVALUATE")
                    self.transition_to(State.EVALUATE)

        elif self.state == State.EVALUATE:
            # Waiting for quality score
            elapsed = time.time() - self.state_entry_time
            if elapsed > 5.0:
                self.log("[EVALUATE] No quality score received within timeout")
                self.request_additional_pass("missing quality score")

        elif self.state == State.PASS:
            # Repair cycle complete, reset to IDLE
            self.log("[PASS] Repair cycle complete, returning to IDLE")
            self.transition_to(State.IDLE)

        elif self.state == State.REPAIR_AGAIN:
            # Start another full pass (scan -> detect -> plan -> repair)
            self.log(f"[REPAIR_AGAIN] Starting repair pass {self.repair_pass_count + 1}")
            self.transition_to(State.SCAN)


def main():
    rclpy.init()
    node = StateManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
