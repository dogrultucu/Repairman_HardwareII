import time
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from repairman_vision.state_manager import State
from repairman_vision.state_manager import StateManager


def _make_manager_for_retry(max_repair_passes, repair_pass_count):
    manager = object.__new__(StateManager)
    manager.max_repair_passes = max_repair_passes
    manager.repair_pass_count = repair_pass_count
    manager.verbose = False
    transitions = []

    manager.log = lambda _msg: None
    manager.transition_to = lambda state: transitions.append(state)
    return manager, transitions


def test_request_additional_pass_transitions_to_repair_again():
    manager, transitions = _make_manager_for_retry(max_repair_passes=2, repair_pass_count=0)

    manager.request_additional_pass("quality below threshold")

    assert manager.repair_pass_count == 1
    assert transitions == [State.REPAIR_AGAIN]


def test_request_additional_pass_transitions_to_pass_when_budget_exhausted():
    manager, transitions = _make_manager_for_retry(max_repair_passes=2, repair_pass_count=2)

    manager.request_additional_pass("quality below threshold")

    assert manager.repair_pass_count == 2
    assert transitions == [State.PASS]


def test_update_state_machine_moves_repair_again_to_scan():
    manager = object.__new__(StateManager)
    manager.state = State.REPAIR_AGAIN
    manager.repair_pass_count = 1
    manager.verbose = False
    transitions = []
    manager.log = lambda _msg: None
    manager.transition_to = lambda state: transitions.append(state)
    manager.is_state_timeout = lambda: False

    manager.update_state_machine()

    assert transitions == [State.SCAN]


def test_update_state_machine_requests_retry_on_evaluate_timeout():
    manager = object.__new__(StateManager)
    manager.state = State.EVALUATE
    manager.state_entry_time = time.time() - 6.0
    manager.verbose = False
    retry_reasons = []
    manager.log = lambda _msg: None
    manager.is_state_timeout = lambda: False
    manager.request_additional_pass = lambda reason: retry_reasons.append(reason)

    manager.update_state_machine()

    assert retry_reasons == ["missing quality score"]
