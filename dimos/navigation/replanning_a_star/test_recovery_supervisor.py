# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import math
from unittest.mock import Mock, patch

import pytest

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.base import NavigationState
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner
from dimos.navigation.replanning_a_star.recovery_supervisor import (
    RecoveryAction,
    RecoveryCapabilities,
    RecoveryCause,
    RecoveryEvent,
    RecoveryOutcome,
    RecoverySupervisor,
)


def _pose(x: float, yaw_degrees: float = 0.0) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, 0.0, 0.0),
        orientation=Quaternion.from_euler(
            Vector3(0.0, 0.0, math.radians(yaw_degrees))
        ),
    )


@pytest.mark.parametrize(
    ("attempt", "expected_action", "expected_degrees"),
    [
        (0, RecoveryAction.REPLAN, None),
        (1, RecoveryAction.ROTATE_RESCAN, 90.0),
        (2, RecoveryAction.REPLAN, None),
        (3, RecoveryAction.ROTATE_RESCAN, -90.0),
        (7, RecoveryAction.ROTATE_RESCAN, -135.0),
    ],
)
def test_default_recovery_sequence_is_bounded_and_scan_alternates(
    attempt: int,
    expected_action: RecoveryAction,
    expected_degrees: float | None,
) -> None:
    decision = RecoverySupervisor().decide(
        attempt=attempt,
        cause=RecoveryCause.OBSTACLE,
        capabilities=RecoveryCapabilities(),
    )

    assert decision.action is expected_action
    if expected_degrees is None:
        assert decision.heading_offset_radians is None
    else:
        assert math.degrees(decision.heading_offset_radians or 0.0) == pytest.approx(
            expected_degrees
        )


def test_stale_local_clear_requires_exact_capability() -> None:
    supervisor = RecoverySupervisor()

    unavailable = supervisor.decide(
        attempt=4,
        cause=RecoveryCause.PROGRESS_TIMEOUT,
        capabilities=RecoveryCapabilities(),
    )
    available = supervisor.decide(
        attempt=4,
        cause=RecoveryCause.PROGRESS_TIMEOUT,
        capabilities=RecoveryCapabilities(can_clear_stale_local_observations=True),
    )

    assert unavailable.action is RecoveryAction.REPLAN
    assert RecoveryAction.CLEAR_STALE_LOCAL in unavailable.skipped_actions
    assert available.action is RecoveryAction.CLEAR_STALE_LOCAL


def test_backup_requires_verified_rear_clearance() -> None:
    supervisor = RecoverySupervisor()

    unavailable = supervisor.decide(
        attempt=5,
        cause=RecoveryCause.OBSTACLE,
        capabilities=RecoveryCapabilities(),
    )
    available = supervisor.decide(
        attempt=5,
        cause=RecoveryCause.OBSTACLE,
        capabilities=RecoveryCapabilities(rear_clearance_verified=True),
    )

    assert unavailable.action is RecoveryAction.ROTATE_RESCAN
    assert RecoveryAction.BACK_UP in unavailable.skipped_actions
    assert available.action is RecoveryAction.BACK_UP


def test_alternate_approach_requires_provider() -> None:
    supervisor = RecoverySupervisor()

    unavailable = supervisor.decide(
        attempt=6,
        cause=RecoveryCause.PATH_DEVIATION,
        capabilities=RecoveryCapabilities(),
    )
    available = supervisor.decide(
        attempt=6,
        cause=RecoveryCause.PATH_DEVIATION,
        capabilities=RecoveryCapabilities(alternate_approach_available=True),
    )

    assert unavailable.action is RecoveryAction.REPLAN
    assert RecoveryAction.ALTERNATE_APPROACH in unavailable.skipped_actions
    assert available.action is RecoveryAction.ALTERNATE_APPROACH


def test_attempt_limit_returns_typed_failure() -> None:
    decision = RecoverySupervisor(max_attempts=8).decide(
        attempt=8,
        cause=RecoveryCause.PLANNER_ERROR,
        capabilities=RecoveryCapabilities(
            can_clear_stale_local_observations=True,
            rear_clearance_verified=True,
            alternate_approach_available=True,
        ),
    )

    assert decision.action is RecoveryAction.FAIL
    assert decision.failure_reason == "attempts_exhausted"
    assert decision.heading_offset_radians is None


def test_recovery_event_serializes_required_diagnostics() -> None:
    event = RecoveryEvent(
        attempt=2,
        cause=RecoveryCause.OBSTACLE,
        action=RecoveryAction.ROTATE_RESCAN,
        outcome=RecoveryOutcome.DISPATCHED,
        reason="refresh_local_perception",
        timestamp=123.5,
    )

    payload = json.loads(event.to_json())

    assert payload == {
        "action": "rotate_rescan",
        "attempt": 2,
        "cause": "obstacle",
        "outcome": "dispatched",
        "reason": "refresh_local_perception",
        "timestamp": 123.5,
    }


def test_rotate_rescan_changes_only_first_heading_of_planned_path() -> None:
    planner = GlobalPlanner(GlobalConfig())
    path = Path(poses=[_pose(0.0), _pose(1.0, 10.0), _pose(2.0, 25.0)])
    current_odom = _pose(0.0, 15.0)

    adjusted = planner._apply_recovery_heading(
        path,
        current_odom,
        math.radians(90.0),
    )

    assert adjusted is path
    assert math.degrees(path.poses[0].orientation.euler.z) == pytest.approx(105.0)
    assert math.degrees(path.poses[1].orientation.euler.z) == pytest.approx(10.0)
    assert math.degrees(path.poses[-1].orientation.euler.z) == pytest.approx(25.0)
    assert [pose.position.x for pose in path.poses] == [0.0, 1.0, 2.0]


def test_replan_dispatches_recovery_without_cancelling_goal() -> None:
    planner = GlobalPlanner(GlobalConfig())
    planner._current_odom = _pose(0.0)
    planner._current_goal = _pose(2.0)
    planner._plan_path = Mock(return_value=True)
    events: list[RecoveryEvent] = []
    planner.recovery_event.subscribe(events.append)

    planner._replan_path(RecoveryCause.OBSTACLE)

    planner._plan_path.assert_called_once_with(
        recovery_heading_offset=None,
        cancel_on_failure=False,
    )
    assert planner._current_goal is not None
    assert planner._replan_limiter.get_attempt() == 1
    assert planner.get_state() is NavigationState.RECOVERY
    assert events[-1].action is RecoveryAction.REPLAN
    assert events[-1].outcome is RecoveryOutcome.DISPATCHED


def test_second_same_area_failure_dispatches_rotate_rescan() -> None:
    planner = GlobalPlanner(GlobalConfig())
    planner._current_odom = _pose(0.0)
    planner._current_goal = _pose(2.0)
    planner._replan_limiter.will_retry()
    planner._plan_path = Mock(return_value=True)
    events: list[RecoveryEvent] = []
    planner.recovery_event.subscribe(events.append)

    planner._replan_path(RecoveryCause.PROGRESS_TIMEOUT)

    heading_offset = planner._plan_path.call_args.kwargs["recovery_heading_offset"]
    assert math.degrees(heading_offset) == pytest.approx(90.0)
    assert planner._current_goal is not None
    assert events[-1].action is RecoveryAction.ROTATE_RESCAN
    assert events[-1].outcome is RecoveryOutcome.DISPATCHED


def test_exhausted_recovery_emits_typed_failure_and_cancels_goal() -> None:
    planner = GlobalPlanner(GlobalConfig())
    planner._current_odom = _pose(0.0)
    planner._current_goal = _pose(2.0)
    for _ in range(8):
        planner._replan_limiter.will_retry()
    events: list[RecoveryEvent] = []
    planner.recovery_event.subscribe(events.append)

    planner._replan_path(RecoveryCause.PLANNER_ERROR)

    assert planner._current_goal is None
    assert planner.get_state() is NavigationState.IDLE
    assert events[-1].action is RecoveryAction.FAIL
    assert events[-1].outcome is RecoveryOutcome.FAILED
    assert events[-1].reason == "attempts_exhausted"


def test_cancel_preempts_recovery_and_stops_local_planner() -> None:
    planner = GlobalPlanner(GlobalConfig())
    stop_planning = Mock(wraps=planner._local_planner.stop_planning)
    planner._local_planner.stop_planning = stop_planning
    planner._current_goal = _pose(2.0)
    planner._recovery_active = True

    assert planner.get_state() is NavigationState.RECOVERY

    planner.cancel_goal()

    assert planner.get_state() is NavigationState.IDLE
    stop_planning.assert_called_once_with()


def test_cancel_during_path_computation_prevents_stale_path_dispatch() -> None:
    planner = GlobalPlanner(GlobalConfig())
    planner._current_odom = _pose(0.0)
    planner._current_goal = _pose(2.0)
    planner._goal_revision = 1
    planner._find_safe_goal = Mock(return_value=Vector3(2.0, 0.0, 0.0))

    def cancel_then_return_path(*args: object) -> Path:
        del args
        planner.cancel_goal()
        return Path(poses=[_pose(0.0), _pose(2.0)])

    planner._find_wide_path = Mock(side_effect=cancel_then_return_path)
    start_planning = Mock()
    planner._local_planner.start_planning = start_planning

    with patch(
        "dimos.navigation.replanning_a_star.global_planner.smooth_resample_path",
        return_value=Path(poses=[_pose(0.0), _pose(2.0)]),
    ):
        planned = planner._plan_path()

    assert planned is False
    start_planning.assert_not_called()
