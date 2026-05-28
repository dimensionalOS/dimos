# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from dimos_lcm.std_msgs import Bool
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.base import NavigationState
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.custom.tasks import target_standoff_behavior_module as behavior_module
from dimos.robot.custom.tasks.target_standoff_behavior_module import (
    TargetStandoffBehaviorModule,
)


class _NoopRPC(RPCSpec):
    def __init__(
        self,
        *,
        rpc_timeouts: dict[str, float] | None = None,
        default_rpc_timeout: float = 120.0,
    ) -> None:
        self.rpc_timeouts = {} if rpc_timeouts is None else dict(rpc_timeouts)
        self.default_rpc_timeout = default_rpc_timeout

    def serve_module_rpc(self, module: Any, name: str | None = None) -> None:
        pass

    def serve_rpc(self, f: Callable[..., Any], name: str) -> Callable[[], None]:
        return lambda: None

    def call(
        self,
        name: str,
        arguments: tuple[list[Any], dict[str, Any]],
        cb: Callable[[Any], None] | None,
    ) -> Callable[[], None] | None:
        return (lambda: None) if cb is not None else None

    def call_nowait(self, name: str, arguments: tuple[list[Any], dict[str, Any]]) -> None:
        pass

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


class _FakePlanner:
    def __init__(self) -> None:
        self.goals: list[PoseStamped] = []
        self.cancel_count = 0
        self.goal_reached = False
        self.state = NavigationState.IDLE

    def set_goal(self, goal: PoseStamped) -> bool:
        self.goals.append(goal)
        self.goal_reached = False
        self.state = NavigationState.FOLLOWING_PATH
        return True

    def get_state(self) -> NavigationState:
        return self.state

    def is_goal_reached(self) -> bool:
        return self.goal_reached

    def cancel_goal(self) -> bool:
        self.cancel_count += 1
        self.state = NavigationState.IDLE
        return True

    def set_replanning_enabled(self, enabled: bool) -> None:
        pass

    def set_safe_goal_clearance(self, clearance: float) -> None:
        pass

    def reset_safe_goal_clearance(self) -> None:
        pass


@pytest.fixture()
def module(monkeypatch: pytest.MonkeyPatch) -> TargetStandoffBehaviorModule:
    instance = TargetStandoffBehaviorModule(rpc_transport=_NoopRPC)
    instance._planner = _FakePlanner()
    monkeypatch.setattr(
        instance,
        "_get_robot_transform",
        lambda: Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            frame_id="world",
            child_frame_id="base_link",
        ),
    )
    try:
        yield instance
    finally:
        instance.stop()


def _pose(x: float, y: float = 0.0, z: float = 0.2) -> PoseStamped:
    return PoseStamped(
        ts=123.0,
        frame_id="world",
        position=Vector3(x, y, z),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )


def _planner(module: TargetStandoffBehaviorModule) -> _FakePlanner:
    assert isinstance(module._planner, _FakePlanner)
    return module._planner


def test_selected_target_computes_target_near_far_and_dispatches_near_goal(
    module: TargetStandoffBehaviorModule,
) -> None:
    module._on_target_pose(_pose(2.0))

    planner = _planner(module)
    assert module._state == "navigating_near"
    assert len(planner.goals) == 1
    assert module._target_pose is not None
    assert module._near_pose is not None
    assert module._far_pose is not None
    assert module._target_pose.position.x == pytest.approx(2.0)
    assert module._near_pose.position.x == pytest.approx(1.5)
    assert module._far_pose.position.x == pytest.approx(0.5)
    assert planner.goals[0].position.x == pytest.approx(1.5)


def test_full_waypoint_sequence_uses_planner_arrival_and_dwell_timers(
    module: TargetStandoffBehaviorModule,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = {"value": 100.0}
    monkeypatch.setattr(behavior_module.time, "monotonic", lambda: now["value"])
    clear_requests: list[Any] = []
    module.clear_selection_request.subscribe(clear_requests.append)

    module._on_target_pose(_pose(2.0))
    planner = _planner(module)
    assert module._state == "navigating_near"

    planner.goal_reached = True
    module._tick()
    assert module._state == "dwelling_near"
    assert len(planner.goals) == 1

    now["value"] = 109.0
    module._tick()
    assert module._state == "dwelling_near"
    assert len(planner.goals) == 1

    now["value"] = 110.1
    module._tick()
    assert module._state == "navigating_far"
    assert len(planner.goals) == 2
    assert planner.goals[1].position.x == pytest.approx(0.5)

    planner.goal_reached = True
    module._tick()
    assert module._state == "dwelling_far"

    now["value"] = 120.2
    module._tick()
    assert module._state == "returning_near"
    assert len(planner.goals) == 3
    assert planner.goals[2].position.x == pytest.approx(1.5)

    planner.goal_reached = True
    module._tick()
    assert module._state == "done"
    assert len(clear_requests) == 1


def test_dwell_countdown_logs_when_started_and_every_two_seconds(
    module: TargetStandoffBehaviorModule,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = {"value": 100.0}
    log_messages: list[str] = []
    monkeypatch.setattr(behavior_module.time, "monotonic", lambda: now["value"])
    monkeypatch.setattr(behavior_module.logger, "info", lambda message: log_messages.append(message))

    module._on_target_pose(_pose(2.0))
    planner = _planner(module)
    planner.goal_reached = True
    module._tick()
    assert module._state == "dwelling_near"

    module._tick()
    assert sum("dwell countdown" in message for message in log_messages) == 1

    now["value"] = 101.0
    module._tick()
    assert sum("dwell countdown" in message for message in log_messages) == 1

    now["value"] = 102.1
    module._tick()
    assert sum("dwell countdown" in message for message in log_messages) == 2


def test_missing_robot_tf_blocks_sequence(
    module: TargetStandoffBehaviorModule,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(module, "_get_robot_transform", lambda: None)

    module._on_target_pose(_pose(2.0))

    assert module._state == "idle"
    assert _planner(module).goals == []


def test_teleop_interrupts_sequence_and_clears_selection(
    module: TargetStandoffBehaviorModule,
) -> None:
    clear_requests: list[Any] = []
    module.clear_selection_request.subscribe(clear_requests.append)
    module._on_target_pose(_pose(2.0))

    module._on_teleop_active(Bool(data=True))

    assert module._state == "idle"
    assert _planner(module).cancel_count == 1
    assert len(clear_requests) == 1


def test_stop_behavior_cancels_planner_and_clears_selection(
    module: TargetStandoffBehaviorModule,
) -> None:
    clear_requests: list[Any] = []
    module.clear_selection_request.subscribe(clear_requests.append)
    module._on_target_pose(_pose(2.0))

    result = module.stop_behavior()

    assert result == "target standoff behavior stopped"
    assert module._state == "idle"
    assert _planner(module).cancel_count == 1
    assert len(clear_requests) == 1


def test_goal_timeout_fails_sequence(
    module: TargetStandoffBehaviorModule,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    now = {"value": 100.0}
    module.config.goal_timeout_sec = 2.0
    monkeypatch.setattr(behavior_module.time, "monotonic", lambda: now["value"])
    module._on_target_pose(_pose(2.0))

    now["value"] = 103.0
    module._tick()

    assert module._state == "failed"
    assert _planner(module).cancel_count == 1
