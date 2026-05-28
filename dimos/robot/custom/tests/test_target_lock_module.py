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
from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    Detection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
import pytest

from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.custom.modules.target_lock_module import TargetLockModule


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


@pytest.fixture()
def module() -> TargetLockModule:
    instance = TargetLockModule(rpc_transport=_NoopRPC, search_timeout_sec=0.5)
    try:
        yield instance
    finally:
        instance.stop()


def _make_detection(
    detection_id: str,
    class_id: str,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> Detection2D:
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    return Detection2D(
        id=detection_id,
        results_length=1,
        header=Header(123.0, "camera_optical"),
        bbox=BoundingBox2D(
            center=Pose2D(position=Point2D(x=center_x, y=center_y), theta=0.0),
            size_x=x2 - x1,
            size_y=y2 - y1,
        ),
        results=[
            ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id=class_id, score=0.9))
        ],
    )


def _make_array(*detections: Detection2D) -> Detection2DArray:
    return Detection2DArray(
        detections_length=len(detections),
        header=Header(123.0, "camera_optical"),
        detections=list(detections),
    )


def test_transition_unselected_to_locked(module: TargetLockModule) -> None:
    selected = _make_array(_make_detection("target", "person", 10.0, 10.0, 40.0, 40.0))

    module._on_selected_bbox(selected)

    assert module.get_lock_state()["state"] == "locked"


def test_stop_movement_clears_locked_bbox(module: TargetLockModule) -> None:
    selected = _make_array(_make_detection("target", "person", 10.0, 10.0, 40.0, 40.0))
    locked_messages: list[Any] = []
    module.locked_bbox.subscribe(locked_messages.append)

    module._on_selected_bbox(selected)
    module._on_stop_movement(Bool(data=True))

    assert module.get_lock_state()["state"] == "unselected"
    assert locked_messages[-1].detections_length == 0


def test_transition_locked_to_searching(module: TargetLockModule) -> None:
    selected = _make_array(_make_detection("target", "person", 10.0, 10.0, 40.0, 40.0))
    module._on_selected_bbox(selected)

    # No candidates and still inside timeout => searching
    module._on_detections(_make_array())

    assert module.get_lock_state()["state"] == "searching"


def test_transition_searching_to_locked_by_reacquire(module: TargetLockModule) -> None:
    selected = _make_array(_make_detection("target", "person", 10.0, 10.0, 40.0, 40.0))
    module._on_selected_bbox(selected)
    module._on_detections(_make_array())
    assert module.get_lock_state()["state"] == "searching"

    # Different id but same class and near last center => reacquire and lock
    module._on_detections(_make_array(_make_detection("new-id", "person", 12.0, 12.0, 42.0, 42.0)))

    state = module.get_lock_state()
    assert state["state"] == "locked"
    assert state["target_id"] == "new-id"


def test_transition_searching_to_lost_after_timeout(
    module: TargetLockModule,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Control monotonic time deterministically.
    now = {"value": 100.0}

    def _monotonic() -> float:
        return now["value"]

    monkeypatch.setattr("dimos.robot.custom.modules.target_lock_module.time.monotonic", _monotonic)

    selected = _make_array(_make_detection("target", "person", 10.0, 10.0, 40.0, 40.0))
    module._on_selected_bbox(selected)

    # No match and still inside timeout => searching
    now["value"] = 100.1
    module._on_detections(_make_array())
    assert module.get_lock_state()["state"] == "searching"

    # Beyond timeout => lost
    now["value"] = 101.0
    module._on_detections(_make_array())
    assert module.get_lock_state()["state"] == "lost"
