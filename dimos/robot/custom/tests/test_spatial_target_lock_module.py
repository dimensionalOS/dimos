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

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.custom.modules.spatial_target_lock_module import SpatialTargetLockModule


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
def module() -> SpatialTargetLockModule:
    instance = SpatialTargetLockModule(rpc_transport=_NoopRPC, reacquire_distance_m=0.5)
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


def _pose(x: float, y: float = 0.0, z: float = 0.0) -> PoseStamped:
    return PoseStamped(
        ts=123.0,
        frame_id="world",
        position=Vector3(x, y, z),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )


def test_selected_bbox_projects_to_locked_target_pose(
    module: SpatialTargetLockModule,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target_messages: list[PoseStamped] = []
    locked_messages: list[Any] = []
    module.target_pose.subscribe(target_messages.append)
    module.locked_bbox.subscribe(locked_messages.append)
    monkeypatch.setattr(module, "_project_detection_pose", lambda _detection: _pose(1.0))

    module._on_selected_bbox(_make_array(_make_detection("target", "person", 10, 10, 40, 40)))

    assert module.get_lock_state()["state"] == "locked"
    assert target_messages[-1].position.x == pytest.approx(1.0)
    assert locked_messages[-1].detections_length == 1


def test_empty_selected_bbox_after_lock_keeps_spatial_memory(
    module: SpatialTargetLockModule,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(module, "_project_detection_pose", lambda _detection: _pose(1.0))

    module._on_selected_bbox(_make_array(_make_detection("target", "person", 10, 10, 40, 40)))
    module._on_selected_bbox(_make_array())

    state = module.get_lock_state()
    assert state["state"] == "locked"
    assert state["last_pose"]["x"] == pytest.approx(1.0)


def test_empty_detections_publish_memory_pose(
    module: SpatialTargetLockModule,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target_messages: list[PoseStamped] = []
    locked_messages: list[Any] = []
    module.target_pose.subscribe(target_messages.append)
    module.locked_bbox.subscribe(locked_messages.append)
    monkeypatch.setattr(module, "_project_detection_pose", lambda _detection: _pose(1.0))

    module._on_selected_bbox(_make_array(_make_detection("target", "person", 10, 10, 40, 40)))
    module._on_detections(_make_array())

    assert module.get_lock_state()["state"] == "using_memory"
    assert target_messages[-1].position.x == pytest.approx(1.0)
    assert locked_messages[-1].detections_length == 0


def test_detections_after_lock_keep_initial_memory_pose(
    module: SpatialTargetLockModule,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    poses = {
        "target": _pose(1.0),
        "new-id": _pose(1.2),
        "far": _pose(4.0),
    }

    def _project(detection: Detection2D) -> PoseStamped:
        return poses[detection.id]

    monkeypatch.setattr(module, "_project_detection_pose", _project)

    module._on_selected_bbox(_make_array(_make_detection("target", "person", 10, 10, 40, 40)))
    module._on_detections(
        _make_array(
            _make_detection("far", "person", 100, 100, 130, 130),
            _make_detection("new-id", "person", 12, 12, 42, 42),
        )
    )

    state = module.get_lock_state()
    assert state["state"] == "using_memory"
    assert state["target_id"] == "target"
    assert state["last_pose"]["x"] == pytest.approx(1.0)


def test_spatial_reacquire_rejects_candidate_with_large_z_jump(
    module: SpatialTargetLockModule,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    poses = {
        "target": _pose(1.0, z=0.6),
        "ground": _pose(1.1, z=0.05),
    }

    def _project(detection: Detection2D) -> PoseStamped:
        return poses[detection.id]

    monkeypatch.setattr(module, "_project_detection_pose", _project)

    module._on_selected_bbox(_make_array(_make_detection("target", "person", 10, 10, 40, 40)))
    module._on_detections(_make_array(_make_detection("ground", "person", 12, 12, 42, 42)))

    state = module.get_lock_state()
    assert state["state"] == "using_memory"
    assert state["target_id"] == "target"
    assert state["last_pose"]["x"] == pytest.approx(1.0)
    assert state["last_pose"]["z"] == pytest.approx(0.6)


def test_clear_request_clears_lock(
    module: SpatialTargetLockModule,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    locked_messages: list[Any] = []
    module.locked_bbox.subscribe(locked_messages.append)
    monkeypatch.setattr(module, "_project_detection_pose", lambda _detection: _pose(1.0))

    module._on_selected_bbox(_make_array(_make_detection("target", "person", 10, 10, 40, 40)))
    module._on_clear_selection_request(Bool(data=True))

    assert module.get_lock_state()["state"] == "unselected"
    assert module.get_lock_state()["last_pose"] is None
    assert locked_messages[-1].detections_length == 0
