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

from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    Detection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
import pytest

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.custom.tasks.bbox_distance_behavior_module import (
    BBoxDistanceBehaviorModule,
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


@pytest.fixture()
def module() -> BBoxDistanceBehaviorModule:
    instance = BBoxDistanceBehaviorModule(rpc_transport=_NoopRPC)
    try:
        yield instance
    finally:
        instance.stop()


def _make_detection(detection_id: str, x1: float, y1: float, x2: float, y2: float) -> Detection2D:
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
            ObjectHypothesisWithPose(
                hypothesis=ObjectHypothesis(class_id="person", score=0.9)
            )
        ],
    )


def _make_array(*detections: Detection2D) -> Detection2DArray:
    return Detection2DArray(
        detections_length=len(detections),
        header=Header(123.0, "camera_optical"),
        detections=list(detections),
    )


def _subscribe_status(module: BBoxDistanceBehaviorModule) -> list[Any]:
    received: list[Any] = []
    module.behavior_status.subscribe(received.append)
    return received


def test_selected_bbox_auto_starts_approach(module: BBoxDistanceBehaviorModule) -> None:
    status = _subscribe_status(module)
    module._on_lidar(PointCloud2())
    module._on_camera_info(CameraInfo.from_intrinsics(100.0, 100.0, 50.0, 50.0, 100, 100))

    module._on_selected_bbox(_make_array(_make_detection("target", 40.0, 40.0, 60.0, 60.0)))

    assert module._state == "approaching"
    assert status


def test_selected_bbox_reaches_point_two_and_finishes(
    module: BBoxDistanceBehaviorModule,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module._on_lidar(PointCloud2())
    module._on_camera_info(CameraInfo.from_intrinsics(100.0, 100.0, 50.0, 50.0, 100, 100))
    module._on_selected_bbox(_make_array(_make_detection("target", 40.0, 40.0, 60.0, 60.0)))

    distances = [0.30, 0.24]

    monkeypatch.setattr(
        module,
        "_estimate_bbox_distance",
        lambda *args, **kwargs: distances.pop(0),
    )

    twist = module._compute_next_twist()

    assert twist is not None
    assert twist.linear.x > 0.0
    done_twist = module._compute_next_twist()
    assert done_twist is not None
    assert done_twist.linear.x == 0.0
    assert done_twist.angular.z == 0.0
    assert module._state == "done"


def test_empty_selected_bbox_resets_to_idle(module: BBoxDistanceBehaviorModule) -> None:
    module._on_selected_bbox(_make_array(_make_detection("target", 40.0, 40.0, 60.0, 60.0)))
    module._on_selected_bbox(_make_array())

    assert module._state == "idle"