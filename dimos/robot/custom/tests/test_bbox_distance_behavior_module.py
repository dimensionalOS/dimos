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

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    Detection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
import numpy as np
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
            ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id="person", score=0.9))
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


def _subscribe_clear_requests(module: BBoxDistanceBehaviorModule) -> list[Any]:
    received: list[Any] = []
    module.clear_selection_request.subscribe(received.append)
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
    clear_requests = _subscribe_clear_requests(module)
    module._on_lidar(PointCloud2())
    module._on_camera_info(CameraInfo.from_intrinsics(100.0, 100.0, 50.0, 50.0, 100, 100))
    module._on_selected_bbox(_make_array(_make_detection("target", 40.0, 40.0, 60.0, 60.0)))

    distances = [0.30, 0.24]

    monkeypatch.setattr(
        module,
        "_estimate_3d_distance",
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
    assert clear_requests and clear_requests[-1].data


def test_empty_selected_bbox_resets_to_idle(module: BBoxDistanceBehaviorModule) -> None:
    cmd_published: list = []
    module.cmd_vel.subscribe(cmd_published.append)

    module._on_selected_bbox(_make_array(_make_detection("target", 40.0, 40.0, 60.0, 60.0)))
    module._on_selected_bbox(_make_array())

    assert module._state == "idle"
    assert cmd_published and cmd_published[-1].linear.x == 0.0


def test_empty_selected_bbox_while_idle_stays_silent(
    module: BBoxDistanceBehaviorModule,
) -> None:
    cmd_published: list = []
    module.cmd_vel.subscribe(cmd_published.append)

    module._on_selected_bbox(_make_array())

    assert module._state == "idle"
    assert cmd_published == []


def test_estimate_3d_distance_returns_none_without_tf(module: BBoxDistanceBehaviorModule) -> None:
    """_estimate_3d_distance returns None when TF transform is not available."""
    detection = _make_detection("target", 100.0, 100.0, 150.0, 150.0)
    lidar = PointCloud2.from_numpy(np.array([[1.0, 2.0, 0.5]], dtype=np.float32), frame_id="world")
    camera_info = CameraInfo.from_intrinsics(100.0, 100.0, 50.0, 50.0, 640, 480)

    # tf.get returns None by default since no TF data is published in tests
    distance = module._estimate_3d_distance(detection, lidar, camera_info)

    assert distance is None


def test_teleop_active_interrupts_approaching_task(module: BBoxDistanceBehaviorModule) -> None:
    """_on_teleop_active resets an approaching task to idle."""
    module._on_selected_bbox(_make_array(_make_detection("target", 40.0, 40.0, 60.0, 60.0)))
    assert module._state == "approaching"

    cmd_published: list = []
    clear_requests = _subscribe_clear_requests(module)
    module.cmd_vel.subscribe(cmd_published.append)

    module._on_teleop_active(Bool(data=True))

    assert module._state == "idle"
    assert module._active_target_id is None
    # A zero Twist must be published to stop the robot
    assert cmd_published and cmd_published[-1].linear.x == 0.0
    assert clear_requests and clear_requests[-1].data


def test_teleop_active_noop_when_already_idle(module: BBoxDistanceBehaviorModule) -> None:
    """_on_teleop_active does nothing when task is already idle."""
    assert module._state == "idle"
    cmd_published: list = []
    module.cmd_vel.subscribe(cmd_published.append)

    module._on_teleop_active(Bool(data=True))

    assert module._state == "idle"
    assert not cmd_published  # no spurious zero published
