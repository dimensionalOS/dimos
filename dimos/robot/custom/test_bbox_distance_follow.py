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

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.custom.bbox_selection import BBoxSelectionModule


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
def module() -> BBoxSelectionModule:
    instance = BBoxSelectionModule(rpc_transport=_NoopRPC)
    try:
        yield instance
    finally:
        instance.stop()


def _make_detection(
    detection_id: str,
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


def _subscribe_selected(module: BBoxSelectionModule) -> list[Any]:
    received: list[Any] = []
    module.selected_bbox.subscribe(received.append)
    return received


def test_camera_click_selects_matching_bbox(module: BBoxSelectionModule) -> None:
    received = _subscribe_selected(module)
    detections = _make_array(
        _make_detection("left", 0.0, 0.0, 100.0, 100.0),
        _make_detection("target", 200.0, 100.0, 260.0, 180.0),
    )

    module._on_detections(detections)
    received.clear()
    module._on_clicked_point(
        PointStamped(x=220.0, y=120.0, z=0.0, frame_id="/world/color_image/detections")
    )

    assert len(received) == 1
    assert received[0].detections_length == 1
    assert received[0].detections[0].id == "target"


def test_camera_click_miss_clears_selection(module: BBoxSelectionModule) -> None:
    received = _subscribe_selected(module)
    detections = _make_array(_make_detection("target", 20.0, 20.0, 120.0, 120.0))

    module._on_detections(detections)
    module.select_bbox(index=0)
    received.clear()
    module._on_clicked_point(PointStamped(x=300.0, y=300.0, z=0.0, frame_id="/world/color_image"))

    assert len(received) == 1
    assert received[0].detections_length == 0
    assert received[0].detections == []


def test_non_camera_click_does_not_change_selection(module: BBoxSelectionModule) -> None:
    received = _subscribe_selected(module)
    detections = _make_array(_make_detection("target", 20.0, 20.0, 120.0, 120.0))

    module._on_detections(detections)
    module.select_bbox(index=0)
    received.clear()
    module._on_clicked_point(PointStamped(x=300.0, y=300.0, z=0.0, frame_id="/world"))
    module._on_detections(detections)

    assert len(received) == 1
    assert received[0].detections_length == 1
    assert received[0].detections[0].id == "target"


def test_overlapping_camera_click_prefers_smaller_bbox(module: BBoxSelectionModule) -> None:
    received = _subscribe_selected(module)
    detections = _make_array(
        _make_detection("large", 0.0, 0.0, 200.0, 200.0),
        _make_detection("small", 50.0, 50.0, 80.0, 80.0),
    )

    module._on_detections(detections)
    received.clear()
    module._on_clicked_point(PointStamped(x=60.0, y=60.0, z=0.0, frame_id="world/color_image"))

    assert len(received) == 1
    assert received[0].detections_length == 1
    assert received[0].detections[0].id == "small"
