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

from collections.abc import Iterator
from typing import Any

from dimos_lcm.std_msgs import Bool
import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.simulation.dimsim.perceive_loop_skill import DimSimPerceiveLoopSkill


@pytest.fixture
def perceive_loop_skill(mocker: Any) -> Iterator[tuple[DimSimPerceiveLoopSkill, Any]]:
    model = mocker.Mock()
    mocker.patch("dimos.perception.perceive_loop_skill.create", return_value=model)
    skill = DimSimPerceiveLoopSkill()
    try:
        yield skill, model
    finally:
        skill.stop()


def _detections(image: Image, name: str) -> ImageDetections2D[Detection2DBBox]:
    result: ImageDetections2D[Detection2DBBox] = ImageDetections2D(image)
    result.detections.append(
        Detection2DBBox(
            bbox=(10.0, 10.0, 20.0, 20.0),
            track_id=0,
            class_id=-1,
            confidence=1.0,
            name=name,
            ts=image.ts,
            image=image,
        )
    )
    return result


def _usable_image(*, ts: float | None = None) -> Image:
    kwargs = {"data": np.full((40, 40, 3), 128, dtype=np.uint8)}
    if ts is not None:
        kwargs["ts"] = ts
    return Image(**kwargs)


def test_lookout_queries_each_description_as_plain_detector_text(
    mocker: Any,
    perceive_loop_skill: tuple[DimSimPerceiveLoopSkill, Any],
) -> None:
    skill, model = perceive_loop_skill
    image = _usable_image()
    empty: ImageDetections2D[Detection2DBBox] = ImageDetections2D(image)
    model.query_detections.side_effect = [empty, _detections(image, "shower")]
    tool_update = mocker.patch.object(skill, "tool_update")
    stop_tool = mocker.patch.object(skill, "stop_tool")
    stop_movement = mocker.patch.object(skill.stop_movement, "publish")
    spatial_memory = mocker.patch.object(skill, "_spatial_memory", create=True)
    skill._on_odom(
        PoseStamped(
            ts=image.ts - 0.05,
            position=Vector3(1.0, 2.0, 0.5),
        )
    )
    skill._active_lookout = ("bathtub", "shower")

    skill._on_image(image)

    assert model.query_detections.call_args_list == [
        mocker.call(image, "bathtub"),
        mocker.call(image, "shower"),
    ]
    tool_update.assert_called_once_with(
        "look_out_for",
        'Found a match for ["bathtub", "shower"]. Please announce audibly.',
    )
    stop_tool.assert_called_once_with("look_out_for")
    stop_movement.assert_called_once()
    assert stop_movement.call_args.args[0].data is True
    spatial_memory.record_detection_viewpoint.assert_called_once()
    assert spatial_memory.record_detection_viewpoint.call_args.args == (
        ["bathtub", "shower"],
        skill._odom_history[-1],
        image.ts,
    )


def test_lookout_does_not_notify_when_plain_queries_do_not_match(
    mocker: Any,
    perceive_loop_skill: tuple[DimSimPerceiveLoopSkill, Any],
) -> None:
    skill, model = perceive_loop_skill
    image = _usable_image()
    model.query_detections.return_value = ImageDetections2D(image)
    tool_update = mocker.patch.object(skill, "tool_update")
    stop_tool = mocker.patch.object(skill, "stop_tool")
    stop_movement = mocker.patch.object(skill.stop_movement, "publish")
    skill._active_lookout = ("bathtub",)

    skill._on_image(image)

    model.query_detections.assert_called_once_with(image, "bathtub")
    tool_update.assert_not_called()
    stop_tool.assert_not_called()
    stop_movement.assert_not_called()
    assert skill._active_lookout == ("bathtub",)


def test_lookout_correlates_detection_with_frame_pose_not_completion_pose(
    mocker: Any,
    perceive_loop_skill: tuple[DimSimPerceiveLoopSkill, Any],
) -> None:
    skill, model = perceive_loop_skill
    image = _usable_image(ts=100.0)
    capture_pose = PoseStamped(ts=99.95, position=Vector3(1.0, 2.0, 0.5))
    completion_pose = PoseStamped(ts=104.0, position=Vector3(8.0, 9.0, 0.5))
    skill._on_odom(capture_pose)
    skill._on_odom(completion_pose)
    model.query_detections.return_value = _detections(image, "bathtub")
    mocker.patch.object(skill.stop_movement, "publish")
    spatial_memory = mocker.patch.object(skill, "_spatial_memory", create=True)
    skill._active_lookout = ("bathtub",)

    skill._on_image(image)

    spatial_memory.record_detection_viewpoint.assert_called_once_with(
        ["bathtub"],
        capture_pose,
        image.ts,
    )


def test_lookout_stops_motion_even_without_correlated_odometry(
    mocker: Any,
    perceive_loop_skill: tuple[DimSimPerceiveLoopSkill, Any],
) -> None:
    skill, model = perceive_loop_skill
    image = _usable_image(ts=100.0)
    model.query_detections.return_value = _detections(image, "bathtub")
    stop_movement = mocker.patch.object(skill.stop_movement, "publish")
    spatial_memory = mocker.patch.object(skill, "_spatial_memory", create=True)
    skill._active_lookout = ("bathtub",)

    skill._on_image(image)

    stop_movement.assert_called_once()
    assert isinstance(stop_movement.call_args.args[0], Bool)
    spatial_memory.record_detection_viewpoint.assert_not_called()


def test_lookout_ignores_mostly_black_camera_frame(
    mocker: Any,
    perceive_loop_skill: tuple[DimSimPerceiveLoopSkill, Any],
) -> None:
    skill, model = perceive_loop_skill
    data = np.zeros((100, 100, 3), dtype=np.uint8)
    data[80:, :, :] = 128
    image = Image(data=data)
    skill._active_lookout = ("bathtub",)
    tool_update = mocker.patch.object(skill, "tool_update")
    stop_movement = mocker.patch.object(skill.stop_movement, "publish")

    skill._on_image(image)

    model.query_detections.assert_not_called()
    tool_update.assert_not_called()
    stop_movement.assert_not_called()
    assert skill._active_lookout == ("bathtub",)


def test_lookout_queries_detector_when_enough_of_frame_is_visible(
    perceive_loop_skill: tuple[DimSimPerceiveLoopSkill, Any],
) -> None:
    skill, model = perceive_loop_skill
    data = np.zeros((100, 100, 3), dtype=np.uint8)
    data[60:, :, :] = 128
    image = Image(data=data)
    model.query_detections.return_value = ImageDetections2D(image)
    skill._active_lookout = ("bathtub",)

    skill._on_image(image)

    model.query_detections.assert_called_once_with(image, "bathtub")
    assert skill._active_lookout == ("bathtub",)
