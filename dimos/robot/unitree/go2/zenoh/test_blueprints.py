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

"""The camera pane has to follow the encoder: jpeg lands on `image`, h264 on `video`."""

from typing import Any

from dimos.robot.unitree.go2.zenoh.blueprints import (
    _dds_camera,
    go2_dds_motion_pointlio,
    go2_viewer,
    go2_zenoh_basic,
)


def _rerun_kwargs(blueprint: Any) -> dict[str, Any]:
    (atom,) = (a for a in blueprint.active_blueprints if a.name == "rerunbridgemodule")
    return atom.kwargs


def _camera_entity(blueprint: Any) -> str:
    kwargs = _rerun_kwargs(blueprint)
    (pane,) = kwargs["blueprint"].args
    # the pinhole must land on the same entity or the frustum draws empty
    assert kwargs["visual_override"]["world/camera_info"].keywords["camera"] == pane
    return str(pane)


def test_camera_entity_follows_the_encoder() -> None:
    assert _dds_camera({"GO2DDS__VIDEO_ENCODING": "jpeg"}) == "world/image"
    assert _dds_camera({"go2dds__video_encoding": "jpeg"}) == "world/image"
    assert _dds_camera({}) == "world/video"


def test_dds_pointlio_pins_no_encoding() -> None:
    # h264 is the default; the robot's .env is what flips it to jpeg
    for atom in go2_dds_motion_pointlio.active_blueprints:
        assert "video_encoding" not in atom.kwargs
        assert "video_fps" not in atom.kwargs
    assert _camera_entity(go2_dds_motion_pointlio) == _dds_camera()


def test_viewer_subscribes_both_encodings() -> None:
    assert _camera_entity(go2_viewer) == _dds_camera()
    topics = _rerun_kwargs(go2_viewer)["topics"]
    assert {"video", "image", "camera_info"} <= set(topics)


def test_h264_stacks_keep_the_video_pane() -> None:
    assert _camera_entity(go2_zenoh_basic) == "world/video"
