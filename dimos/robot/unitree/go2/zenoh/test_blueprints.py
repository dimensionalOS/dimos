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


def test_dds_pointlio_streams_1hz_jpeg() -> None:
    (dds,) = (a for a in go2_dds_motion_pointlio.active_blueprints if "video_fps" in a.kwargs)
    assert dds.kwargs["video_encoding"] == "jpeg"
    assert dds.kwargs["video_fps"] == 1.0
    assert _camera_entity(go2_dds_motion_pointlio) == "world/image"


def test_viewer_subscribes_both_encodings() -> None:
    assert _camera_entity(go2_viewer) == "world/image"
    topics = _rerun_kwargs(go2_viewer)["topics"]
    assert {"video", "image", "camera_info"} <= set(topics)


def test_h264_stacks_keep_the_video_pane() -> None:
    assert _camera_entity(go2_zenoh_basic) == "world/video"
