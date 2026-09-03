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

from types import SimpleNamespace

import numpy as np

from dimos.memory.cli.render import _pair_camera_infos
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image


def _entry(name: str, data: object) -> tuple:  # type: ignore[type-arg]
    return (name, SimpleNamespace(), SimpleNamespace(data=data))


def _info(frame_id: str) -> CameraInfo:
    return CameraInfo.from_intrinsics(
        fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480, frame_id=frame_id
    )


def _image(frame_id: str) -> Image:
    return Image(data=np.zeros((4, 4, 3), dtype=np.uint8), frame_id=frame_id)


def test_pairs_by_frame_id() -> None:
    renderable = [
        _entry("camera_info", _info("camera_optical")),
        _entry("color_image", _image("camera_optical")),
        _entry("lidar", SimpleNamespace()),
    ]
    pinholes, paired = _pair_camera_infos(renderable)
    assert set(pinholes) == {"color_image"}
    info, frame = pinholes["color_image"]
    assert info.frame_id == "camera_optical"
    assert frame == "camera_optical"
    assert paired == {"camera_info"}


def test_falls_back_to_single_pair_on_frame_mismatch() -> None:
    renderable = [
        _entry("camera_info", _info("camera_optical")),
        _entry("color_image", _image("other_frame")),
    ]
    pinholes, paired = _pair_camera_infos(renderable)
    assert set(pinholes) == {"color_image"}
    _, frame = pinholes["color_image"]
    assert frame == "other_frame"
    assert paired == {"camera_info"}


def test_unmatched_info_stays_unpaired() -> None:
    renderable = [
        _entry("front_info", _info("front_optical")),
        _entry("rear_info", _info("rear_optical")),
        _entry("front_image", _image("front_optical")),
    ]
    pinholes, paired = _pair_camera_infos(renderable)
    assert set(pinholes) == {"front_image"}
    assert paired == {"front_info"}


def test_one_info_many_images_matches_all_same_frame() -> None:
    renderable = [
        _entry("camera_info", _info("camera_optical")),
        _entry("color_image", _image("camera_optical")),
        _entry("depth_image", _image("camera_optical")),
    ]
    pinholes, paired = _pair_camera_infos(renderable)
    assert set(pinholes) == {"color_image", "depth_image"}
    assert paired == {"camera_info"}


def test_no_infos() -> None:
    renderable = [_entry("color_image", _image("camera_optical"))]
    pinholes, paired = _pair_camera_infos(renderable)
    assert pinholes == {}
    assert paired == set()
