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

from pathlib import Path

from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control
from dimos.robot.galaxea.r1pro.connection import R1ProConnection
from dimos.robot.galaxea.r1pro.head_cameras import (
    HEAD_LEFT_V4L2,
    HEAD_RIGHT_V4L2,
    HeadLeftCameraConfig,
    HeadRightCameraConfig,
    head_camera_infos,
)

# The ISX031's only real mode: width, height (px), pixel format.
_SENSOR_MODE = (1920, 1536, "UYVY")


def test_each_eye_defaults_to_its_own_node_at_the_sensor_mode() -> None:
    left, right = HeadLeftCameraConfig(), HeadRightCameraConfig()

    assert (left.device, left.frame_id) == (HEAD_LEFT_V4L2, "camera_head_left_link")
    assert (right.device, right.frame_id) == (HEAD_RIGHT_V4L2, "camera_head_right_link")
    for eye in (left, right):
        assert (eye.width, eye.height, eye.fourcc) == _SENSOR_MODE


def test_head_colour_is_jpeg_under_the_old_names() -> None:
    blueprint = r1pro_control()

    assert {"head_left_color", "head_right_color"} <= set(blueprint.remapping_map.values())
    assert ("head_left_color", CompressedImage) in blueprint.transport_map
    assert ("head_right_color", CompressedImage) in blueprint.transport_map


def test_connection_no_longer_publishes_head_colour() -> None:
    assert not hasattr(R1ProConnection, "head_left_color")
    assert not hasattr(R1ProConnection, "head_right_color")


_CALIBRATION = """%YAML:1.0
---
image_width: 1920
image_height: 1536
K_left: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1012.5, 0., 962.2, 0., 1012.1, 765.6, 0., 0., 1. ]
D_left: !!opencv-matrix
   rows: 1
   cols: 14
   dt: d
   data: [ -0.68, -0.64, 0.0002, -0.0002, -0.03, -0.29, -1.0, -0.18, 0., 0., 0., 0., 0., 0. ]
K_right: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1013.8, 0., 958.7, 0., 1013.4, 768.2, 0., 0., 1. ]
D_right: !!opencv-matrix
   rows: 1
   cols: 14
   dt: d
   data: [ -0.25, -0.43, 0.0001, 0.0, -0.02, 0.14, -0.62, -0.12, 0., 0., 0., 0., 0., 0. ]
"""


def test_each_eye_gets_its_own_intrinsics_from_the_factory_calibration(tmp_path: Path) -> None:
    path = tmp_path / "stereo.yaml"
    path.write_text(_CALIBRATION)

    left, right = head_camera_infos(str(path))

    assert (left.width, left.height, left.frame_id) == (1920, 1536, "camera_head_left_link")
    assert right.frame_id == "camera_head_right_link"
    assert left.K[0] == 1012.5 and right.K[2] == 958.7
    # Eight rational-polynomial coefficients; the six zero tilt terms are dropped.
    assert left.distortion_model == "rational_polynomial" and len(left.D) == 8
    assert left.P[:3] == left.K[:3] and left.P[3] == 0.0


def test_r1pro_control_publishes_both_eyes_intrinsics() -> None:
    blueprint = r1pro_control()

    assert {"head_left_info", "head_right_info"} <= set(blueprint.remapping_map.values())
