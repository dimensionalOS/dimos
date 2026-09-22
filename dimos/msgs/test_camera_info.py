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

"""Calibration helpers preserve generated ROS fields and reject malformed input."""

import math
from pathlib import Path

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import CameraInfo
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest
import yaml

from dimos.msgs.camera_info import (
    camera_info_from_fov,
    camera_info_from_intrinsics,
    camera_info_from_yaml,
    intrinsic_matrix,
)

_CALIBRATION = Path(__file__).parents[1] / "robot/unitree/go2/front_camera_720.yaml"


def test_real_go2_calibration_survives_cdr_with_exact_header():
    header = Header(frame_id="camera_optical", stamp=Time(sec=1700000000, nanosec=123456789))
    source = yaml.safe_load(_CALIBRATION.read_text())
    message = camera_info_from_yaml(_CALIBRATION, header=header)
    decoded = CameraInfo.decode(message.encode())
    assert decoded.header == header
    assert (decoded.width, decoded.height) == (1280, 720)
    assert decoded.distortion_model == "equidistant"
    for field, key in [
        ("k", "camera_matrix"),
        ("d", "distortion_coefficients"),
        ("r", "rectification_matrix"),
        ("p", "projection_matrix"),
    ]:
        np.testing.assert_array_equal(getattr(decoded, field), source[key]["data"])
    matrix = intrinsic_matrix(decoded)
    assert matrix.shape == (3, 3)
    matrix[0, 0] = 0
    assert decoded.k[0] == source["camera_matrix"]["data"][0]
    decoded.header.frame_id = "changed"
    assert header.frame_id == "camera_optical"


@pytest.mark.parametrize("axis,focal", [("vertical", 240), ("horizontal", 320)])
def test_fov_converts_selected_axis_and_builds_projection(axis, focal):
    message = camera_info_from_fov(90, 640, 480, header=Header(frame_id="optical"), axis=axis)
    np.testing.assert_allclose(message.k, [focal, 0, 320, 0, focal, 240, 0, 0, 1])
    np.testing.assert_allclose(message.p, [focal, 0, 320, 0, 0, focal, 240, 0, 0, 0, 1, 0])
    np.testing.assert_array_equal(message.r, np.eye(3).reshape(-1))
    assert message.header.frame_id == "optical"


@pytest.mark.parametrize("fov", [0, -1, 180, 181, math.nan, math.inf])
def test_fov_rejects_nonphysical_angles(fov):
    with pytest.raises(ValueError, match="Field of view"):
        camera_info_from_fov(fov, 640, 480, header=Header())


@pytest.mark.parametrize("width,height", [(0, 480), (640, -1), (True, 480), (1.5, 480)])
def test_intrinsics_reject_invalid_dimensions(width, height):
    with pytest.raises(ValueError, match="positive integers"):
        camera_info_from_intrinsics(300, 300, 320, 240, width, height, header=Header())


@pytest.mark.parametrize("fx", [0, -1, math.nan, math.inf])
def test_intrinsics_reject_invalid_focal_length(fx):
    with pytest.raises(ValueError, match="positive focal"):
        camera_info_from_intrinsics(fx, 300, 320, 240, 640, 480, header=Header())


@pytest.mark.parametrize("mutation", ["rows", "length", "nonfinite"])
def test_yaml_rejects_invalid_camera_matrix(tmp_path, mutation):
    data = yaml.safe_load(_CALIBRATION.read_text())
    matrix = data["camera_matrix"]
    if mutation == "rows":
        matrix["rows"] = 1
    elif mutation == "length":
        matrix["data"].pop()
    else:
        matrix["data"][0] = math.nan
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="camera_matrix"):
        camera_info_from_yaml(path, header=Header())
