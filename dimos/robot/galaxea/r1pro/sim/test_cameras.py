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

import math

import pytest

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.galaxea.r1pro.sim.cameras import head_camera, wrist_camera


def test_vendor_fov_is_preserved_at_preview_resolution():
    camera = head_camera()
    assert camera.width / camera.height == 1920 / 1536
    assert math.degrees(2 * math.atan(camera.width / (2 * camera.fx))) == pytest.approx(118)
    assert math.degrees(2 * math.atan(camera.height / (2 * camera.fy))) == pytest.approx(92)


def test_rectified_factory_intrinsics_scale_with_image():
    calibration = CameraInfo.from_intrinsics(900, 950, 960, 768, 1920, 1536)
    camera = head_camera(calibration)
    assert (camera.fx, camera.fy, camera.cx, camera.cy) == pytest.approx((300, 950 / 3, 320, 256))
    calibration.D = [0.1, 0, 0, 0, 0]
    with pytest.raises(ValueError, match="rectified"):
        head_camera(calibration)


@pytest.mark.parametrize("side", ["left", "right"])
def test_wrist_fov_matches_depth_option_at_reduced_resolution(side):
    camera = wrist_camera(side)
    assert camera.width / camera.height == 1280 / 720
    assert camera.frame_id == f"r1pro/{side}_d405_link"
    assert math.degrees(2 * math.atan(camera.width / (2 * camera.fx))) == pytest.approx(87)
    assert math.degrees(2 * math.atan(camera.height / (2 * camera.fy))) == pytest.approx(58)
