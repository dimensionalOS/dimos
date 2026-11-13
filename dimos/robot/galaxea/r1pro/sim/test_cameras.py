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
from pathlib import Path

import cv2
import numpy as np
import pytest

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.galaxea.r1pro.collection import CAMERAS
from dimos.robot.galaxea.r1pro.sim.cameras import head_camera, wrist_camera
from dimos.robot.galaxea.r1pro.stereo import HeadStereo

NO_CALIBRATION = Path("/nonexistent/stereo.yaml")


def test_cameras_have_the_sizes_and_rate_the_robot_records():
    views = {
        "head": head_camera(calibration=NO_CALIBRATION),
        "left_wrist": wrist_camera("left"),
        "right_wrist": wrist_camera("right"),
    }
    for name, camera in views.items():
        # The collection profile states (height, width, channels) per camera.
        assert (camera.height, camera.width, 3) == CAMERAS[name][1]
        assert camera.hz == 5


def test_vendor_fov_stands_in_without_the_factory_calibration():
    camera = head_camera(calibration=NO_CALIBRATION)
    assert camera.width / camera.height == 1920 / 1536
    assert math.degrees(2 * math.atan(camera.width / (2 * camera.fx))) == pytest.approx(118)
    assert math.degrees(2 * math.atan(camera.height / (2 * camera.fy))) == pytest.approx(92)


def test_rectified_factory_intrinsics_scale_with_image():
    calibration = CameraInfo.from_intrinsics(900, 950, 960, 768, 1920, 1536)
    camera = head_camera(calibration)
    assert (camera.width, camera.height) == (480, 384)
    assert (camera.fx, camera.fy, camera.cx, camera.cy) == pytest.approx((225, 237.5, 240, 192))
    calibration.D = [0.1, 0, 0, 0, 0]
    with pytest.raises(ValueError, match="rectified"):
        head_camera(calibration)


def test_factory_calibration_file_gives_the_intrinsics_the_robot_publishes(tmp_path):
    path = tmp_path / "stereo.yaml"
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    storage.write("image_width", 1920)
    storage.write("image_height", 1536)
    for side, cx in (("left", 955.0), ("right", 970.0)):
        storage.write(f"K_{side}", np.array([[980.0, 0, cx], [0, 985.0, 760.0], [0, 0, 1]]))
        storage.write(f"D_{side}", np.array([[-0.05, 0.01, 0.0, 0.0, 0.0]]))
    storage.write("R", np.eye(3))
    storage.write("T", np.array([[-0.06], [0.0], [0.0]]))
    storage.release()
    published = HeadStereo(path, 480).info
    camera = head_camera(calibration=path)
    assert (camera.width, camera.height) == (published.width, published.height) == (480, 384)
    assert (camera.fx, camera.fy, camera.cx, camera.cy) == pytest.approx(
        (published.K[0], published.K[4], published.K[2], published.K[5])
    )


@pytest.mark.parametrize("side", ["left", "right"])
def test_wrist_fov_matches_depth_option_at_full_resolution(side):
    camera = wrist_camera(side)
    assert (camera.width, camera.height) == (1280, 720)
    assert camera.frame_id == f"r1pro/{side}_D405_link"
    assert math.degrees(2 * math.atan(camera.width / (2 * camera.fx))) == pytest.approx(87)
    assert math.degrees(2 * math.atan(camera.height / (2 * camera.fy))) == pytest.approx(58)
