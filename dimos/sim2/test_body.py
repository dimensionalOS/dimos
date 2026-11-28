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

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.sim2.body import imu_reading

# An IMU off the body's origin and turned on it, so the lever arm and the mounting both show.
IMU_XYZ = (0.1, 0.0, -0.1)
IMU_YAW = math.pi / 2


def _imu_in_body(yaw: float = 0.0) -> np.ndarray:
    imu = np.eye(4)
    imu[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    imu[:3, 3] = IMU_XYZ
    return imu


def test_a_body_at_rest_reads_gravity_up_and_no_spin() -> None:
    gyro, accelerometer, orientation = imu_reading(
        np.array([1.0, 2.0, 0.8, 0.0, 0.0, 0.0, 1.0]), np.zeros(6), np.zeros(6), _imu_in_body()
    )

    assert accelerometer == pytest.approx([0.0, 0.0, 9.81])
    assert gyro == pytest.approx([0.0, 0.0, 0.0])
    assert orientation == pytest.approx([0.0, 0.0, 0.0, 1.0])


def test_a_spinning_body_s_gyro_reads_the_spin_in_the_imu_s_axes() -> None:
    spin = np.array([0.0, 0.0, 0.0, 0.3, 0.0, 0.0])
    # The IMU is yawed a quarter turn on the body, so the world's x spin is about its -y.
    gyro, _, _ = imu_reading(
        np.array([0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0]), spin, np.zeros(6), _imu_in_body(IMU_YAW)
    )

    assert gyro == pytest.approx([0.0, -0.3, 0.0])


def test_a_spinning_body_s_accelerometer_feels_the_pull_toward_the_axis() -> None:
    rate = 2.0
    spin = np.array([0.0, 0.0, 0.0, 0.0, 0.0, rate])
    _, accelerometer, _ = imu_reading(
        np.array([0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0]), spin, np.zeros(6), _imu_in_body()
    )

    # Centripetal: rate^2 * radius toward the axis, from an IMU IMU_XYZ[0] out along x.
    assert accelerometer == pytest.approx([-(rate**2) * IMU_XYZ[0], 0.0, 9.81])


def test_a_tilted_body_s_imu_is_turned_as_its_root_is_and_feels_gravity_tilted() -> None:
    tilt = Rotation.from_euler("xyz", (0.3, -0.2, 1.1))
    gyro, accelerometer, orientation = imu_reading(
        np.array([0.0, 0.0, 0.8, *tilt.as_quat()]), np.zeros(6), np.zeros(6), _imu_in_body(IMU_YAW)
    )

    expected = tilt * Rotation.from_euler("z", IMU_YAW)
    assert Rotation.from_quat(orientation).approx_equal(expected, atol=1e-9)
    assert accelerometer == pytest.approx(expected.inv().apply([0.0, 0.0, 9.81]))
    assert gyro == pytest.approx([0.0, 0.0, 0.0])


def test_a_site_carries_isaac_s_site_api_its_description_and_forward_axis() -> None:
    pytest.importorskip("pxr")
    from dimos.robot.assets.model import UsdModel
    from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr
    from dimos.sim2.body import is_site, with_sites
    from dimos.sim2.sensor_usd import frame_usd

    frame = frame_usd("base_link", name="robot")
    framed = frame.with_frames({"lidar": (frame.root_link, (0.1, 0.0, 0.2), (0.0, 0.0, 0.0))})
    sited = with_sites(
        UsdModel.from_layer(framed.usd), {f"{frame.root_link}/lidar": ("lidar mount", "X")}
    )

    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, sited.usd, "a robot", None))
    site = stage.GetPrimAtPath(f"/{frame.root_link}/lidar")
    assert is_site(site)
    assert site.GetAttribute("isaac:description").Get() == "lidar mount"
    assert site.GetAttribute("isaac:forwardAxis").Get() == "X"
    # Still a frame a mount can hold.
    assert f"{frame.root_link}/lidar" in sited.tf_frames


def test_a_site_that_is_no_prim_of_the_usd_is_refused() -> None:
    pytest.importorskip("pxr")
    from dimos.sim2.body import with_sites
    from dimos.sim2.sensor_usd import frame_usd

    with pytest.raises(KeyError, match="no prim"):
        with_sites(frame_usd("base_link", name="robot"), {"robot/nowhere": ("x", "X")})
