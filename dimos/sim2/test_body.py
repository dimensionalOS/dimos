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


def _read(
    spin: tuple[float, ...] = (0.0, 0.0, 0.0),
    imu_yaw: float = 0.0,
    orientation: tuple[float, ...] = (0.0, 0.0, 0.0, 1.0),
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    imu_in_body = np.eye(4)
    imu_in_body[:3, :3] = Rotation.from_euler("z", imu_yaw).as_matrix()
    imu_in_body[:3, 3] = IMU_XYZ
    pose, velocity = (
        np.array([1.0, 2.0, 0.8, *orientation]),
        np.array([0.0, 0.0, 0.0, *spin]),
    )
    return imu_reading(pose, velocity, np.zeros(6), imu_in_body)


def test_an_imu_reads_its_body_s_motion_in_its_own_axes() -> None:
    gyro, accelerometer, orientation = _read()
    assert accelerometer == pytest.approx([0.0, 0.0, 9.81]), "at rest it reads gravity up"
    assert gyro == pytest.approx([0.0, 0.0, 0.0])
    assert orientation == pytest.approx([0.0, 0.0, 0.0, 1.0])
    # Yawed a quarter turn on the body, the world's x spin is about the IMU's -y.
    assert _read(spin=(0.3, 0.0, 0.0), imu_yaw=IMU_YAW)[0] == pytest.approx([0.0, -0.3, 0.0])
    # Centripetal: rate^2 * radius toward the axis, from an IMU IMU_XYZ[0] out along x.
    rate = 2.0
    assert _read(spin=(0.0, 0.0, rate))[1] == pytest.approx([-(rate**2) * IMU_XYZ[0], 0.0, 9.81])

    # Tilted, the IMU is turned as its root is and feels gravity tilted.
    tilt = Rotation.from_euler("xyz", (0.3, -0.2, 1.1))
    gyro, accelerometer, orientation = _read(imu_yaw=IMU_YAW, orientation=tuple(tilt.as_quat()))
    expected = tilt * Rotation.from_euler("z", IMU_YAW)
    assert Rotation.from_quat(orientation).approx_equal(expected, atol=1e-9)
    assert accelerometer == pytest.approx(expected.inv().apply([0.0, 0.0, 9.81]))
    assert gyro == pytest.approx([0.0, 0.0, 0.0])


def test_a_site_carries_isaac_s_site_api_and_one_that_is_no_prim_is_refused() -> None:
    pytest.importorskip("pxr")
    from dimos.robot.assets.model import UsdModel
    from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr
    from dimos.sim2.body import is_site, with_sites
    from dimos.sim2.sensor_usd import frame_usd

    frame = frame_usd("base_link", name="robot")
    framed = frame.with_frames({"lidar": (frame.root_link, (0.1, 0.0, 0.2), (0.0, 0.0, 0.0))})
    sited = with_sites(
        UsdModel.from_layer(framed.usd),
        {f"{frame.root_link}/lidar": ("lidar mount", "X")},
    )
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, sited.usd, "a robot", None))
    site = stage.GetPrimAtPath(f"/{frame.root_link}/lidar")
    assert is_site(site)
    assert site.GetAttribute("isaac:description").Get() == "lidar mount"
    assert site.GetAttribute("isaac:forwardAxis").Get() == "X"
    # Still a frame a mount can hold.
    assert f"{frame.root_link}/lidar" in sited.tf_frames
    with pytest.raises(KeyError, match="no prim"):
        with_sites(frame, {"robot/nowhere": ("x", "X")})
