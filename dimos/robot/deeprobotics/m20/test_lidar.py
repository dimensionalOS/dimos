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

"""The M20 lidar blueprint: two Airys on separate streams and frames, mounted on base_link."""

import math

import numpy as np

from dimos.hardware.sensors.lidar.robosense.module import Airy
from dimos.robot.deeprobotics.m20.constants import FRONT_LIDAR_XYZ_M, REAR_LIDAR_XYZ_M
from dimos.robot.deeprobotics.m20.lidar import (
    FRONT_AIRY,
    REAR_AIRY,
    M20LidarMountTf,
    m20_airy_lidars,
)


def _atoms() -> dict:
    return {atom.name: atom for atom in m20_airy_lidars(voxel=0.1).solidify().blueprints}


def test_two_airys_on_separate_streams() -> None:
    blueprint = m20_airy_lidars().solidify()
    atoms = _atoms()
    assert atoms[FRONT_AIRY].module is Airy and atoms[REAR_AIRY].module is Airy
    remap = blueprint.remapping_map
    assert remap[(FRONT_AIRY, "lidar")] == "front_lidar"
    assert remap[(REAR_AIRY, "lidar")] == "rear_lidar"
    assert remap[(FRONT_AIRY, "imu")] == "front_lidar_imu"
    assert remap[(REAR_AIRY, "imu")] == "rear_lidar_imu"


def test_each_airy_has_its_own_lidar_and_frame() -> None:
    front, rear = _atoms()[FRONT_AIRY].kwargs, _atoms()[REAR_AIRY].kwargs
    assert (front["group"], front["msop_port"], front["difop_port"], front["imu_port"]) == (
        "224.10.10.201",
        6691,
        7781,
        6681,
    )
    assert (rear["group"], rear["msop_port"], rear["difop_port"], rear["imu_port"]) == (
        "224.10.10.202",
        6692,
        7782,
        6682,
    )
    assert front["frame_id"] == front["imu_frame_id"] == "front_lidar"
    assert rear["frame_id"] == rear["imu_frame_id"] == "rear_lidar"
    assert front["voxel"] == rear["voxel"] == 0.1


def _matrix(transform) -> np.ndarray:
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def test_mount_tf() -> None:
    by_child = {t.child_frame_id: t for t in M20LidarMountTf.transforms(None)}  # type: ignore[arg-type]
    assert set(by_child) == {"front_lidar", "rear_lidar"}
    for child, xyz, outward in [
        ("front_lidar", FRONT_LIDAR_XYZ_M, 1.0),
        ("rear_lidar", REAR_LIDAR_XYZ_M, -1.0),
    ]:
        transform = by_child[child]
        assert transform.frame_id == "base_link"
        assert (transform.translation.x, transform.translation.y, transform.translation.z) == xyz
        rotation = _matrix(transform)
        # lidar x is up (the IMU's gravity reads +x), its spin axis z looks out its end of the body
        assert np.allclose(rotation @ [1, 0, 0], [0, 0, 1], atol=1e-9)
        assert np.allclose(rotation @ [0, 0, 1], [outward, 0, 0], atol=1e-9)
        assert math.isclose(np.linalg.det(rotation), 1.0)
