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

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation
import yourdfpy  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.lio_base_pose import (
    MID360_MOUNT_CORRECTION,
    lio_odometry_to_base_pose,
    pelvis_to_sensor_from_urdf,
)
from dimos.utils.transform_utils import pose_to_matrix


def test_lio_pose_recovers_pelvis_after_upside_down_mount_correction() -> None:
    pelvis_to_sensor = np.eye(4)
    pelvis_to_sensor[:3, :3] = Rotation.from_euler("y", 0.04).as_matrix()
    pelvis_to_sensor[:3, 3] = [0.0, 0.0, 0.42]

    expected_world_to_pelvis = np.eye(4)
    expected_world_to_pelvis[:3, :3] = Rotation.from_euler("xyz", [0.02, -0.03, 0.8]).as_matrix()
    expected_world_to_pelvis[:3, 3] = [1.2, -0.7, 0.74]
    modeled_world_to_sensor = expected_world_to_pelvis @ pelvis_to_sensor

    # Point-LIO reports the physical, upside-down IMU body. Applying the
    # correction in the production helper must recover the modeled link.
    raw_rotation = modeled_world_to_sensor[:3, :3] @ MID360_MOUNT_CORRECTION.T
    odometry = Odometry(
        ts=12.5,
        frame_id="world",
        child_frame_id="mid360_link",
        pose=Pose(
            position=modeled_world_to_sensor[:3, 3],
            orientation=Quaternion.from_rotation_matrix(raw_rotation),
        ),
    )

    pose = lio_odometry_to_base_pose(odometry, pelvis_to_sensor)

    assert pose.ts == 12.5
    assert pose.frame_id == "world"
    np.testing.assert_allclose(pose_to_matrix(pose), expected_world_to_pelvis, atol=1e-9)


def test_live_waist_state_changes_the_urdf_sensor_mount() -> None:
    urdf = yourdfpy.URDF.load(str(G1.model_path), load_meshes=False)
    rest = pelvis_to_sensor_from_urdf(
        urdf,
        {"waist_yaw": 0.0, "waist_roll": 0.0, "waist_pitch": 0.0},
    )
    bent = pelvis_to_sensor_from_urdf(
        urdf,
        {"waist_yaw": 0.2, "waist_roll": -0.1, "waist_pitch": 0.15},
    )

    assert rest.shape == (4, 4)
    assert rest[2, 3] > 0.4
    assert not np.allclose(rest, bent)
