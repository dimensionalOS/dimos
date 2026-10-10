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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

import math
from types import SimpleNamespace

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from dimos_generated.sensor_msgs.msg import Imu
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np

from dimos.memory.type.observation import Observation
from dimos.msgs.geometry import transform_matrix
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz, transform_cloud
from dimos.msgs.time import time_from_seconds
from dimos.robot.unitree.go2.dds.cli.render import (
    accumulate_path,
    integrate_position,
    integrate_velocity,
    sportmode_pose,
    world_accel,
)
from dimos.robot.unitree.go2.dds.extrinsics import EXT_R, EXT_T, LIDAR_TO_BASE


def test_world_acceleration_and_integrators_keep_world_axes_and_clock():
    imu = Imu(
        orientation=Quaternion(z=math.sqrt(0.5), w=math.sqrt(0.5), x=0.0, y=0.0),
        linear_acceleration=Vector3(x=1, z=9.81, y=0.0),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        orientation_covariance=np.zeros(9, dtype=np.float64),
        angular_velocity=Vector3(x=0.0, y=0.0, z=0.0),
        angular_velocity_covariance=np.zeros(9, dtype=np.float64),
        linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
    )
    observations = [Observation(ts=ts, _data=imu) for ts in (2.0, 3.0)]
    np.testing.assert_allclose(world_accel(observations[0]), [0, 1, 9.81], atol=1e-12)
    velocity_step = integrate_velocity(np.array([0.0, 0.0, 9.81]))
    state = (np.zeros(3), None)
    twists = []
    for obs in observations:
        state, twist = velocity_step(state, obs)
        twists.append(Observation(ts=obs.ts, _data=twist))
        assert twist.header.stamp == time_from_seconds(obs.ts)
        assert twist.header.frame_id == "world"
    position_state = (np.zeros(3), None)
    for obs in twists:
        position_state, pose = integrate_position(position_state, obs)
    np.testing.assert_allclose(
        [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z], [0, 1, 0], atol=1e-12
    )
    assert pose.header.stamp == time_from_seconds(3.0)


def test_path_accumulation_keeps_previous_snapshot_and_exact_pose_headers():
    poses = [
        PoseStamped(
            header=Header(frame_id="world", stamp=time_from_seconds(ts)),
            pose=Pose(
                position=Point(x=ts, y=0.0, z=0.0), orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
            ),
        )
        for ts in (1.0, 2.0)
    ]
    result = list(
        accumulate_path(
            iter([Observation(ts=ts, _data=p) for ts, p in zip((1.0, 2.0), poses, strict=True)])
        )
    )
    assert [len(obs.data.poses) for obs in result] == [1, 2]
    assert result[1].data.header == poses[1].header
    assert cdr_encode(result[0].data.poses[0]) == cdr_encode(poses[0])


def test_sportmode_wxyz_is_converted_to_generated_xyzw():
    sm = SimpleNamespace(
        position=[1.0, 2.0, 3.0], imu_state=SimpleNamespace(quaternion=[0.5, -0.5, 0.5, -0.5])
    )
    pose = sportmode_pose(Observation(ts=4.25, _data=sm))
    assert pose.header.stamp == time_from_seconds(4.25)
    q = pose.pose.orientation
    assert (q.x, q.y, q.z, q.w) == (-0.5, 0.5, -0.5, 0.5)
    assert cdr_decode(cdr_encode(pose), PoseStamped).pose.position.y == 2.0


def test_measured_lidar_mount_retains_coordinates_and_cloud_stamp():
    cloud = pointcloud_from_xyz(
        np.array([[1.0, 0.0, 0.0]]), header=Header(frame_id="lidar", stamp=time_from_seconds(12.25))
    )
    original = cdr_encode(cloud)
    transformed = transform_cloud(cloud, LIDAR_TO_BASE)
    matrix = transform_matrix(LIDAR_TO_BASE.transform)
    np.testing.assert_allclose(matrix[:3, :3], EXT_R, atol=1e-6)
    np.testing.assert_allclose(
        pointcloud_xyz(transformed), (matrix[:3, :3] @ [1.0, 0.0, 0.0] + EXT_T)[None, :], atol=1e-6
    )
    assert transformed.header.stamp == cloud.header.stamp
    assert transformed.header.frame_id == "base_link"
    assert cdr_encode(cloud) == original
