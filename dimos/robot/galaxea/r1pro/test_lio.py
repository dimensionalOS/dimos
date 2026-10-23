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

import os

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.galaxea.r1pro.head_cameras import HEAD_LEFT_FRAME
from dimos.robot.galaxea.r1pro.lio import (
    R1ProLio,
    R1ProLioConfig,
    base_pose,
    mount_transforms,
    set_affinity,
)
from dimos.robot.galaxea.r1pro.rerun import r1pro_nav_rerun_config, render_cloud


def lidar_odometry() -> Odometry:
    return Odometry(
        ts=3.0,
        frame_id="odom",
        child_frame_id="lidar_pointlio_link",
        pose=Pose(Vector3(1.0, 2.0, 0.5), Quaternion.from_euler(Vector3(0.0, 0.0, 0.7))),
    )


def test_mount_hangs_base_link_under_the_lidar() -> None:
    lidar_to_base, base_to_chassis_lidar = mount_transforms(R1ProLioConfig())
    assert (lidar_to_base.frame_id, lidar_to_base.child_frame_id) == (
        "lidar_pointlio_link",
        "base_link",
    )
    assert (base_to_chassis_lidar.frame_id, base_to_chassis_lidar.child_frame_id) == (
        "base_link",
        "lidar_chassis_left_link",
    )


def test_base_pose_carries_the_lidar_pose_through_the_mount() -> None:
    lidar_to_base = mount_transforms(R1ProLioConfig())[0]
    pose = base_pose(lidar_odometry(), lidar_to_base)
    assert (pose.frame_id, pose.ts) == ("odom", 3.0)
    # The mount is a fixed offset, so the yaw carries over and the base sits that offset away.
    assert pose.orientation.to_euler().z == pytest.approx(0.7, abs=1e-6)
    offset = (pose.position - Vector3(1.0, 2.0, 0.5)).length()
    assert offset == pytest.approx(lidar_to_base.translation.length(), abs=1e-9)


def test_r1prolio_publishes_base_link_pose() -> None:
    lio = R1ProLio()
    try:
        lio._lidar_to_base = lio.transforms()[0]
        published: list[PoseStamped] = []
        lio.pose.publish = published.append  # type: ignore[method-assign,assignment]
        lio._on_odometry(lidar_odometry())
        assert len(published) == 1
        assert published[0].position == base_pose(lidar_odometry(), lio._lidar_to_base).position
    finally:
        lio.stop()


@pytest.mark.skipif(not hasattr(os, "sched_setaffinity"), reason="Linux only")
def test_set_affinity_pins_every_thread() -> None:
    cores = os.sched_getaffinity(0)
    set_affinity(os.getpid(), cores)
    assert os.sched_getaffinity(0) == cores


@pytest.mark.parametrize("frame_id", ["lidar_pointlio_link", HEAD_LEFT_FRAME, "other"])
def test_render_cloud_hangs_each_cloud_off_its_frame(frame_id: str) -> None:
    cloud = PointCloud2.from_numpy(
        np.random.default_rng(0).random((50, 3)).astype(np.float32), frame_id=frame_id
    )
    (path, _points), (transform_path, _transform) = render_cloud(cloud)
    assert path == transform_path == f"world/lidar/{frame_id}"


def test_rerun_config_drops_the_heavy_streams() -> None:
    config = r1pro_nav_rerun_config(voxel_size_m=0.07, wall_clearance_m=0.4, planner_viz_hz=0.0)
    assert config["visual_override"]["world/lidar"] is render_cloud
    assert config["visual_override"]["world/lidar_raw"] is None
