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

"""UrdfManagerPointClearing on the M20: the body cut out of both lidars' scans (interleaved on one stream), each
scan keeping its own frame, at the standing pose M20StandingJointState publishes."""

from collections.abc import Iterator
from types import SimpleNamespace
from typing import Any, cast

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.tf import TF
from dimos.robot.deeprobotics.m20.lidar import M20LidarConfig
from dimos.robot.deeprobotics.m20.model import M20_MODEL, STAND_JOINT_POSITIONS
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing

# Points in base_link on the standing body (trunk centre, trunk front and back) and well clear of it.
_BODY = [[0.0, 0.0, 0.0], [0.2, 0.0, 0.0], [-0.2, 0.0, 0.05]]
_CLEAR = [[3.0, 0.0, 0.0], [0.0, 3.0, 0.0], [-3.0, 0.0, 1.0]]


@pytest.fixture
def body_filter() -> Iterator[tuple[UrdfManagerPointClearing, dict[str, np.ndarray]]]:
    module = UrdfManagerPointClearing(model=M20_MODEL)
    cast("dict[str, Any]", module.__dict__)["_tf"] = TF()
    mounts = {}
    for which in ("front", "rear"):
        frame = f"{which}_lidar"
        config = M20LidarConfig(
            lidar=which, frame_id=frame, imu_frame_id=frame, mount_parent_frame_id="base_link"
        )
        mount = Transform(
            translation=Vector3(*config.mount_xyz),
            rotation=Quaternion.from_euler(Vector3(*config.mount_rpy)),
            frame_id="base_link",
            child_frame_id=frame,
            ts=1.0,
        )
        module.tfbuffer.receive_transform(mount)
        mounts[frame] = mount.to_matrix()
    module.add_joint_state(
        JointState(
            ts=1.0,
            name=list(STAND_JOINT_POSITIONS),
            position=list(STAND_JOINT_POSITIONS.values()),
        )
    )
    yield module, mounts
    cast("dict[str, Any]", module.__dict__)["_tf"] = None
    module.dispose()


def _scan(
    points_in_base: list[list[float]], frame: str, base_from_lidar: np.ndarray
) -> PointCloud2:
    lidar_from_base = np.linalg.inv(base_from_lidar)
    points = np.asarray(points_in_base, dtype=np.float64)
    local = points @ lidar_from_base[:3, :3].T + lidar_from_base[:3, 3]
    return PointCloud2.from_numpy(local.astype(np.float32), frame_id=frame, timestamp=1.0)


def test_both_lidars_lose_the_body_and_keep_their_frame(body_filter) -> None:
    module, mounts = body_filter
    for frame in ("front_lidar", "rear_lidar", "front_lidar"):
        scan = _scan(_BODY + _CLEAR, frame, mounts[frame])
        filtered = module.filter_cloud(scan)
        assert filtered is not None
        assert (filtered.frame_id, filtered.ts) == (frame, scan.ts)
        # what is left is exactly the clear points, still in the lidar's own frame
        np.testing.assert_allclose(
            filtered.points_f32(), scan.points_f32()[len(_BODY) :], atol=1e-6
        )


def test_interleaved_scans_come_out_one_per_scan_in_order_with_their_stamp_and_frame(
    body_filter,
) -> None:
    """Both lidars share one stream: every scan in gives exactly one scan out, its stamp, frame and points kept."""
    module, mounts = body_filter
    published: list[PointCloud2] = []
    cast("dict[str, Any]", module.__dict__)["filtered_pointcloud"] = SimpleNamespace(
        publish=published.append
    )
    scans = []
    for i in range(6):
        frame = ("front_lidar", "rear_lidar")[i % 2]
        # overlapping scans, as the two Airys' are: each starts 30 ms after the other
        scan = _scan(_CLEAR, frame, mounts[frame])
        scan.ts = 1.0 + 0.03 * i
        # each lidar publishes its mount stamped like its scan; the joint state is republished alongside
        module.tfbuffer.receive_transform(
            Transform.from_matrix(
                mounts[frame], frame_id="base_link", child_frame_id=frame, ts=scan.ts
            )
        )
        module.add_joint_state(
            JointState(
                ts=scan.ts,
                name=list(STAND_JOINT_POSITIONS),
                position=list(STAND_JOINT_POSITIONS.values()),
            )
        )
        scans.append(scan)
        module._on_pointcloud(scan)
    assert [(p.frame_id, p.ts) for p in published] == [(s.frame_id, s.ts) for s in scans]
    for scan, out in zip(scans, published, strict=True):
        np.testing.assert_array_equal(out.points_f32(), scan.points_f32())
