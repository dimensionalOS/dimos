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

"""Unit tests for the stream-level `pose_fill` (no LFS data, runs in normal CI).

The end-to-end `dimos map pose-fill` path against a real recording lives in
`test_cli.py` (self-hosted). These cover the pure stream transform directly.
"""

from __future__ import annotations

from copy import deepcopy
import math

from dimos_cdrgen.registry import encode
import numpy as np

from dimos.mapping.cli.pose_fill import pose_fill, pose_fill_db
from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.message_utils.geometry import quaternion_from_euler
from dimos.message_utils.pointcloud import pointcloud_from_xyz
from dimos.message_utils.time import time_from_nanoseconds
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    Vector3,
)
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.messages.std_msgs.msg import Header


def test_pose_fill_attaches_nearest_pose() -> None:
    """Each target obs gets the nearest pose-source pose; payload is preserved."""
    with MemoryStore() as store:
        target = store.stream("image", str)
        poses = store.stream("odom", Transform)
        target.append("img0", ts=0.0)
        poses.append(
            Transform(
                translation=Vector3(x=1.0, y=0.0, z=0.0), rotation=Quaternion(x=0, y=0, z=0, w=1)
            ),
            ts=0.001,
        )

        out = pose_fill(target, poses, tolerance=0.05).to_list()

    assert len(out) == 1
    assert out[0].data == "img0"
    assert out[0].pose_tuple is not None
    assert out[0].pose_tuple[:3] == (1.0, 0.0, 0.0)


def test_pose_fill_mount_composes_static_child_transform() -> None:
    """`mount` composes ``world_base + mount`` onto each attached pose."""
    with MemoryStore() as store:
        target = store.stream("image", str)
        poses = store.stream("odom", Transform)
        target.append("img0", ts=0.0)
        # Base pose 1m forward in x; mount offsets 1m in y (identity rotations).
        poses.append(
            Transform(
                translation=Vector3(x=1.0, y=0.0, z=0.0), rotation=Quaternion(x=0, y=0, z=0, w=1)
            ),
            ts=0.001,
        )
        mount = Transform(
            translation=Vector3(x=0, y=1, z=0), rotation=Quaternion(x=0, y=0, z=0, w=1)
        )

        out = pose_fill(target, poses, tolerance=0.05, mount=mount).to_list()

    assert len(out) == 1
    assert out[0].pose_tuple is not None
    # Identity rotations → composed translation is the component-wise sum.
    assert out[0].pose_tuple[:3] == (1.0, 1.0, 0.0)


def test_stamped_pose_mount_rotates_the_offset_and_keeps_payload():
    header = Header(frame_id="world", stamp=time_from_nanoseconds(1700000000123456789))
    pose = PoseStamped(
        header=header,
        pose=Pose(
            position=Point(x=1, y=2, z=3), orientation=quaternion_from_euler(0, 0, math.pi / 2)
        ),
    )
    mount = Transform(translation=Vector3(x=1, y=0, z=0), rotation=Quaternion(x=0, y=0, z=0, w=1))
    with MemoryStore() as store:
        target = store.stream("target", str)
        source = store.stream("pose", PoseStamped)
        target.append("frame", ts=10.0)
        source.append(pose, ts=10.01)
        before = encode(pose)
        [result] = pose_fill(target, source, mount=mount).to_list()
        assert result.data == "frame"
        np.testing.assert_allclose(result.pose_tuple[:3], [1, 3, 3], atol=1e-12)
        assert encode(pose) == before


def test_sqlite_pose_fill_copies_original_cdr_bytes_and_exact_header(tmp_path):
    source_path = tmp_path / "source.db"
    destination = tmp_path / "posed.db"
    header = Header(frame_id="lidar", stamp=time_from_nanoseconds(1700000000123456789))
    cloud = pointcloud_from_xyz(np.array([[1, 2, 3]], dtype=np.float32), header=header)
    pose_header = deepcopy(header)
    pose_header.frame_id = "odom"
    pose = PoseStamped(
        header=pose_header,
        pose=Pose(position=Point(x=4, y=5, z=6), orientation=Quaternion(x=0, y=0, z=0, w=1)),
    )
    with SqliteStore(path=str(source_path)) as store:
        store.stream("lidar", PointCloud2).append(cloud, ts=10.0)
        store.stream("odom", PoseStamped).append(pose, ts=10.001)
    written = pose_fill_db(source_path, destination, target="lidar", pose_source="odom")
    assert written == {"lidar": 1, "odom": 1}
    with (
        SqliteStore(path=str(source_path), must_exist=True) as original,
        SqliteStore(path=str(destination), must_exist=True) as store,
    ):
        result = store.stream("lidar", PointCloud2).first()
        assert result.pose_tuple == (4.0, 5.0, 6.0, 0.0, 0.0, 0.0, 1.0)
        assert result.ts == 10.0
        assert encode(result.data) == encode(cloud)
        assert result.data.header.stamp.nanosec == 123456789
        assert encode(store.stream("odom", PoseStamped).first().data) == encode(pose)
        before = original.stream("lidar", PointCloud2)
        after = store.stream("lidar", PointCloud2)
        assert before._source.blob_store.get(
            "lidar", before.first().id
        ) == after._source.blob_store.get("lidar", result.id)
