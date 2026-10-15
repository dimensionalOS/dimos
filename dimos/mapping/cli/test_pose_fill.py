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

import math

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.std_msgs.msg import Header
import pytest

from dimos.mapping.cli.pose_fill import pose_fill
from dimos.memory.store.memory import MemoryStore


def test_pose_fill_attaches_nearest_pose() -> None:
    """Each target obs gets the nearest pose-source pose; payload is preserved."""
    with MemoryStore() as store:
        target = store.stream("image", str)
        poses = store.stream("odom", Pose)
        target.append("img0", ts=0.0)
        poses.append(
            Pose(
                position=Point(x=1.0, y=0.0, z=0.0),
                orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
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
        poses = store.stream("odom", Pose)
        target.append("img0", ts=0.0)
        # Base pose 1m forward in x; mount offsets 1m in y (identity rotations).
        poses.append(
            Pose(
                position=Point(x=1.0, y=0.0, z=0.0),
                orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
            ),
            ts=0.001,
        )
        mount = TransformStamped(
            transform=Transform(
                translation=Vector3(y=1, x=0.0, z=0.0),
                rotation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
            ),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            child_frame_id="",
        )

        out = pose_fill(target, poses, tolerance=0.05, mount=mount).to_list()

    assert len(out) == 1
    assert out[0].pose_tuple is not None
    # Identity rotations → composed translation is the component-wise sum.
    assert out[0].pose_tuple[:3] == (1.0, 1.0, 0.0)


def test_pose_fill_rotates_mount_offset_in_base_axes() -> None:
    """A base yaw rotates the local camera offset before the world translation."""
    with MemoryStore() as store:
        target = store.stream("image", str)
        poses = store.stream("odom", Pose)
        target.append("camera frame", ts=5.0)
        poses.append(
            Pose(
                position=Point(x=2, y=0.0, z=0.0),
                orientation=Quaternion(z=math.sqrt(0.5), w=math.sqrt(0.5), x=0.0, y=0.0),
            ),
            ts=5.001,
        )
        mount = TransformStamped(
            transform=Transform(
                translation=Vector3(x=1, y=0.0, z=0.0),
                rotation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
            ),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            child_frame_id="",
        )
        result = pose_fill(target, poses, tolerance=0.05, mount=mount).to_list()
    assert len(result) == 1
    assert result[0].data == "camera frame"
    assert result[0].ts == 5.0
    assert result[0].pose_tuple == pytest.approx((2, 1, 0, 0, 0, math.sqrt(0.5), math.sqrt(0.5)))


def test_pose_fill_drops_unmatched_frame_without_loading_payload() -> None:
    with MemoryStore() as store:
        target = store.stream("image", str)
        poses = store.stream("odom", Pose)
        target.append("unmatched", ts=1.0)
        poses.append(
            Pose(
                position=Point(x=1, y=0.0, z=0.0), orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
            ),
            ts=3.0,
        )
        assert pose_fill(target, poses, tolerance=0.05).to_list() == []
