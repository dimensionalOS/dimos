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

from pathlib import Path

import pytest

from dimos.mapping.cli.pose_fill import pose_fill, pose_fill_db
from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def test_pose_fill_attaches_nearest_pose() -> None:
    """Each target obs gets the nearest pose-source pose; payload is preserved."""
    with MemoryStore() as store:
        target = store.stream("image", str)
        poses = store.stream("odom", Transform)
        target.append("img0", ts=0.0)
        poses.append(Transform(translation=Vector3(1.0, 0.0, 0.0)), ts=0.001)

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
        poses.append(Transform(translation=Vector3(1.0, 0.0, 0.0)), ts=0.001)
        mount = Transform(translation=Vector3(0.0, 1.0, 0.0), rotation=Quaternion())

        out = pose_fill(target, poses, tolerance=0.05, mount=mount).to_list()

    assert len(out) == 1
    assert out[0].pose_tuple is not None
    # Identity rotations → composed translation is the component-wise sum.
    assert out[0].pose_tuple[:3] == (1.0, 1.0, 0.0)


def test_pose_fill_db_rejected_run_leaves_no_destination(tmp_path: Path) -> None:
    """A wrong ``target`` must not leave a file that blocks the corrected run."""
    src, dest = tmp_path / "memory.db", tmp_path / "memory_posed.db"
    with SqliteStore(path=str(src)) as store:
        store.stream("joint_state", float).append(1.0, ts=0.0)

    with pytest.raises(ValueError, match="target stream 'lidar'"):
        pose_fill_db(src, dest)

    assert not dest.exists()
