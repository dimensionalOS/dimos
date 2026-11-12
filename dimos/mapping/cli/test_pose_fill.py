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

"""Unit tests for pose filling and dataset validation (no LFS data).

The end-to-end `dimos map pose-fill` path against a real recording lives in
`test_cli.py` (self-hosted).
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


@pytest.fixture
def pose_fill_source(tmp_path: Path) -> Path:
    src = tmp_path / "memory.db"
    with SqliteStore(path=str(src)) as store:
        store.stream("lidar", str).append("matched", ts=1.0)
        store.stream("lidar").append("unmatched", ts=2.0)
        store.stream("odom", Transform).append(
            Transform(translation=Vector3(1.0, 2.0, 3.0)), ts=1.01
        )
        store.stream("other", str).append("preserved", ts=1.0, pose=(4, 5, 6))
    return src


@pytest.mark.parametrize(
    ("target", "pose_source", "streams", "match"),
    [
        pytest.param("missing", "odom", None, "target stream 'missing'", id="missing-target"),
        pytest.param("lidar", "odom", ["odom"], "target stream 'lidar'", id="excluded-target"),
        pytest.param(
            "lidar", "missing", None, "pose_source stream 'missing'", id="missing-pose-source"
        ),
        pytest.param(
            "missing",
            "odom",
            ["missing"],
            "streams not found.*missing",
            id="selected-missing-target",
        ),
        pytest.param(
            "lidar",
            "odom",
            ["lidar", "missing"],
            "streams not found.*missing",
            id="missing-other-stream",
        ),
    ],
)
def test_pose_fill_db_rejected_run_leaves_no_destination(
    pose_fill_source, tmp_path, target, pose_source, streams, match
):
    dest = tmp_path / "memory_posed.db"

    with pytest.raises(ValueError, match=match):
        pose_fill_db(
            pose_fill_source, dest, target=target, pose_source=pose_source, streams=streams
        )

    assert not dest.exists()


def test_pose_fill_db_copies_selected_streams(pose_fill_source, tmp_path):
    dest = tmp_path / "memory_posed.db"

    written = pose_fill_db(pose_fill_source, dest, streams=["lidar", "other"])

    assert written == {"lidar": 1, "other": 1}
    with SqliteStore(path=str(dest), must_exist=True) as store:
        assert store.list_streams() == ["lidar", "other"]
        assert [(obs.data, obs.pose_tuple) for obs in store.stream("lidar")] == [
            ("matched", (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0))
        ]
        assert [(obs.data, obs.pose_tuple) for obs in store.stream("other")] == [
            ("preserved", (4.0, 5.0, 6.0, 0.0, 0.0, 0.0, 1.0))
        ]


@pytest.mark.parametrize(
    ("seek", "duration", "expected"),
    [
        (0.25, 0.1, []),
        (1.0, 1.0, [101.0]),
        (0.0, 1.0, [100.0]),
        (1.0, None, [101.0, 102.0]),
        (0.0, 0.0, []),
        (0.0, None, [100.0, 101.0, 102.0]),
    ],
)
def test_pose_fill_db_uses_half_open_recording_window(
    tmp_path: Path, seek: float, duration: float | None, expected: list[float]
) -> None:
    source = tmp_path / "source.db"
    destination = tmp_path / "destination.db"
    with SqliteStore(path=str(source)) as store:
        for ts in (100.0, 101.0, 102.0):
            store.stream("lidar", str).append(f"scan-{ts}", ts=ts)
            store.stream("odom", Transform).append(Transform(translation=Vector3(ts, 0, 0)), ts=ts)

    written = pose_fill_db(source, destination, seek=seek, duration=duration)

    assert written == {"lidar": len(expected), "odom": len(expected)}
    with SqliteStore(path=str(destination), must_exist=True) as store:
        lidar = store.stream("lidar").order_by("ts").to_list()
        assert [obs.ts for obs in lidar] == expected
        assert [obs.data for obs in lidar] == [f"scan-{ts}" for ts in expected]
        assert [obs.pose_tuple for obs in lidar] == [
            (ts, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0) for ts in expected
        ]
        assert [obs.ts for obs in store.stream("odom").order_by("ts")] == expected


@pytest.mark.parametrize("streams", [None, ["lidar", "aux"]])
def test_pose_fill_db_shares_window_across_offset_streams(
    tmp_path: Path, streams: list[str] | None
) -> None:
    source = tmp_path / "source.db"
    destination = tmp_path / "destination.db"
    with SqliteStore(path=str(source)) as store:
        store.stream("empty", str)
        store.stream("reference", str).append("start", ts=100.0)
        for ts in (100.25, 101.0, 101.25, 101.5, 102.0):
            store.stream("lidar", str).append(f"scan-{ts}", ts=ts)
        for ts in (100.5, 101.0, 101.25, 101.5, 102.0):
            store.stream("odom", Transform).append(Transform(), ts=ts)
        for ts in (100.75, 101.0, 101.25, 101.5, 102.0):
            store.stream("aux", str).append(f"aux-{ts}", ts=ts)

    written = pose_fill_db(source, destination, streams=streams, seek=1.0, duration=0.5)

    expected = {"lidar": 2, "aux": 2}
    if streams is None:
        expected.update(empty=0, reference=0, odom=2)
    assert written == expected
    with SqliteStore(path=str(destination), must_exist=True) as store:
        assert set(store.list_streams()) == set(expected)
        assert [obs.ts for obs in store.stream("lidar").order_by("ts")] == [101.0, 101.25]
        assert [obs.ts for obs in store.stream("aux").order_by("ts")] == [101.0, 101.25]


def test_pose_fill_db_empty_recording(tmp_path: Path) -> None:
    source = tmp_path / "source.db"
    destination = tmp_path / "destination.db"
    with SqliteStore(path=str(source)) as store:
        store.stream("lidar", str)
        store.stream("odom", Transform)

    assert pose_fill_db(source, destination, seek=0.25, duration=0.1) == {"lidar": 0, "odom": 0}
