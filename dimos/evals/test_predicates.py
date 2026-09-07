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

"""Predicate library tests: a real SqliteStore, hand-built GT pose series."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from dimos.evals.predicates import (
    GT_STREAM,
    contained,
    displaced,
    grasped,
    gt_poses,
    knocked_over,
    lifted,
    near,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3, make_vector3

ROLES = {"cup": "cup", "bowl": "bowl"}


def _pose(x: float, y: float, z: float, orientation: Quaternion | None = None) -> PoseStamped:
    return PoseStamped(
        position=make_vector3(x, y, z),
        orientation=orientation or Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
    )


def _gt_store(tmp_path: Path, series: dict[str, list[PoseStamped]]) -> Any:
    """A GT db with each body's pose series on the GT stream (frame_id = body)."""
    from dimos.memory.store.sqlite import SqliteStore

    tmp_path.mkdir(parents=True, exist_ok=True)
    try:
        store = SqliteStore(path=str(tmp_path / "gt.db"))
    except Exception as e:  # pragma: no cover — sqlite-vec unavailable platforms
        pytest.skip(f"SqliteStore unavailable: {e}")
    stream = store.stream(GT_STREAM, PoseStamped)
    for body, poses in series.items():
        for i, pose in enumerate(poses):
            pose.frame_id = body
            stream.append(pose, ts=1000.0 + i)
    return store


def test_gt_poses_filters_by_role(tmp_path: Path) -> None:
    store = _gt_store(tmp_path, {"cup": [_pose(0.5, 0.0, 0.1)], "bowl": [_pose(0.2, 0.1, 0.05)]})
    try:
        poses = gt_poses(store, ROLES, "cup")
        assert len(poses) == 1 and poses[0].position.x == 0.5
    finally:
        store.stop()


def test_gt_poses_raises_until_data(tmp_path: Path) -> None:
    store = _gt_store(tmp_path, {})
    try:
        with pytest.raises(LookupError):  # stream missing entirely
            gt_poses(store, ROLES, "cup")
        store.stream(GT_STREAM, PoseStamped).append(_pose(0.0, 0.0, 0.0), ts=1000.0)
        with pytest.raises(LookupError):  # stream live, but nothing for this body
            gt_poses(store, ROLES, "cup")
    finally:
        store.stop()


def test_lifted_and_displaced(tmp_path: Path) -> None:
    rising = [_pose(0.5, 0.0, 0.1), _pose(0.5, 0.0, 0.2)]
    slid = [_pose(0.2, 0.1, 0.05), _pose(0.45, 0.1, 0.05)]
    store = _gt_store(tmp_path, {"cup": rising, "bowl": slid})
    try:
        assert lifted(store, ROLES, "cup", min_delta=0.05) == 1.0
        assert lifted(store, ROLES, "cup", min_delta=0.5) == 0.0
        assert displaced(store, ROLES, "cup", threshold=0.1) == 0.0  # straight up, no xy
        assert displaced(store, ROLES, "bowl", threshold=0.1) == 1.0
    finally:
        store.stop()


def test_knocked_over(tmp_path: Path) -> None:
    on_side = Quaternion.from_euler(Vector3(1.5708, 0.0, 0.0))  # rolled 90°
    store = _gt_store(tmp_path, {"cup": [_pose(0.5, 0.0, 0.1), _pose(0.5, 0.0, 0.05, on_side)]})
    try:
        assert knocked_over(store, ROLES, "cup") == 1.0
    finally:
        store.stop()

    store = _gt_store(tmp_path / "upright", {"cup": [_pose(0.5, 0.0, 0.1)]})
    try:
        assert knocked_over(store, ROLES, "cup") == 0.0
    finally:
        store.stop()


def test_near_and_contained(tmp_path: Path) -> None:
    store = _gt_store(
        tmp_path,
        {"cup": [_pose(0.24, 0.0, 0.2)], "bowl": [_pose(0.2, 0.0, 0.05)]},
    )
    try:
        assert near(store, ROLES, "cup", "bowl", dist=0.2) == 1.0
        assert near(store, ROLES, "cup", "bowl", dist=0.1) == 0.0
        # inside the bowl's tolerance disc, above its origin -> in
        assert contained(store, ROLES, "cup", "bowl", xy_tol=0.1) == 1.0
        assert contained(store, ROLES, "cup", "bowl", xy_tol=0.01) == 0.0
    finally:
        store.stop()

    below = _gt_store(
        tmp_path / "below",
        {"cup": [_pose(0.24, 0.0, 0.01)], "bowl": [_pose(0.2, 0.0, 0.05)]},
    )
    try:
        assert contained(below, ROLES, "cup", "bowl", xy_tol=0.1) == 0.0  # under, not in
    finally:
        below.stop()


def test_grasped_is_a_phase2_placeholder(tmp_path: Path) -> None:
    store = _gt_store(tmp_path, {"cup": [_pose(0.5, 0.0, 0.1)]})
    try:
        with pytest.raises(NotImplementedError, match="contact"):
            grasped(store, ROLES, "cup")
    finally:
        store.stop()
