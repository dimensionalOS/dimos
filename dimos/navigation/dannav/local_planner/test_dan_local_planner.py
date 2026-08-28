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

"""Tests for ``_ReplanGate`` commit-window gating."""

from __future__ import annotations

import math
from typing import Any

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.dannav.local_planner.module import (
    DanLocalPlannerConfig,
    _ReplanGate,
)


def _path_from_points(points: list[tuple[float, float]]) -> Path:
    poses = [
        PoseStamped(
            ts=1.0, frame_id="world", position=[x, y, 0.0], orientation=[0.0, 0.0, 0.0, 1.0]
        )
        for x, y in points
    ]
    return Path(frame_id="world", poses=poses)


def _odom(x: float, y: float, *, ts: float = 1.0) -> PoseStamped:
    return PoseStamped(
        ts=ts,
        frame_id="world",
        position=[x, y, 0.0],
        orientation=[0.0, 0.0, 0.0, 1.0],
    )


def _point(x: float, y: float) -> PointStamped:
    return PointStamped(x=x, y=y, z=0.0, frame_id="world")


def _gate(**config: Any) -> _ReplanGate:
    config.setdefault("resample_spacing_m", 0.0)
    return _ReplanGate(DanLocalPlannerConfig(**config))


def test_replan_after_advancing_lock_is_published() -> None:
    gate = _gate(lock_replan=0.5)
    gate.on_odom(_odom(0.0, 0.0))
    gate.on_planner_path(_path_from_points([(0.0, 0.0), (5.0, 0.0)]))

    # A replan within the window is held...
    gate.on_odom(_odom(0.3, 0.0))
    assert gate.on_planner_path(_path_from_points([(0.3, 0.0), (5.0, 0.0)])) is None

    # ...then once the robot has advanced >= L the next replan commits and
    # re-anchors, so a subsequent replan is held again until the new window.
    gate.on_odom(_odom(0.6, 0.0))
    committed = _path_from_points([(0.6, 0.0), (5.0, 0.0)])
    assert gate.on_planner_path(committed) is not None
    gate.on_odom(_odom(0.7, 0.0))
    assert gate.on_planner_path(_path_from_points([(0.7, 0.0), (5.0, 0.0)])) is None


def test_fresh_click_commits_within_lock() -> None:
    gate = _gate(lock_replan=0.5)
    gate.on_odom(_odom(0.0, 0.0))
    gate.on_planner_path(_path_from_points([(0.0, 0.0), (5.0, 0.0)]))

    # The robot has only crept forward (< L), so the window has not released...
    gate.on_odom(_odom(0.1, 0.0))
    # ...but a fresh click whose path actually reaches the goal commits anyway
    # ("the dog agrees when clicked"), and the arm is consumed.
    gate.on_goal(_point(5.0, 0.0))
    clicked = _path_from_points([(0.1, 0.0), (5.0, 0.0)])
    assert gate.on_planner_path(clicked) is not None
    assert gate._armed_goal is None


def test_fresh_click_does_not_commit_a_stale_replan() -> None:
    # The click arms a commit, but a path that does NOT end at the clicked goal
    # (a stale in-flight replan for the previous goal) is still held within L,
    # and the arm stays set until a path that reaches the goal arrives.
    gate = _gate(lock_replan=0.5)
    gate.on_odom(_odom(0.0, 0.0))
    gate.on_planner_path(_path_from_points([(0.0, 0.0), (5.0, 0.0)]))
    gate.on_odom(_odom(0.1, 0.0))
    gate.on_goal(_point(9.0, 0.0))
    stale = _path_from_points([(0.1, 0.0), (5.0, 0.0)])  # ends at the OLD goal
    assert gate.on_planner_path(stale) is None
    assert gate._armed_goal is not None


def test_empty_path_published_and_resets_gate() -> None:
    gate = _gate(lock_replan=0.5)
    gate.on_odom(_odom(0.0, 0.0))
    gate.on_planner_path(_path_from_points([(0.0, 0.0), (5.0, 0.0)]))

    # An empty path (nothing safe ahead) forwards immediately as a stop and
    # drops the committed path.
    empty = Path(frame_id="world", poses=[])
    assert gate.on_planner_path(empty) is empty
    assert gate._committed is None

    # The reset means the next path cold-starts a commit even though the robot
    # has not advanced a window's worth.
    gate.on_odom(_odom(0.1, 0.0))
    restart = _path_from_points([(0.1, 0.0), (5.0, 0.0)])
    assert gate.on_planner_path(restart) is not None


def test_cancel_resets_committed_path() -> None:
    # A NaN goal is MovementManager's cancel: it disarms AND drops the committed
    # path, so the next planner path cold-starts a fresh commit within L.
    gate = _gate(lock_replan=0.5)
    gate.on_odom(_odom(0.0, 0.0))
    gate.on_planner_path(_path_from_points([(0.0, 0.0), (5.0, 0.0)]))
    gate.on_goal(_point(math.nan, math.nan))
    assert gate._committed is None

    gate.on_odom(_odom(0.1, 0.0))
    restart = _path_from_points([(0.1, 0.0), (5.0, 0.0)])
    assert gate.on_planner_path(restart) is not None


def test_lock_replan_zero_commits_every_non_empty_path() -> None:
    # L = 0 disables gating: every non-empty path commits, even with no advance.
    gate = _gate(lock_replan=0.0)
    gate.on_odom(_odom(0.0, 0.0))
    for path in (
        _path_from_points([(0.0, 0.0), (1.0, 0.0)]),
        _path_from_points([(0.0, 0.0), (0.0, 1.0)]),
        _path_from_points([(0.0, 0.0), (1.0, 1.0)]),
    ):
        assert gate.on_planner_path(path) is not None
