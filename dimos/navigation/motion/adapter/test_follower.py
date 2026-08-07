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

import math
from threading import RLock

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.motion.adapter.floor import FloorAnchor
from dimos.navigation.motion.adapter.follower import (
    GoalLatch,
    TrajectoryFollower,
    TrajectoryFollowerConfig,
    path_clearance,
)
from dimos.navigation.motion.control.tracks import TRACKS
from dimos.navigation.motion.planner.referee.scenarios import EMBODIMENTS


def test_clearance_is_band_distance_minus_half_width():
    xy = np.array([[0.0, 0.0]])
    points = np.array([[1.0, 0.0, 0.2]])  # inside the z band
    clr = path_clearance(xy, points, half_width=0.155)
    assert abs(clr[0] - (1.0 - 0.155)) < 1e-6


def test_clearance_ignores_points_outside_z_band():
    xy = np.array([[0.0, 0.0]])
    points = np.array([[0.1, 0.0, 0.01], [0.1, 0.0, 1.0]])  # floor + overhang
    assert np.isinf(path_clearance(xy, points, half_width=0.155)[0])


def test_clearance_empty_path():
    assert path_clearance(np.zeros((0, 2)), np.zeros((0, 3)), 0.1).shape == (0,)


def test_goal_latch_fires_once_then_holds():
    latch = GoalLatch(tolerance=0.2)
    latch.set_goal((1.0, 0.0))
    assert not latch.arrive((0.0, 0.0))
    assert latch.arrive((0.95, 0.0))
    assert latch.reached
    assert not latch.arrive((0.95, 0.0))


def test_goal_latch_ignores_sub_tolerance_goal_moves():
    latch = GoalLatch(tolerance=0.2)
    latch.set_goal((1.0, 0.0))
    assert latch.arrive((1.0, 0.0))
    latch.set_goal((1.05, 0.0))  # replan grid snap, same goal
    assert latch.reached
    latch.set_goal((3.0, 0.0))  # a new task
    assert not latch.reached


# --- the room hint's band (adapter/floor.py anchors it; this is the wiring)


def _room_follower(**config):
    """A TrajectoryFollower with just enough wired up to call _clearance_for."""
    follower = object.__new__(TrajectoryFollower)
    follower.config = TrajectoryFollowerConfig(**config)
    follower._lock = RLock()
    follower._track = TRACKS[follower.config.track]
    follower._half_width = EMBODIMENTS[follower.config.embodiment].width / 2.0
    follower._clearance = None
    follower._clearance_key = None
    follower._floor = FloorAnchor(
        doing="measuring the room",
        enabled=follower.config.floor_anchor,
        lidar_height=follower.config.lidar_height,
        ground_margin=follower.config.ground_margin_m,
        base_frame=follower.config.base_frame,
    )
    return follower


def _room_with_a_post(floor_z: float, n: int = 400) -> PointCloud2:
    """A floor slab at `floor_z` with a post 0.20..0.30 m above it, at x=1.

    Sunk far enough that the post is entirely UNDER the raw 0.05..0.45 band —
    the recording's case, where the map's z origin is base height.
    """
    a = np.arange(n) / n * 2 * math.pi
    slab = np.column_stack([2.0 * np.cos(a), 2.0 * np.sin(a), np.full(n, floor_z)])
    post = np.array([[1.0, 0.0, floor_z + 0.20 + 0.025 * k] for k in range(5)])
    return PointCloud2.from_numpy(np.concatenate([slab, post]).astype(np.float32), frame_id="odom")


def _straight_path() -> Path:
    return Path(
        ts=0.0,
        frame_id="odom",
        poses=[
            PoseStamped(ts=0.0, frame_id="odom", position=Vector3(0.0, 0.0, 0.0)),
            PoseStamped(ts=0.0, frame_id="odom", position=Vector3(0.5, 0.0, 0.0)),
        ],
    )


def test_the_room_hint_is_measured_on_the_floor_anchored_map():
    follower = _room_follower(lidar_height=0.45)
    follower._cloud = _room_with_a_post(-0.28)
    # unanchored, the post sits under the band and reads as infinite room —
    # the governor would drive at full speed into what the planner routed round
    blind = follower._clearance_for(_straight_path(), (0.0, 0.0))
    assert np.all(np.isinf(blind))
    # anchored off the tf prior: the post is where the planner sees it
    follower._floor.base_z, follower._floor.base_height = 0.05, 0.29  # prior: floor at -0.24
    follower._clearance_key = None
    hint = follower._clearance_for(_straight_path(), (0.0, 0.0))
    hw = follower._half_width
    assert abs(float(hint[0]) - (1.0 - hw)) < 1e-6
    assert abs(float(hint[1]) - (0.5 - hw)) < 1e-6


def test_an_unanchored_room_hint_is_the_raw_band():
    # no tf prior: the follower degrades to the band as it was, exactly as the
    # planner does, rather than anchoring to whatever the low quantile is
    follower = _room_follower()
    follower._cloud = _room_with_a_post(-0.28)
    path = _straight_path()
    want = path_clearance(
        np.array([[0.0, 0.0], [0.5, 0.0]]),
        follower._cloud.points_f32(),
        follower._half_width,
    )
    assert np.array_equal(follower._clearance_for(path, (0.0, 0.0)), want)


def test_the_anchor_switch_turns_the_room_hint_back_to_the_raw_band():
    follower = _room_follower(floor_anchor=False, lidar_height=0.45)
    follower._floor.base_z, follower._floor.base_height = 0.05, 0.29
    follower._cloud = _room_with_a_post(-0.28)
    path = _straight_path()
    want = path_clearance(
        np.array([[0.0, 0.0], [0.5, 0.0]]),
        follower._cloud.points_f32(),
        follower._half_width,
    )
    assert np.array_equal(follower._clearance_for(path, (0.0, 0.0)), want)


def test_a_room_hint_on_a_floor_already_at_zero_is_the_raw_band():
    # the referee's sim worlds put the plan poses on the ground; anchoring
    # there only drops the ground the body is standing on, which was never in
    # the band, so the hint the judge hands the controller cannot move
    follower = _room_follower(lidar_height=0.45)
    follower._floor.base_z, follower._floor.base_height = 0.29, 0.29  # prior: floor at 0.0
    follower._cloud = _room_with_a_post(0.0)
    path = _straight_path()
    want = path_clearance(
        np.array([[0.0, 0.0], [0.5, 0.0]]),
        follower._cloud.points_f32(),
        follower._half_width,
    )
    assert np.array_equal(follower._clearance_for(path, (0.0, 0.0)), want)
