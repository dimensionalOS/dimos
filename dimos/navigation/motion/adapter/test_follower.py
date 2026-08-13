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
from dimos.navigation.motion.adapter.follower import (
    GoalLatch,
    TrajectoryFollower,
    TrajectoryFollowerConfig,
    path_clearance,
)
from dimos.navigation.motion.control.tracks import TRACKS
from dimos.navigation.motion.embodiment import EMBODIMENTS
from dimos.navigation.motion.obstacles import hard_points, load as load_model


def test_clearance_is_obstacle_distance_minus_half_width():
    xy = np.array([[0.0, 0.0]])
    points = np.array([[1.0, 0.0, 0.2]])
    clr = path_clearance(xy, points, half_width=0.155)
    assert abs(clr[0] - (1.0 - 0.155)) < 1e-6


def test_clearance_reads_every_point_it_is_handed_whatever_its_z():
    # the model already decided; a floor or ceiling z arriving here means the
    # model KEPT it, and re-judging it would price a world nobody planned
    xy = np.array([[0.0, 0.0]])
    for z in (-0.5, 0.01, 0.2, 0.46, 1.0):
        points = np.array([[0.1, 0.0, z]])
        assert abs(path_clearance(xy, points, half_width=0.0)[0] - 0.1) < 1e-6


def test_clearance_empty_path_or_no_obstacles():
    assert path_clearance(np.zeros((0, 2)), np.zeros((0, 3)), 0.1).shape == (0,)
    assert np.isinf(path_clearance(np.zeros((1, 2)), np.zeros((0, 3)), 0.1)[0])


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


# --- the room hint's band (motion/obstacles.py is the rule; this is the wiring)


def _room_follower(**config):
    """A TrajectoryFollower with just enough wired up to call _clearance_for.

    The real constructor stands up the module's LCM RPC transport (a
    run_forever + _lcm_loop thread pair per instance), which these pure
    geometry tests neither use nor can tear down.
    """
    follower = object.__new__(TrajectoryFollower)
    follower.config = TrajectoryFollowerConfig(**config)
    follower._lock = RLock()
    follower._track = TRACKS[follower.config.track]
    follower._emb = EMBODIMENTS[follower.config.embodiment]
    follower._model = load_model(follower.config.obstacle_model, follower._emb)
    follower._half_width = follower._emb.width / 2.0
    follower._clearance = None
    follower._clearance_key = None
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


def _base_at(z: float) -> PoseStamped:
    """The tf-resolved base pose; the ground sits emb.base_height under it."""
    return PoseStamped(ts=0.0, frame_id="odom", position=Vector3(0.0, 0.0, z))


def _straight_path() -> Path:
    return Path(
        ts=0.0,
        frame_id="odom",
        poses=[
            PoseStamped(ts=0.0, frame_id="odom", position=Vector3(0.0, 0.0, 0.0)),
            PoseStamped(ts=0.0, frame_id="odom", position=Vector3(0.5, 0.0, 0.0)),
        ],
    )


def test_the_room_hint_is_measured_against_the_body_reference():
    follower = _room_follower()
    follower._cloud = _room_with_a_post(-0.28)
    # the base rides 0.29 m over the ground, so the post is where the planner
    # sees it: 0.20..0.30 m up, well inside the band
    hint = follower._clearance_for(_straight_path(), _base_at(0.01))
    hw = follower._half_width
    assert abs(float(hint[0]) - (1.0 - hw)) < 1e-6
    assert abs(float(hint[1]) - (0.5 - hw)) < 1e-6


def test_the_raw_band_model_reads_the_map_origin_instead():
    # the legacy model: on a LIO stack the post sits under the absolute band
    # and reads as infinite room, which is what the robot actually ran
    follower = _room_follower(obstacle_model="raw_band")
    follower._cloud = _room_with_a_post(-0.28)
    blind = follower._clearance_for(_straight_path(), _base_at(0.01))
    assert np.all(np.isinf(blind))


def test_the_room_hint_is_the_model_hard_set_not_the_whole_map():
    # The governor has to measure the very points the search routed around.
    # path_clearance takes every point it is handed, so the whole map would
    # price a ceiling as room taken away -- which is what the control is here.
    follower = _room_follower()
    raw = _room_with_a_post(-0.28).points_f32()
    whole = np.concatenate([raw, np.array([[0.25, 0.0, -0.28 + 1.5]], dtype=np.float32)])
    follower._cloud = PointCloud2.from_numpy(whole, frame_id="odom")
    xy = np.array([[0.0, 0.0], [0.5, 0.0]])
    hard = hard_points(follower._model, whole, -0.28)
    want = path_clearance(xy, hard, follower._half_width)
    assert np.array_equal(follower._clearance_for(_straight_path(), _base_at(0.01)), want)
    over = path_clearance(xy, whole, follower._half_width)
    assert over[0] < want[0], "the whole map has to read tighter, or this proves nothing"


def test_a_room_hint_on_a_ground_already_at_zero_matches_the_raw_band():
    # the referee's sim worlds put the plan poses on the ground, so the two
    # models agree there and the hint the judge hands the controller cannot move
    follower = _room_follower()
    raw = _room_follower(obstacle_model="raw_band")
    cloud = _room_with_a_post(0.0)
    follower._cloud = raw._cloud = cloud
    assert np.array_equal(
        follower._clearance_for(_straight_path(), _base_at(0.29)),
        raw._clearance_for(_straight_path(), _base_at(0.29)),
    )
