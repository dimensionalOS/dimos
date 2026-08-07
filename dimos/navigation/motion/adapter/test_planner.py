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
from types import SimpleNamespace

import numpy as np

from dimos.navigation.motion.adapter import floor as floor_module, planner as planner_module
from dimos.navigation.motion.adapter.floor import FloorAnchor
from dimos.navigation.motion.adapter.planner import (
    MotionPlanner,
    MotionPlannerConfig,
    carrot_along,
    to_nav_path,
)
from dimos.navigation.motion.control.laws.seed import PursuitController
from dimos.navigation.motion.planner.referee.planners.gold import pose_stamped
from dimos.navigation.motion.planner.referee.types import Path as RefereePath


def test_to_nav_path_preserves_positions_and_yaw():
    ref = RefereePath(
        frame_id="world", poses=[pose_stamped(0.0, 0.0, 0.0), pose_stamped(1.0, 2.0, math.pi / 2)]
    )
    nav = to_nav_path(ref, ts=3.0, frame_id="odom")
    assert nav.frame_id == "odom"
    assert len(nav.poses) == 2
    assert nav.poses[1].position.x == 1.0
    assert nav.poses[1].position.y == 2.0
    assert abs(nav.poses[1].yaw - math.pi / 2) < 1e-9


def test_to_nav_path_empty():
    assert len(to_nav_path(RefereePath(frame_id="world", poses=[])).poses) == 0


def test_carrot_walks_arc_from_closest_waypoint():
    path = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 4.0]])
    # closest waypoint to (2.1, 0.5) is (2, 0); 1.5 m of arc up the second leg
    assert carrot_along(path, (2.1, 0.5), 1.5) == (2.0, 1.5)


def test_carrot_interpolates_within_segment():
    path = np.array([[0.0, 0.0], [10.0, 0.0]])
    assert carrot_along(path, (0.0, 0.0), 5.0) == (5.0, 0.0)


def test_carrot_clamps_to_path_end():
    path = np.array([[0.0, 0.0], [1.0, 0.0]])
    assert carrot_along(path, (0.9, 0.0), 5.0) == (1.0, 0.0)


def test_carrot_single_waypoint():
    assert carrot_along(np.array([[3.0, 4.0]]), (0.0, 0.0), 5.0) == (3.0, 4.0)


def _holding_planner():
    """A MotionPlanner with just enough wired up to call _hold."""
    planner = object.__new__(MotionPlanner)
    planner._stale = False
    planner.config = MotionPlannerConfig()
    planner._viz_at = 0.0
    published, drawn = [], []
    planner.path = SimpleNamespace(publish=published.append)
    planner.plan_body = SimpleNamespace(publish=drawn.append)
    return planner, published, drawn


def test_hold_publishes_single_pose_stub_at_the_current_pose():
    planner, published, _drawn = _holding_planner()
    planner._hold((1.5, -2.0, math.pi / 2), age=7.0)
    assert len(published) == 1
    path = published[0]
    assert path.frame_id == "odom"
    # a single pose is the planner's refusal shape: "hold, no safe route"
    assert len(path.poses) == 1
    assert path.poses[0].position.x == 1.5
    assert path.poses[0].position.y == -2.0
    assert abs(path.poses[0].yaw - math.pi / 2) < 1e-9


def test_hold_stub_stops_the_controller():
    planner, published, _drawn = _holding_planner()
    planner._hold((1.5, -2.0, 0.0), age=7.0)
    pose = pose_stamped(1.5, -2.0, 0.0)
    twist = PursuitController().update(pose, published[0], t=0.0)
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)


def test_hold_warns_once_per_stale_episode(monkeypatch):
    warnings = []
    monkeypatch.setattr(
        planner_module.logger, "warning", lambda msg, **kw: warnings.append(msg), raising=False
    )
    planner, _published, _drawn = _holding_planner()
    for _ in range(3):
        planner._hold((0.0, 0.0, 0.0), age=7.0)
    assert planner._stale
    # edge-triggered: replan_hz would otherwise warn 5x a second for as long
    # as the link stays down
    assert len(warnings) == 1


def test_hold_draws_the_veto_so_it_is_not_mistaken_for_a_dead_module():
    """A refusal must reach the viewer: an empty viewport looks like a crash."""
    planner, published, drawn = _holding_planner()
    planner._hold((1.0, 2.0, 0.0), age=7.0)
    assert len(drawn) == 1
    assert drawn[0] is published[0]
    assert len(drawn[0].poses) == 1


# --- the replan gate


def _gated_planner(**config):
    """A MotionPlanner with just enough wired up to call _due."""
    planner = object.__new__(MotionPlanner)
    planner.config = MotionPlannerConfig(**config)
    planner._planned = None
    return planner


def test_a_tick_with_nothing_new_does_not_replan():
    planner = _gated_planner()
    assert planner._due((7, 2))  # nothing planned yet
    planner._planned = (7, 2)
    assert not planner._due((7, 2))


def test_a_new_map_or_a_new_route_replans():
    planner = _gated_planner()
    planner._planned = (7, 2)
    assert planner._due((8, 2))
    assert planner._due((7, 3))


def test_an_ungated_planner_replans_on_every_tick():
    planner = _gated_planner(replan_on_change=False)
    planner._planned = (7, 2)
    assert planner._due((7, 2))


def test_a_route_republished_unchanged_is_not_a_change():
    # MLS republishes at ~1 Hz and holds still for seconds at a time
    route = np.array([[0.0, 0.0], [1.0, 0.0]])
    assert not planner_module.route_changed(route, route.copy())


def test_a_route_that_moved_is_a_change():
    route = np.array([[0.0, 0.0], [1.0, 0.0]])
    assert planner_module.route_changed(route, np.array([[0.0, 0.0], [1.0, 0.1]]))
    assert planner_module.route_changed(route, np.array([[0.0, 0.0]]))


def test_a_route_appearing_or_going_away_is_a_change():
    route = np.array([[0.0, 0.0], [1.0, 0.0]])
    assert planner_module.route_changed(None, route)
    assert planner_module.route_changed(route, None)
    assert not planner_module.route_changed(None, None)


# --- the floor anchoring (adapter/floor.py is the estimator; this is the wiring)


def _anchoring_planner(**config):
    """A MotionPlanner with just enough wired up to anchor a cloud."""
    planner = object.__new__(MotionPlanner)
    planner.config = MotionPlannerConfig(**config)
    planner._lock = RLock()
    planner._floor = FloorAnchor(
        doing="planning",
        enabled=planner.config.floor_anchor,
        lidar_height=planner.config.lidar_height,
        ground_margin=planner.config.ground_margin_m,
        base_frame=planner.config.base_frame,
    )
    return planner


def _room(floor_z: float, n: int = 400) -> np.ndarray:
    """A floor slab at `floor_z` with clutter 0.2..0.4 m above it."""
    a = np.arange(n) / n * 2 * math.pi
    slab = np.column_stack([np.cos(a), np.sin(a), np.full(n, floor_z)])
    clutter = np.array([[1.0, 0.0, floor_z + 0.2], [1.0, 0.2, floor_z + 0.4]])
    return np.concatenate([slab, clutter]).astype(np.float32)


def test_anchor_moves_the_band_onto_the_floor():
    planner = _anchoring_planner(lidar_height=0.45)
    planner._floor.base_z, planner._floor.base_height = 0.05, 0.29  # prior: floor at -0.24
    out = planner._floor.anchor(_room(-0.28), (0.0, 0.0))
    # the slab is gone and the clutter reads as its true height over the floor
    assert len(out) == 2
    assert abs(float(out[:, 2].min()) - 0.2) < 1e-6
    assert abs(float(out[:, 2].max()) - 0.4) < 1e-6


def test_anchor_without_a_tf_prior_leaves_the_cloud_alone():
    # a low quantile of the cloud alone is only the floor if the floor is in
    # the cloud; without tf saying where the ground is, the band does not move
    planner = _anchoring_planner()
    pts = _room(-0.28)
    assert np.array_equal(planner._floor.anchor(pts, (0.0, 0.0)), pts)


def test_anchor_warns_once_when_it_cannot_anchor(monkeypatch):
    warnings = []
    monkeypatch.setattr(
        floor_module.logger, "warning", lambda msg, **kw: warnings.append(msg), raising=False
    )
    planner = _anchoring_planner()
    for _ in range(3):
        planner._floor.anchor(_room(-0.28), (0.0, 0.0))
    assert len(warnings) == 1


def test_anchor_is_off_when_the_switch_is_off():
    planner = _anchoring_planner(floor_anchor=False, lidar_height=0.45)
    planner._floor.base_z, planner._floor.base_height = 0.05, 0.29
    pts = _room(-0.28)
    assert np.array_equal(planner._floor.anchor(pts, (0.0, 0.0)), pts)


def test_anchoring_a_floor_already_at_zero_shifts_nothing():
    # the referee's sim worlds put the plan poses on the ground; anchoring
    # there must not MOVE anything, only drop the ground it is standing on
    planner = _anchoring_planner(lidar_height=0.45)
    planner._floor.base_z, planner._floor.base_height = 0.29, 0.29  # prior: floor at 0.0
    pts = _room(0.0)
    out = planner._floor.anchor(pts, (0.0, 0.0))
    assert np.array_equal(out, pts[pts[:, 2] > planner.config.ground_margin_m])


def test_the_prior_bounds_an_estimate_the_cloud_gets_wrong():
    # a cloud with no floor in it (a wall the robot faces): the low quantile
    # is the wall's base, and the prior is what stops the wall being deleted
    planner = _anchoring_planner(lidar_height=0.45)
    planner._floor.base_z, planner._floor.base_height = 0.05, 0.29  # prior: floor at -0.24
    wall = np.column_stack(
        [np.full(400, 1.0), np.linspace(-1.0, 1.0, 400), np.linspace(0.6, 1.4, 400)]
    ).astype(np.float32)
    out = planner._floor.anchor(wall, (0.0, 0.0))
    # shifted by the PRIOR (-0.24), not by the wall's own base (0.6)
    assert abs(float(out[:, 2].min()) - (0.6 + 0.24)) < 1e-5
