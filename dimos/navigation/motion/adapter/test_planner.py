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

from dimos.msgs.nav_msgs.Path import Path as RefereePath
from dimos.navigation.motion.adapter import planner as planner_module
from dimos.navigation.motion.adapter.planner import (
    MotionPlanner,
    MotionPlannerConfig,
    carrot_along,
    to_nav_path,
)
from dimos.navigation.motion.control.laws.seed import PursuitController
from dimos.navigation.motion.embodiment import EMBODIMENTS, Embodiment
from dimos.navigation.motion.geometry import AvoidanceConfig
from dimos.navigation.motion.obstacles import RAW_BAND, hard_points, load as load_model
from dimos.navigation.motion.planner.planners.gold import pose_stamped
from dimos.navigation.motion.planner.planners.target import make_py
from dimos.navigation.motion.scenarios import Scenario


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
    """A MotionPlanner with just enough wired up to call _due and _retask."""
    planner = object.__new__(MotionPlanner)
    planner.config = MotionPlannerConfig(**config)
    planner._planned = None
    return planner


def test_a_tick_with_nothing_new_does_not_replan():
    planner = _gated_planner()
    assert planner._due(7, (2.0, 0.0))  # nothing planned yet
    planner._planned = (7, (2.0, 0.0))
    assert not planner._due(7, (2.0, 0.0))


def test_a_new_map_or_a_moved_carrot_replans():
    planner = _gated_planner()
    planner._planned = (7, (2.0, 0.0))
    assert planner._due(8, (2.0, 0.0))
    assert planner._due(7, (2.3, 0.0))


def test_a_republished_route_moves_the_carrot_by_nothing_and_is_not_a_replan():
    # MLS trims the route head to the robot and re-solves the tail on every
    # ~1 Hz republish: the waypoints move, the carrot does not
    planner = _gated_planner()
    planner._planned = (7, (2.0, 0.0))
    assert not planner._due(7, (2.02, -0.01))


def test_an_ungated_planner_replans_on_every_tick():
    planner = _gated_planner(replan_on_change=False)
    planner._planned = (7, (2.0, 0.0))
    assert planner._due(7, (2.0, 0.0))


def test_only_a_carrot_that_jumped_is_a_new_task():
    planner = _gated_planner()
    planner._planned = (7, (2.0, 0.0))
    assert not planner._retask((2.3, 0.0))  # republish wobble, same task
    assert planner._retask((6.6, 0.0))  # the door recording's reroute
    # and nothing to compare against is not a jump
    planner._planned = None
    assert not planner._retask((6.6, 0.0))


def test_the_gate_is_the_one_the_diagnosis_replays():
    # the free function is what diagnose.gated_ticks reconstructs the gate with
    assert planner_module.replan_due(None, 7, (2.0, 0.0))
    assert not planner_module.replan_due((7, (2.0, 0.0)), 7, (2.1, 0.0))
    assert planner_module.replan_due((7, (2.0, 0.0)), 7, (2.4, 0.0))
    assert planner_module.replan_due((7, (2.0, 0.0)), 8, (2.0, 0.0))


# --- the obstacle model (motion/obstacles.py is the rule; this is the wiring)


def _model_planner(**config):
    """A MotionPlanner with just enough wired up to select obstacles."""
    planner = object.__new__(MotionPlanner)
    planner.config = MotionPlannerConfig(**config)
    planner._lock = RLock()
    planner._emb = EMBODIMENTS[planner.config.embodiment]
    planner._model = load_model(planner.config.obstacle_model, planner._emb)
    return planner


def _room(floor_z: float, n: int = 400) -> np.ndarray:
    """A floor slab at `floor_z` with clutter 0.2..0.4 m above it."""
    a = np.arange(n) / n * 2 * math.pi
    slab = np.column_stack([np.cos(a), np.sin(a), np.full(n, floor_z)])
    clutter = np.array([[1.0, 0.0, floor_z + 0.2], [1.0, 0.2, floor_z + 0.4]])
    return np.concatenate([slab, clutter]).astype(np.float32)


def test_the_band_rides_the_body_not_the_map_origin():
    planner = _model_planner()
    # base at +0.01, so the surface the feet stand on is at -0.28
    out = hard_points(planner._model, _room(-0.28), ground_z=-0.28)
    # the slab is gone and the clutter reads as its true height over the ground
    assert len(out) == 2
    assert abs(float(out[:, 2].min()) - 0.2) < 1e-6
    assert abs(float(out[:, 2].max()) - 0.4) < 1e-6


def test_the_default_model_is_the_body_band():
    planner = _model_planner()
    assert planner.config.obstacle_model == "body_band"
    assert planner._model.body_referenced


def test_the_raw_band_model_ignores_the_body_reference():
    # the legacy model, for replaying recordings the deployed stack made
    # before the body was the reference: the map's z origin is the band's
    planner = _model_planner(obstacle_model="raw_band")
    pts = _room(-0.28)
    assert not planner._model.body_referenced
    out = hard_points(planner._model, pts, ground_z=-0.28)
    # the band stays at absolute 0.05..0.45: the 0.2 m clutter is under it and
    # invisible, only the 0.4 m one survives, and neither z has moved
    assert len(out) == 1
    assert abs(float(out[0][2]) - 0.12) < 1e-6


def test_a_ground_already_at_zero_selects_the_same_band():
    # the referee's sim worlds put the plan poses on the ground, so the two
    # models agree there and the judge's scores cannot move
    planner = _model_planner()
    raw = _model_planner(obstacle_model="raw_band")
    pts = _room(0.0)
    assert np.array_equal(hard_points(planner._model, pts, 0.0), hard_points(raw._model, pts, 0.0))


def _wall_over(ground_z: float, height: float) -> np.ndarray:
    """A wall across the route at `height` over the ground, and nothing else."""
    ys = np.arange(-1.0, 1.0 + 1e-9, 0.05)
    return np.column_stack([np.full(len(ys), 1.5), ys, np.full(len(ys), ground_z + height)]).astype(
        np.float32
    )


def _detour(emb: Embodiment, cloud: np.ndarray, ground_z: float) -> float:
    """How far off the straight line the plan goes, planning the way the module does."""
    hard = hard_points(load_model("body_band", emb), cloud, ground_z)
    episode = make_py(Scenario("tall", [], goal=(3.0, 0.0), emb=emb), AvoidanceConfig())
    episode.reset()
    path = episode.plan(hard[:, :2], (0.0, 0.0, 0.0), (3.0, 0.0))
    if len(path.poses) < 2:
        return math.inf  # a refusal is the strongest form of "it saw the wall"
    return max(abs(p.position.y) for p in path.poses)


def test_a_tall_body_plans_around_what_the_old_band_cut_off():
    """The latent bug the 2D search contract closes.

    A 0.55 m wall is inside a 0.60 m body's band and outside the absolute
    0.05..0.45 one. While the search re-sliced that band it dropped the very
    points the model had correctly kept, and drove straight through them.
    """
    wall = _wall_over(0.0, 0.55)
    assert wall[0][2] > RAW_BAND[1], "the fixture has to sit above the old band"
    tall = Embodiment(tag="tall", height=0.60)
    assert _detour(tall, wall, 0.0) > 0.8, "the tall body drove through its own obstacle"
    # and the control: the same wall IS over a go2's belly, so it is not a wall
    assert not len(hard_points(load_model("body_band", EMBODIMENTS["go2"]), wall, 0.0))
    assert _detour(EMBODIMENTS["go2"], wall, 0.0) < 0.2
