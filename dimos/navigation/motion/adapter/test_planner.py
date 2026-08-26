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

from dataclasses import replace
import math

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.motion.adapter import planner as planner_module
from dimos.navigation.motion.adapter.planner import (
    MotionPlanner,
    MotionPlannerConfig,
    carrot_along,
    stamped,
)
from dimos.navigation.motion.control.laws.seed import PursuitController
from dimos.navigation.motion.embodiment.base import Embodiment
from dimos.navigation.motion.embodiment.go2 import GO2
from dimos.navigation.motion.obstacles import hard_points, load as load_model
from dimos.navigation.motion.planner.planners.base import pose_stamped
from dimos.navigation.motion.planner.planners.target import make_py

# Modules built by the helpers below. The real constructor stands up the module's
# LCM RPC transport (a run_forever + _lcm_loop daemon pair per instance); these
# tests exercise pure methods on top of it, so the fixture hands them back.
_BUILT: list[MotionPlanner] = []


@pytest.fixture(autouse=True)
def _stop_modules():
    yield
    while _BUILT:
        _BUILT.pop().stop()


def _planner(**config) -> MotionPlanner:
    planner = MotionPlanner(**config)
    _BUILT.append(planner)
    return planner


def test_stamped_preserves_positions_and_yaw():
    ref = Path(
        frame_id="world", poses=[pose_stamped(0.0, 0.0, 0.0), pose_stamped(1.0, 2.0, math.pi / 2)]
    )
    nav = stamped(ref, ts=3.0, frame_id="odom")
    assert nav.frame_id == "odom"
    assert len(nav.poses) == 2
    assert nav.poses[1].position.x == 1.0
    assert nav.poses[1].position.y == 2.0
    assert abs(nav.poses[1].yaw - math.pi / 2) < 1e-9


def test_stamped_empty():
    assert len(stamped(Path(frame_id="world", poses=[])).poses) == 0


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
    """A MotionPlanner with both of its output ports tapped."""
    planner = _planner()
    published, drawn = [], []
    planner.path.subscribe(published.append)
    planner.plan_body.subscribe(drawn.append)
    return planner, published, drawn


def test_hold_publishes_single_pose_stub_at_the_current_pose():
    planner, published, _drawn = _holding_planner()
    planner._hold(
        PoseStamped(
            position=(1.5, -2.0, 0.0), orientation=Quaternion.from_euler(Vector3(0, 0, math.pi / 2))
        ),
        age=7.0,
    )
    assert len(published) == 1
    path = published[0]
    assert path.frame_id == "odom"
    # a single pose is the planner's refusal shape: "hold, no safe route"
    assert len(path.poses) == 1
    assert path.poses[0].position.x == 1.5
    assert path.poses[0].position.y == -2.0
    assert abs(path.poses[0].yaw - math.pi / 2) < 1e-9
    # the stub stands on the ground like every planned pose, not at z=0
    assert abs(path.poses[0].position.z - (0.0 - planner._emb.base_height)) < 1e-9


def test_hold_stub_stops_the_controller():
    planner, published, _drawn = _holding_planner()
    planner._hold(PoseStamped(position=(1.5, -2.0, 0.0)), age=7.0)
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
        planner._hold(PoseStamped(position=(0.0, 0.0, 0.0)), age=7.0)
    assert planner._stale
    # edge-triggered: replan_hz would otherwise warn 5x a second for as long
    # as the link stays down
    assert len(warnings) == 1


def test_hold_draws_the_veto_so_it_is_not_mistaken_for_a_dead_module():
    """A refusal must reach the viewer: an empty viewport looks like a crash."""
    planner, published, drawn = _holding_planner()
    planner._hold(PoseStamped(position=(1.0, 2.0, 0.0)), age=7.0)
    assert len(drawn) == 1
    assert drawn[0] is published[0]
    assert len(drawn[0].poses) == 1


# --- the replan gate


def _gated_planner(**config):
    """A MotionPlanner for the replan gate: _due and _retask."""
    return _planner(**config)


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
    # a free function so a post-mortem can reconstruct the gate off a recording
    assert planner_module.replan_due(None, 7, (2.0, 0.0))
    assert not planner_module.replan_due((7, (2.0, 0.0)), 7, (2.1, 0.0))
    assert planner_module.replan_due((7, (2.0, 0.0)), 7, (2.4, 0.0))
    assert planner_module.replan_due((7, (2.0, 0.0)), 8, (2.0, 0.0))


# --- the obstacle model (motion/obstacles.py is the rule; this is the wiring)


def _model_planner(**config):
    """A MotionPlanner for the obstacle-model wiring."""
    return _planner(**config)


def _room(floor_z: float, n: int = 400) -> NDArray[np.float64]:
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


def test_a_map_with_a_non_finite_return_still_plans():
    # one NaN x used to reach the search as a NaN grid corner: every tick
    # raised, "keeping the last published path" forever
    planner, published, _drawn = _holding_planner()
    room = np.concatenate([_room(-0.28), np.array([[np.nan, 0.0, -0.08]], dtype=np.float32)])
    cloud = PointCloud2.from_numpy(room, frame_id="odom")
    planner._on_local_map(cloud)
    assert planner._plan_once(cloud, pose_stamped(0.0, 0.0, 0.0), (0.5, 0.0), ground_z=-0.28)
    assert len(published) == 1 and len(published[0].poses) >= 2


def test_the_default_model_is_the_body_band():
    planner = _model_planner()
    assert planner.config.obstacle_model == "body_band"


def _wall_over(ground_z: float, height: float) -> NDArray[np.float64]:
    """A wall across the route at `height` over the ground, and nothing else."""
    ys = np.arange(-1.0, 1.0 + 1e-9, 0.05)
    return np.column_stack([np.full(len(ys), 1.5), ys, np.full(len(ys), ground_z + height)]).astype(
        np.float32
    )


def _detour(emb: Embodiment, cloud: NDArray[np.float64], ground_z: float) -> float:
    """How far off the straight line the plan goes, planning the way the module does."""
    hard = hard_points(load_model("body_band", emb), cloud, ground_z)
    episode = make_py(emb)
    episode.reset()
    path = episode.plan(hard[:, :2], Pose(0.0, 0.0, 0.0), Pose(3.0, 0.0, 0.0))
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
    tall = replace(GO2, height=0.60)
    assert _detour(tall, wall, 0.0) > 0.8, "the tall body drove through its own obstacle"
    # and the control: the same wall IS over a go2's belly, so it is not a wall
    assert not len(hard_points(load_model("body_band", GO2), wall, 0.0))
    assert _detour(GO2, wall, 0.0) < 0.2


def test_native_twin_shares_the_python_defaults():
    from dimos.navigation.motion.adapter.planner_native import MotionPlannerNativeConfig

    native, py = MotionPlannerNativeConfig.model_fields, MotionPlannerConfig.model_fields
    shared = set(native) & set(py) - set(ModuleConfig.model_fields)
    assert {f: native[f].default for f in shared} == {f: py[f].default for f in shared}
