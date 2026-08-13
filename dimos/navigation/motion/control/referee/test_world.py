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

import mujoco
import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped as RefereePose
from dimos.msgs.nav_msgs.Path import Path, Path as RefereePath
from dimos.navigation.motion.control.referee import world
from dimos.navigation.motion.scenarios import SCENARIOS, Scenario
from dimos.navigation.motion.simulation.evaluate import FITTED_PHYSICS

CORRIDOR = next(s for s in SCENARIOS if s.name == "corridor")
DOOR_SIDE = next(s for s in SCENARIOS if s.name == "door_side")


def _standing_pose(model: mujoco.MjModel) -> np.ndarray:
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    return np.array(model.key_qpos[kid][7:19])


def test_walls_match_scenario_geometry() -> None:
    model, data = world.load_world(CORRIDOR)
    walls = world.wall_geom_ids(model)
    assert len(walls) == len(CORRIDOR.boxes)
    for i, b in enumerate(CORRIDOR.boxes):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"wall_{i}")
        assert gid in walls
        np.testing.assert_allclose(
            model.geom_size[gid], [b.sx / 2, b.sy / 2, b.height / 2], atol=1e-9
        )
        bid = model.geom_bodyid[gid]
        np.testing.assert_allclose(model.body_pos[bid][:2], [b.cx, b.cy], atol=1e-9)


def test_spawn_at_start_no_contact() -> None:
    model, data = world.load_world(DOOR_SIDE)
    world.reset_to_start(model, data, DOOR_SIDE, _standing_pose(model))
    np.testing.assert_allclose(data.qpos[0:2], DOOR_SIDE.start[:2], atol=1e-9)
    # recovered yaw
    w, z = data.qpos[3], data.qpos[6]
    assert math.isclose(2 * math.atan2(z, w), DOOR_SIDE.start[2], abs_tol=1e-9)
    mujoco.mj_step(model, data)
    assert not world.wall_contact(model, data, world.wall_geom_ids(model))


def test_teleport_into_wall_contacts() -> None:
    sc = CORRIDOR
    model, data = world.load_world(sc)
    world.reset_to_start(model, data, sc, _standing_pose(model))
    b = sc.boxes[0]
    data.qpos[0:2] = (b.cx, b.cy)  # trunk inside the wall
    mujoco.mj_forward(model, data)
    assert world.wall_contact(model, data, world.wall_geom_ids(model))


def test_fitted_physics_applies() -> None:
    stock, _ = world.load_world(CORRIDOR)
    fitted, _ = world.load_world(CORRIDOR, physics=FITTED_PHYSICS)
    assert not np.allclose(stock.dof_damping, fitted.dof_damping)


def test_planner_cloud_matches_referee_sampling() -> None:
    cloud = world.planner_cloud(CORRIDOR)
    expected = sum(len(b.surface(world.CLOUD_STEP)) for b in CORRIDOR.boxes)
    assert len(cloud) == expected
    assert world.planner_cloud(Scenario("empty", [], goal=(1.0, 0.0))).points_f32().shape == (0, 3)


def test_to_nav_path_converts_poses() -> None:
    ref = RefereePath(
        frame_id="world",
        poses=[RefereePose(frame_id="world", position=[float(i), 0.5, 0.0]) for i in range(4)],
    )
    out = world.to_nav_path(ref, ts=12.0, frame_id="odom")
    assert isinstance(out, Path)
    assert out.frame_id == "odom"
    assert len(out) == 4
    assert out.poses[3].position.x == pytest.approx(3.0)
    assert out.poses[0].ts == pytest.approx(12.0)
