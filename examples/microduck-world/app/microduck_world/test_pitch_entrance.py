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

"""Floor support and real-policy walking across the locker-to-pitch threshold."""

import math
from copy import copy, deepcopy

import mujoco
import numpy as np
import pytest
from dimos.robot.pollen.microduck.policies import PolicyBank
from microduck_world.physics_robots import WorldRobots
from microduck_world.robot_io import RobotCommand
from microduck_world.roster import ROBOT_IDS, SETTINGS
from microduck_world.scene import PROJECT_ROOT, load_world
from microduck_world.world_sim import WorldSimModule


def test_pitch_entrance_has_continuous_level_floor_support():
    model = mujoco.MjModel.from_xml_path(str(load_world()[0].mujoco_scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    club, pitch = model.geom("club_floor"), model.geom("football_floor")
    north = data.geom_xpos[club.id, 1] + club.size[1]
    south = data.geom_xpos[pitch.id, 1] - pitch.size[1]
    assert north == pytest.approx(south, abs=1e-10)
    for x in np.linspace(-0.5, 0.5, 5):
        for y in np.linspace(1.95, 2.2, 51):
            geom = np.zeros(1, dtype=np.int32)
            distance = mujoco.mj_ray(
                model,
                data,
                np.array([x, y, 0.2]),
                np.array([0.0, 0.0, -1.0]),
                np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8),
                1,
                -1,
                geom,
            )
            assert distance == pytest.approx(0.2, abs=1e-10), (x, y, distance)
            assert int(geom[0]) in (club.id, pitch.id)


@pytest.fixture(scope="module")
def entrance_world():
    module = WorldSimModule(
        scene_xml=load_world()[0].mujoco_scene_path,
        robot_mjcf=str(PROJECT_ROOT / "assets/microduck/robot/robot_allcollisions.xml"),
        headless=True,
        auto_stand=False,
    )
    model = module._compose_spec().compile()
    bank = PolicyBank(PROJECT_ROOT / "assets/microduck/policies", model)
    return model, bank, module.config


@pytest.mark.parametrize("robot_id", ROBOT_IDS)
@pytest.mark.parametrize("direction", [1, -1])
def test_real_walking_policy_crosses_pitch_threshold_without_falling(
    entrance_world, robot_id, direction
):
    source, bank, config = entrance_world
    model = copy(source)
    data = mujoco.MjData(model)
    settings = deepcopy(SETTINGS)
    index = ROBOT_IDS.index(robot_id)
    x = (-0.3, 0, 0.3)[index % 3]
    settings["robots"][robot_id].update(
        spawn=[x, 1.4 if direction == 1 else 2.7],
        yaw=direction * math.pi / 2,
    )
    robots = WorldRobots(model, bank, settings, config)
    robot = robots.robots[robot_id]
    adr = robot.bank.root_qpos_adr
    # Use the normal walk command, on both sides and the centre of the doorway.
    # Smaller requests can select the scheduler's standing mode.
    speed = 0.15
    crossed_at = None
    for step in range(4400):
        if step % 20 == 0:
            robots.leases({robot_id: "entrance-test"})
            moving = step >= 400 and crossed_at is None
            robots.command(
                robot_id,
                RobotCommand("entrance-test", "twist", (speed if moving else 0, 0, 0)),
            )
        robots.step(data)
        mujoco.mj_step(model, data)
        if step >= 400:
            assert robot.bank.projected_gravity(data)[2] < -0.9, (
                robot_id,
                direction,
                data.time,
                data.qpos[adr : adr + 3],
            )
            assert data.qpos[adr + 2] > 0.09
        y = data.qpos[adr + 1]
        if crossed_at is None and (y > 2.45 if direction == 1 else y < 1.7):
            crossed_at = step
        if crossed_at is not None and step >= crossed_at + 200:
            break  # Remain upright for one second after stopping on the other side.
    assert crossed_at is not None, (robot_id, direction, data.qpos[adr : adr + 3])


def test_all_six_midfield_spawns_activate_and_remain_upright(entrance_world):
    source, bank, config = entrance_world
    model = copy(source)
    data = mujoco.MjData(model)
    robots = WorldRobots(model, bank, deepcopy(SETTINGS), config)
    leases = {robot_id: f"spawn-{robot_id}" for robot_id in ROBOT_IDS}
    for step in range(600):
        if step % 20 == 0:
            robots.leases(leases)
        robots.step(data)
        mujoco.mj_step(model, data)
    for robot_id in ROBOT_IDS:
        robot = robots.robots[robot_id]
        assert robot.active, robot_id
        adr = robot.bank.root_qpos_adr
        x, y, z = data.qpos[adr : adr + 3]
        expected_x, expected_y = SETTINGS["robots"][robot_id]["spawn"]
        assert abs(x - expected_x) < 0.1 and abs(y - expected_y) < 0.1, (robot_id, x, y)
        assert z > 0.09, (robot_id, z)
        assert robot.bank.projected_gravity(data)[2] < -0.9, robot_id
    # Use the same normal spawn path again for an explicit respawn.
    robots.command("duck1", RobotCommand(leases["duck1"], "respawn", ""))
    robots.step(data)
    assert robots.robots["duck1"].respawns == 1
