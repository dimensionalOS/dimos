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

import json
from unittest.mock import Mock

import mujoco
import numpy as np
import pytest
from dimos.robot.pollen.microduck.gait import MicroduckObserver
from dimos.robot.pollen.microduck.policies import PolicyBank
from dimos.robot.pollen.microduck.sim_module import MicroduckSimModuleConfig
from microduck_world.physics_robots import WorldRobots
from microduck_world.robot_io import RobotCommand
from microduck_world.roster import ROBOT_IDS


@pytest.fixture
def world(monkeypatch):
    spec = mujoco.MjSpec()
    for prefix in ("", *(id + "_" for id in ROBOT_IDS[1:])):
        robot = mujoco.MjSpec.from_string("""<mujoco><worldbody>
          <body name="trunk_base" pos="0 0 .2"><freejoint name="trunk_base_freejoint"/>
            <geom type="sphere" size=".03" mass="1"/><site name="imu"/>
            <body pos="0 0 .1"><joint name="knee"/>
              <geom type="sphere" size=".01" mass=".1"/>
            </body>
          </body>
        </worldbody><sensor><gyro name="imu_ang_vel" site="imu"/></sensor>
        <actuator><position joint="knee" kp="1"/></actuator></mujoco>""")
        spec.attach(robot, prefix=prefix, frame=spec.worldbody.add_frame())
    model = spec.compile()
    data = mujoco.MjData(model)
    banks = {}
    for prefix in ("", *(id + "_" for id in ROBOT_IDS[1:])):
        observer = MicroduckObserver(model, ["knee"], np.array([0.2]), prefix=prefix)
        bank = Mock(spec=PolicyBank)
        bank.joint_names = ["knee"]
        bank.default_pose = observer.default_pose
        bank.variant = "default"
        bank.availability = {"walk": None}
        bank.root_qpos_adr = observer.root_qpos_adr
        bank.initial_qpos.side_effect = observer.initial_qpos
        bank.projected_gravity.side_effect = observer.projected_gravity
        bank.root_yaw.side_effect = observer.root_yaw
        bank.step.return_value = observer.default_pose
        banks[prefix] = bank
    bank = banks[""]
    bank.for_robot.side_effect = lambda model, prefix: banks[prefix]
    clock = Mock(return_value=0.0)
    monkeypatch.setattr("microduck_world.physics_robots.time.monotonic", clock)
    robots = WorldRobots(
        model,
        bank,
        {
            "robots": {
                id: {"team": "red" if n < 3 else "blue", "spawn": [n, 0], "yaw": 0}
                for n, id in enumerate(ROBOT_IDS)
            },
            "clearance": 0.4,
        },
        MicroduckSimModuleConfig(auto_stand=False),
    )
    robots.leases({id: "visitor" + str(n + 1) for n, id in enumerate(ROBOT_IDS)})
    for _ in range(4):
        robots.step(data)
    return robots, data, clock


def test_fallen_duck_stays_fallen_without_automatic_pose_reset(world):
    robots, data, clock = world
    duck = robots.robots["duck2"]
    adr = duck.bank.root_qpos_adr
    data.qpos[adr + 2 : adr + 7] = (0.03, 0, 1, 0, 0)
    for _ in range(4):
        robots.step(data)
    clock.return_value = 2.5
    before = data.qpos.copy()
    robots.step(data)
    np.testing.assert_array_equal(data.qpos, before)
    assert duck.scheduler.fallen


def test_respawn_only_resets_own_duck_and_preserves_its_map_frame(world):
    robots, data, _ = world
    duck = robots.robots["duck2"]
    origin = duck.origin
    adr = duck.bank.root_qpos_adr
    data.qpos[adr : adr + 7] = (0.5, 0.6, 0.03, 0, 1, 0, 0)
    data.qvel[:] = 0.4
    before_pos, before_vel = data.qpos.copy(), data.qvel.copy()
    robots.command("duck2", RobotCommand("visitor2", "twist", (0.1, 0.0, 0.0)))
    robots.command("duck2", RobotCommand("visitor2", "respawn", ""))
    robots.step(data)

    np.testing.assert_allclose(data.qpos[adr : adr + 7], [1, 0, 0.125, 1, 0, 0, 0])
    for other_id in ("duck1", "duck3"):
        other = robots.robots[other_id]
        root = other.bank.root_qpos_adr
        pos = list(range(root, root + 7)) + other.joint_qpos
        vel_root = int(
            robots.model.joint(
                ("" if other_id == "duck1" else other_id + "_") + "trunk_base_freejoint"
            ).dofadr[0]
        )
        vel = list(range(vel_root, vel_root + 6)) + other.joint_qvel
        np.testing.assert_array_equal(data.qpos[pos], before_pos[pos])
        np.testing.assert_array_equal(data.qvel[vel], before_vel[vel])
    assert duck.origin == origin
    assert duck.generation == "visitor2"
    assert json.loads(robots.states(data)["duck2"].policy)["respawns"] == 1
    assert "duck2" not in robots._commands


def test_previous_visitor_cannot_respawn_current_duck(world):
    robots, data, _ = world
    before = data.qpos.copy()
    robots.command("duck2", RobotCommand("previous-visitor", "respawn", ""))
    robots.step(data)
    np.testing.assert_array_equal(data.qpos, before)
    assert robots.robots["duck2"].respawns == 0


def test_all_six_have_private_state_and_no_permanent_host(world):
    robots, data, _ = world
    assert set(robots.states(data)) == set(ROBOT_IDS)
    robots.leases({})
    robots.step(data)
    assert robots.states(data) == {}


def test_respawn_cannot_use_the_other_teams_available_spawns(world):
    robots, data, _ = world
    red = robots.robots["duck2"]
    # Block all red bays using active robots, leaving blue bays irrelevant.
    for id, x in [("duck4", 0), ("duck5", 1), ("duck6", 2)]:
        data.qpos[
            robots.robots[id].bank.root_qpos_adr : robots.robots[id].bank.root_qpos_adr + 2
        ] = [x, 0]
    root = red.bank.root_qpos_adr
    data.qpos[root : root + 2] = [8, 0]
    robots.command("duck2", RobotCommand("visitor2", "respawn", ""))
    robots.step(data)
    assert red.respawns == 0
    assert "duck2" in robots._respawns
    assert data.qpos[root] == 8
    # Free its own bay and confirm that the pending request succeeds there.
    data.qpos[robots.robots["duck5"].bank.root_qpos_adr] = 20
    robots.step(data)
    assert red.respawns == 1
    assert data.qpos[root] == 1
