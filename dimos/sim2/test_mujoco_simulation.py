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

import asyncio
import contextlib
import json
from pathlib import Path
import time
from typing import Any

import mujoco
import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import shared_config
from dimos.robot.assets.model import UsdModel
from dimos.sim2.actions import CborKeyValueCommand, JointCommand, read_actions, world_action
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.module import Simulation, SimulationConfig
from dimos.sim2.mujoco_backend import MujocoBackend
from dimos.sim2.spec import Servo, SimulatedRobot, SimulatedSensor, resolve
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world import compile_world
from dimos.sim2.world_observations import ACT, WorldMirror


@pytest.fixture
def scene(tmp_path: Path) -> Path:
    path = tmp_path / "scene.xml"
    path.write_text("""
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 .1"/>
    <body name="shelf" pos="2 0 1"/>
    <site name="overhead" pos="0 0 3"/>
    <body name="crate" pos="0 2 .5"><freejoint/><geom type="box" size=".1 .1 .1"/></body>
  </worldbody>
</mujoco>
""")
    return path


@pytest.fixture
def robot_model() -> UsdModel:
    return toy_robot()


def test_robots_and_sensors_compile_into_one_world(scene, robot_model, tmp_path):
    # Two dogs, one with an eye on its head, the other a lidar 20 cm above its base; and a camera over the scene.
    left = robot_model.with_frames({"eye": ("head", (0, 0, 0), (0, 0, 0))})
    right = robot_model.with_frames({"lidar": ("base", (0, 0, 0.2), (0, 0, 0))})
    model = compile_world(
        scene,
        resolve(
            [
                SimulatedRobot("left", left, xyz=(-1, 0, 0.3)),
                SimulatedRobot("right", right, xyz=(1, 0, 0.3)),
                SimulatedSensor("camera", "eye"),
                SimulatedSensor("lidar", "lidar"),
                SimulatedSensor("camera", "overhead"),
            ]
        ),
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.body("left/base").xpos.tolist() == pytest.approx([-1, 0, 0.3])
    assert data.body("right/base").xpos.tolist() == pytest.approx([1, 0, 0.3])
    assert model.joint("right/neck") is not None
    assert model.camera("left/eye") is not None
    assert data.cam("world/overhead").xpos.tolist() == pytest.approx([0, 0, 3])
    assert data.site("right/lidar").xpos.tolist() == pytest.approx([1, 0, 0.5])


def test_sensor_at_a_missing_frame_is_rejected(scene):
    with pytest.raises(ValueError, match="no site or body 'nowhere'"):
        compile_world(scene, [SimulatedSensor("imu", "nowhere")])


class _DogConfig(ModuleConfig):
    model: UsdModel


class _Dog(Module):
    config: _DogConfig

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        return {"simulated": SimulatedRobot("dog", self.config.model)}


def test_simulation_builds_the_world_its_blueprint_describes(scene, robot_model):
    blueprint = autoconnect(_Dog.blueprint(model=robot_model), Simulation.blueprint(scene=scene))
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        description = coordinator.get_instance(Simulation).describe()
        assert description["robots"] == ["dog"]
        assert description["scene"] == str(scene)
        assert [d.robot_id for d in description["simulated"]] == ["dog"]
    finally:
        coordinator.stop()


class _Collect:
    """A module's view of the world: every frame offered, applied to its mirror."""

    busy = False

    def __init__(self) -> None:
        self.world = WorldMirror()
        self.frames: list[Any] = []

    def offer(self, frame: Any) -> None:
        self.world.apply(frame)
        self.frames.append(frame)


def test_backend_steps_physics_and_takes_commands(scene, robot_model, tmp_path):
    backend = MujocoBackend(
        SimulationConfig(scene=scene),
        scene,
        {"dog": SimulatedRobot("dog", robot_model, xyz=(0, 0, 1))},
        tmp_path,
    )
    published = _Collect()

    async def run_briefly() -> None:
        running = asyncio.create_task(backend.run(published))
        await asyncio.sleep(0.05)
        backend.apply([JointCommand("dog", ("neck",), effort=(1.0,))])
        await asyncio.sleep(0.5)
        running.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await running

    asyncio.run(run_briefly())
    world = published.world
    assert world.time > 0.3
    # It fell from z=1 onto the floor. Not nearer than 0.3, because it comes to rest tilted on
    # its head as well as its box, and how high that leaves the base is the toy's shape rather
    # than anything this test is about.
    assert world.entity("dog").pose[2] < 0.3
    names, joints = world.joints("dog")
    assert names == ["neck"]
    assert joints[0, 1] > 1.0
    # A body's velocity is differenced from its poses.
    assert world.entity("dog").velocity is not None


def test_frames_report_robots_and_objects_and_actions_drive_them(scene, robot_model, tmp_path):
    backend = MujocoBackend(
        SimulationConfig(scene=scene),
        scene,
        {"dog": SimulatedRobot("dog", robot_model)},
        tmp_path,
    )
    backend.apply([JointCommand("dog", ("neck",), effort=(0.5,))])
    backend._apply_pending()
    seen = _Collect()
    seen.offer(backend.frame())
    assert seen.world.entity("dog").cbor["model"] == str(usd_layer(robot_model))
    assert seen.world.joints("dog")[0] == ["neck"]
    assert seen.world.entity("crate").pose[:3] == pytest.approx([0, 2, 0.5])
    # Its ts is the wall time the sim started at, plus sim time.
    assert seen.world.time == pytest.approx(time.time(), abs=5.0)
    assert backend.data.ctrl[backend.model.actuator("dog/neck").id] == 0.5


def test_actions_cross_the_wire_by_id_and_cbor_keys_are_observed_back(scene, robot_model, tmp_path):
    backend = MujocoBackend(
        SimulationConfig(scene=scene),
        scene,
        {"dog": SimulatedRobot("dog", robot_model)},
        tmp_path,
    )
    seen = _Collect()
    seen.offer(backend.frame())
    actions = [
        JointCommand("dog", ("neck",), position=(0.2,), kp=(10.0,), kd=(1.0,)),
        CborKeyValueCommand("dog", "note", {"a": 1}),
    ]
    assert read_actions(world_action(seen.world, *actions), backend.world) == actions
    backend.apply(actions)
    backend._apply_pending()
    seen.offer(backend.frame())
    assert seen.world.entity("dog").cbor["note"] == {"a": 1}


def test_scene_data_describes_the_world_without_its_robots(scene, robot_model, tmp_path):
    backend = MujocoBackend(
        SimulationConfig(scene=scene),
        scene,
        {"dog": SimulatedRobot("dog", robot_model)},
        tmp_path,
    )
    data = json.loads(backend.scene_data())
    assert data["format"] == "dimos.scene_data.v1" and data["frame"] == "world"
    bodies = {body["name"]: body for body in data["bodies"]}
    assert set(bodies) == {"world", "crate"}
    assert [geom["type"] for geom in bodies["world"]["geoms"]] == ["plane"]
    crate = bodies["crate"]
    assert crate["pos"] == pytest.approx([0, 2, 0.5])
    assert crate["geoms"][0]["type"] == "box" and crate["geoms"][0]["size"] == pytest.approx(
        [0.1] * 3
    )


def test_a_motor_command_drives_a_servo_with_its_own_gains(scene, robot_model, tmp_path):
    robot = SimulatedRobot("dog", robot_model, servos=(Servo("neck", 0.01, 0.001),))
    backend = MujocoBackend(SimulationConfig(scene=scene), scene, {"dog": robot}, tmp_path)
    published = _Collect()

    async def run_briefly() -> None:
        running = asyncio.create_task(backend.run(published))
        await asyncio.sleep(0.05)
        backend.apply([JointCommand("dog", ("neck",), position=(0.5,), kp=(0.05,), kd=(0.005,))])
        await asyncio.sleep(1.0)
        running.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await running

    asyncio.run(run_briefly())
    actuator = backend.model.actuator("dog/neck").id
    assert backend.model.actuator_gainprm[actuator, 0] == 0.05
    assert tuple(backend.model.actuator_biasprm[actuator, 1:3]) == (-0.05, -0.005)
    assert published.world.joints("dog")[1][0, 0] == pytest.approx(0.5, abs=0.02)


def test_world_holds_at_each_control_tick_for_its_controller(scene, robot_model, tmp_path):
    robot = SimulatedRobot("dog", robot_model, control_period=0.02)
    backend = MujocoBackend(SimulationConfig(scene=scene), scene, {"dog": robot}, tmp_path)
    tick_times: list[float] = []

    class _Controller(_Collect):
        """Acts only through actions, on each frame that waits for it, as a robot module does."""

        def offer(self, frame: Any) -> None:
            super().offer(frame)
            if self.world.got("dog", ACT):
                tick_times.append(self.world.time)
                asyncio.get_running_loop().call_later(
                    0.03, backend.apply, [JointCommand("dog", ("neck",), effort=(0.1,))]
                )

    async def run_briefly() -> None:
        running = asyncio.create_task(backend.run(_Controller()))
        await asyncio.sleep(0.6)
        running.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await running

    asyncio.run(run_briefly())
    # It is not waited for until it has first acted; from then on, though slow, it sees every tick.
    assert len(tick_times) > 5
    assert np.diff(tick_times[1:]) == pytest.approx(0.02, abs=1e-6)
    assert backend.data.ctrl[backend.model.actuator("dog/neck").id] == 0.1
