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
from typing import Any

import mujoco
import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import shared_config
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.sim2.actions import (
    Attach,
    Detach,
    Interact,
    JointCommand,
    SetPose,
    SetVelocity,
    Spawn,
)
from dimos.sim2.module import Simulation, SimulationConfig
from dimos.sim2.mujoco_backend import MujocoBackend
from dimos.sim2.spec import Mount, Servo, SimulatedRobot, SimulatedSensor
from dimos.sim2.world import compile_world
from dimos.sim2.world_change import EVENTS, OBJECT, WorldMirror


@pytest.fixture
def scene(tmp_path: Path) -> Path:
    path = tmp_path / "scene.xml"
    path.write_text("""
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 .1"/>
    <body name="shelf" pos="2 0 1"/>
    <body name="crate" pos="0 2 .5"><freejoint/><geom type="box" size=".1 .1 .1"/></body>
  </worldbody>
</mujoco>
""")
    return path


@pytest.fixture
def robot_model(tmp_path: Path) -> Path:
    path = tmp_path / "robot.xml"
    path.write_text("""
<mujoco>
  <worldbody>
    <body name="base" pos="9 9 9">
      <freejoint/>
      <geom type="box" size=".2 .1 .1" mass="1"/>
      <body name="head" pos=".2 0 .1">
        <joint name="neck" type="hinge" axis="0 0 1"/>
        <geom type="sphere" size=".05" mass=".1"/>
      </body>
    </body>
  </worldbody>
  <actuator><motor name="neck" joint="neck"/></actuator>
  <keyframe><key name="home" qpos="0 0 0 1 0 0 0 0"/></keyframe>
</mujoco>
""")
    return path


def test_robots_and_sensors_compile_into_one_world(scene, robot_model):
    model = compile_world(
        scene,
        [
            SimulatedRobot("left", robot_model, "base", xyz=(-1, 0, 0.3)),
            SimulatedRobot("right", robot_model, "base", xyz=(1, 0, 0.3)),
            SimulatedSensor("camera", "camera", Mount("head"), robot_id="left"),
            SimulatedSensor("lidar", "lidar", Mount("base", xyz=(0, 0, 0.2)), robot_id="right"),
            SimulatedSensor("overhead", "camera", Mount("world", xyz=(0, 0, 3))),
        ],
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.body("left/base").xpos.tolist() == pytest.approx([-1, 0, 0.3])
    assert data.body("right/base").xpos.tolist() == pytest.approx([1, 0, 0.3])
    assert model.joint("right/neck") is not None
    assert model.camera("left/camera") is not None
    assert model.camera("world/overhead") is not None
    assert data.site("right/lidar").xpos.tolist() == pytest.approx([1, 0, 0.5])


def test_sensor_on_a_missing_body_is_rejected(scene):
    with pytest.raises(ValueError, match="no body 'nowhere'"):
        compile_world(scene, [SimulatedSensor("imu", "imu", Mount("nowhere"))])


class _DogConfig(ModuleConfig):
    model: Path


class _Dog(Module):
    config: _DogConfig

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        return {"simulated": SimulatedRobot("dog", self.config.model, "base")}


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
        self.events: list[tuple[int, int, Any]] = []

    def offer(self, change: Any) -> None:
        self.world.apply(change)
        self.frames.append(change)
        self.events += self.world.events


def test_backend_steps_physics_and_takes_commands(scene, robot_model, tmp_path):
    backend = MujocoBackend(
        SimulationConfig(scene=scene),
        {"dog": SimulatedRobot("dog", robot_model, "base", xyz=(0, 0, 1))},
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
    # It fell from z=1 onto the floor.
    assert world.entity("dog").pose[2] < 0.2
    names, joints = world.joints("dog")
    assert names == ["neck"]
    assert joints[0, 1] > 1.0


def test_frames_report_robots_and_objects_and_actions_drive_them(scene, robot_model, tmp_path):
    backend = MujocoBackend(
        SimulationConfig(scene=scene), {"dog": SimulatedRobot("dog", robot_model, "base")}, tmp_path
    )
    backend.apply(
        [
            SetPose("crate", Pose(1, 2, 3)),
            JointCommand("dog", ("neck",), effort=(0.5,)),
        ]
    )
    backend._apply_pending()
    seen = _Collect()
    seen.offer(backend.frame())
    assert [e.path for e in seen.world.tagged("robot")] == ["dog"]
    assert [e.path for e in seen.world.tagged(OBJECT)] == ["crate"]
    assert seen.world.joints("dog")[0] == ["neck"]
    assert seen.world.entity("crate").pose[0] == pytest.approx(1)
    assert backend.data.ctrl[backend.model.actuator("dog/neck").id] == 0.5


def test_scene_data_describes_the_world_without_its_robots(scene, robot_model, tmp_path):
    backend = MujocoBackend(
        SimulationConfig(scene=scene), {"dog": SimulatedRobot("dog", robot_model, "base")}, tmp_path
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


def test_set_velocity_drives_a_base_and_spawning_is_refused(scene, tmp_path):
    backend = MujocoBackend(SimulationConfig(scene=scene), {}, tmp_path)
    backend.apply(
        [
            SetVelocity("crate", Twist(linear=[1.0, 0.0, 0.0])),
            Spawn("box", "box", (0.1, 0.1, 0.1), Pose()),
        ]
    )
    published = _Collect()

    async def run_briefly() -> None:
        running = asyncio.create_task(backend.run(published))
        await asyncio.sleep(0.5)
        running.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await running

    asyncio.run(run_briefly())
    crate = published.world.entity("crate")
    # Held at 1 m/s along x from x=0 while gravity settles it onto the floor.
    assert crate.pose[0] == pytest.approx(published.world.time, abs=0.05)
    assert crate.pose[2] < 0.15
    assert [e.path for e in published.world.tagged(OBJECT)] == ["crate"]
    # The world could not spawn the box, and said so.
    refused = [
        cbor["reason"] for _, event, cbor in published.events if event == EVENTS["events/refused"]
    ]
    assert any("Spawn" in reason for reason in refused)


def test_a_motor_command_drives_a_servo_with_its_own_gains(scene, robot_model, tmp_path):
    robot = SimulatedRobot("dog", robot_model, "base", servos=(Servo("neck", 0.01, 0.001),))
    backend = MujocoBackend(SimulationConfig(scene=scene), {"dog": robot}, tmp_path)
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
    robot = SimulatedRobot("dog", robot_model, "base", control_period=0.02)
    backend = MujocoBackend(SimulationConfig(scene=scene), {"dog": robot}, tmp_path)
    tick_times: list[float] = []

    class _Controller(_Collect):
        """Acts only through actions, on each frame that waits for it, as a robot module does."""

        def offer(self, change: Any) -> None:
            super().offer(change)
            if self.world.has_event("dog", "events/act"):
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


def test_interact_runs_an_entitys_keyframed_action_and_attach_carries_an_object(
    tmp_path, robot_model
):
    scene = tmp_path / "rooms.xml"
    scene.write_text("""
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 .1"/>
    <body name="door" pos="2 0 1"><joint name="hinge" type="hinge" axis="0 0 1"/><geom type="box" size=".05 .5 1"/></body>
    <body name="cup" pos="0 0 1.2"><freejoint/><geom type="sphere" size=".05" mass=".1"/></body>
  </worldbody>
  <keyframe><key name="door/open" qpos="1.5 0 0 1.2 1 0 0 0"/></keyframe>
</mujoco>
""")
    dog = SimulatedRobot("dog", robot_model, "base", xyz=(0, 0, 1))
    backend = MujocoBackend(SimulationConfig(scene=scene), {"dog": dog}, tmp_path)
    backend.apply(
        [Interact("door", "open"), Attach("cup", "dog"), SetPose("dog", Pose(1.0, 1.0, 1.0))]
    )
    backend._apply_pending()
    backend._hold_carried()
    mujoco.mj_forward(backend.model, backend.data)
    seen = _Collect()
    seen.offer(backend.frame())
    assert seen.world.joints("door")[1][:, 0] == pytest.approx([1.5])
    # Carried 0.2 above the dog's root, wherever the dog goes.
    assert seen.world.entity("cup").pose[0] == pytest.approx(1.0)
    assert seen.world.entity("cup").pose[2] == pytest.approx(1.2)
    backend.apply([Detach("cup"), SetPose("dog", Pose(0.0, -1.0, 1.0)), Interact("door", "slam")])
    backend._apply_pending()
    backend._hold_carried()
    mujoco.mj_forward(backend.model, backend.data)
    seen.offer(backend.frame())
    assert seen.world.entity("cup").pose[0] == pytest.approx(1.0)
