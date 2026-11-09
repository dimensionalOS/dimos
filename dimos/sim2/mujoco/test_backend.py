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
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd import graft_usd, prefix_usd_frames
from dimos.sim2.actions import CborKeyValueCommand, JointCommand, read_actions, world_action
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.mujoco.backend import MujocoBackend
from dimos.sim2.mujoco.sim import MujocoSim
from dimos.sim2.mujoco.world import SensorMount, compile_world, mount_sensor
from dimos.sim2.sim import sim_module
from dimos.sim2.spec import Servo, SimulatedRobot
from dimos.sim2.usd_layer import layer_file, usd_layer
from dimos.sim2.world_observations import ACT, WorldMirror

SCENE = Path(__file__).parent / "assets" / "scene.usda"


@pytest.fixture
def scene() -> UsdModel:
    """The room these tests run in: a floor, a shelf, a crate on a free joint, and a mount above.

    USD, like everything else a world is made of, and converted once from the MJCF it used to be
    written as -- `assets/scene.usda`, beside this file. `overhead` is added as a *frame* rather
    than being in the layer, which is how a scene says where something can be mounted: the room
    holds the spot, and whatever is bolted there says what it is.
    """
    return UsdModel.from_file(SCENE).with_frames(
        {"overhead": ("scene", (0.0, 0.0, 3.0), (0.0, 0.0, 0.0))}
    )


@pytest.fixture
def robot_model() -> UsdModel:
    return toy_robot()


def test_robots_compile_into_one_world(scene, robot_model):
    """Two dogs in one room, each where its description puts it -- and no sensors in it.

    The world the physics runs holds robots and a room. A sensor is not something the simulation
    was told about, so there is nothing here to put a camera or a site in it.
    """
    left = robot_model.with_frames({"eye": ("head", (0, 0, 0), (0, 0, 0))})
    right = robot_model.with_frames({"lidar": ("base", (0, 0, 0.2), (0, 0, 0))})
    model = compile_world(
        usd_layer(scene),
        [
            SimulatedRobot("left", left, xyz=(-1, 0, 0.3)),
            SimulatedRobot("right", right, xyz=(1, 0, 0.3)),
        ],
    ).compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.body("left/base").xpos.tolist() == pytest.approx([-1, 0, 0.3])
    assert data.body("right/base").xpos.tolist() == pytest.approx([1, 0, 0.3])
    assert model.joint("right/neck") is not None
    assert model.ncam == 0, "the physics world has no cameras; a camera renders its own copy"


def test_a_sensor_mounts_itself_in_a_world_of_its_own(scene, robot_model):
    """The same scene, compiled again by a sensor, with what that sensor needs added to it.

    A camera's field of view and a lidar's site belong to the sensor that reads them, so they go
    into the copy it renders from. The mount frames are already there: a robot's model carries
    them, and a composed scene carries the room's.
    """
    left = robot_model.with_frames({"eye": ("head", (0, 0, 0), (0, 0, 0))})
    spec = compile_world(usd_layer(scene), [SimulatedRobot("left", left, xyz=(-1, 0, 0.3))])
    mount_sensor(spec, SensorMount("camera", "left/eye", "left/eye", fovy=42.0))
    mount_sensor(spec, SensorMount("camera", "overhead", "world/overhead"))
    mount_sensor(spec, SensorMount("imu", "left/base", "left/imu"))
    model = spec.compile()

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.camera("left/eye") is not None
    assert model.cam_fovy[model.camera("left/eye").id] == 42.0
    assert data.cam("world/overhead").xpos.tolist() == pytest.approx([0, 0, 3])
    # An IMU is a site plus the three sensors that read it.
    assert data.site("left/imu").xpos.tolist() == pytest.approx([-1, 0, 0.3])
    assert {model.sensor(i).name for i in range(model.nsensor)} == {
        "left/imu/gyro",
        "left/imu/accelerometer",
        "left/imu/orientation",
    }


def test_a_robot_grafted_into_a_scene_is_still_free_to_move(scene, robot_model):
    """A composed world is how a robot really arrives: grafted onto a frame the room holds.

    USD says "this subtree floats on its mount" with a generic six-degree joint, and MuJoCo's USD
    reader does not read that one. A nested body with no joint is welded, so every robot in a
    composed world was bolted to the room: it stood, it never fell over, and it never walked,
    with nothing in the log but one "Unsupported joint type" warning.
    """
    composed = graft_usd(
        scene.with_frames({"dog": ("scene", (0.0, 0.0, 1.0), (0.0, 0.0, 0.0))}).usd,
        {"dog": prefix_usd_frames(robot_model.usd, "dog")},
    )
    model = compile_world(layer_file(composed, called="composed"), ids=["dog"]).compile()

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    started = data.body("dog/base").xpos[2]
    for _ in range(300):
        mujoco.mj_step(model, data)

    assert data.body("dog/base").xpos[2] < started - 0.5, "the dog is welded to the room"
    # The frame holds the joint, since MuJoCo takes a free joint only on a top-level body, and
    # a mount frame with nothing grafted onto it is still a stand that does not move.
    assert data.body("/scene/overhead").xpos.tolist() == pytest.approx([0, 0, 3])


def test_mounting_at_a_frame_the_world_does_not_have_is_rejected(scene):
    spec = compile_world(usd_layer(scene))
    with pytest.raises(ValueError, match="no site or body 'nowhere'"):
        mount_sensor(spec, SensorMount("imu", "nowhere", "nowhere"))


def test_the_engine_runs_the_robots_the_blueprint_put_in_the_world(scene, robot_model):
    """Through `sim_module`, which names neither MuJoCo nor anything that reaches into it.

    The robots are config, set where the blueprint already knows them, so a robot module needs
    no way to find whatever engine is running and the engine needs no way to hear from anything.
    """
    blueprint = sim_module(scene=scene, robots=[SimulatedRobot("dog", robot_model)])
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        sim = coordinator.get_instance(MujocoSim)
        assert sim.describe()["robots"] == ["dog"]
        assert [d.robot_id for d in sim.describe()["simulated"]] == ["dog"]
    finally:
        coordinator.stop()


def test_the_world_the_engine_runs_is_the_scene_it_was_given(scene, robot_model):
    """And it is built when the module runs, not when it is built.

    Building it means asking a manager for the scene, and a manager that has not been
    constructed yet cannot be asked -- which it had not, while the modules were still being
    built.
    """
    sim = MujocoSim(scene=scene, robots=[SimulatedRobot("dog", robot_model)])
    try:
        assert sim._engine().describe() == {"backend": "mujoco", "scene": str(usd_layer(scene))}
    finally:
        sim.stop()


class _Collect:
    """A module's view of the world: every frame offered, applied to its mirror."""

    busy = False

    def __init__(self) -> None:
        self.world = WorldMirror()
        self.frames: list[Any] = []

    def offer(self, frame: Any) -> None:
        self.world.apply(frame)
        self.frames.append(frame)


@contextlib.asynccontextmanager
async def _looping(backend: MujocoBackend, published: _Collect) -> Any:
    """The module's own loop, running this engine and publishing into `published`.

    The engine does not loop: `step` advances the world once and hands back a frame, and pacing
    it, publishing it and waiting at a controller's tick is the module's work. So a test about
    what a running world does runs the real `MujocoSim._run`, with its publisher swapped for
    something that keeps what it was given.
    """
    sim = MujocoSim()
    # The engine the module would have built for itself: these tests are about what the loop
    # does with one, and each wants a world of its own making.
    sim._engine = lambda: backend  # type: ignore[method-assign]
    sim._publisher = published
    sim._all_started.set()
    running = asyncio.create_task(sim._run())
    try:
        yield sim
    finally:
        running.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await running


def test_backend_steps_physics_and_takes_commands(scene, robot_model):
    backend = MujocoBackend(scene, [SimulatedRobot("dog", robot_model, xyz=(0, 0, 1))])
    published = _Collect()

    async def run_briefly() -> None:
        async with _looping(backend, published):
            await asyncio.sleep(0.05)
            backend.apply([JointCommand("dog", ("neck",), effort=(1.0,))])
            await asyncio.sleep(0.5)

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


def test_frames_report_robots_and_objects_and_actions_drive_them(scene, robot_model):
    backend = MujocoBackend(scene, [SimulatedRobot("dog", robot_model)])
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


def test_actions_cross_the_wire_by_id_and_cbor_keys_are_observed_back(scene, robot_model):
    backend = MujocoBackend(scene, [SimulatedRobot("dog", robot_model)])
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


def test_scene_data_describes_the_world_without_its_robots(scene, robot_model):
    backend = MujocoBackend(scene, [SimulatedRobot("dog", robot_model)])
    data = json.loads(backend.scene_data())
    assert data["format"] == "dimos.scene_data.v1" and data["frame"] == "world"
    bodies = {body["name"]: body for body in data["bodies"]}
    assert set(bodies) == {"world", "shelf", "crate"}
    assert [geom["type"] for geom in bodies["world"]["geoms"]] == ["plane"]
    crate = bodies["crate"]
    assert crate["pos"] == pytest.approx([0, 2, 0.5])
    assert crate["geoms"][0]["type"] == "box" and crate["geoms"][0]["size"] == pytest.approx(
        [0.1] * 3
    )


def test_a_motor_command_drives_a_servo_with_its_own_gains(scene, robot_model):
    robot = SimulatedRobot("dog", robot_model, servos=(Servo("neck", 0.01, 0.001),))
    backend = MujocoBackend(scene, [robot])
    published = _Collect()

    async def run_briefly() -> None:
        async with _looping(backend, published):
            await asyncio.sleep(0.05)
            backend.apply(
                [JointCommand("dog", ("neck",), position=(0.5,), kp=(0.05,), kd=(0.005,))]
            )
            await asyncio.sleep(1.0)

    asyncio.run(run_briefly())
    actuator = backend.model.actuator("dog/neck").id
    assert backend.model.actuator_gainprm[actuator, 0] == 0.05
    assert tuple(backend.model.actuator_biasprm[actuator, 1:3]) == (-0.05, -0.005)
    assert published.world.joints("dog")[1][0, 0] == pytest.approx(0.5, abs=0.02)


def test_world_holds_at_each_control_tick_for_its_controller(scene, robot_model):
    robot = SimulatedRobot("dog", robot_model, control_period=0.02)
    backend = MujocoBackend(scene, [robot])
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
        async with _looping(backend, _Controller()):
            await asyncio.sleep(0.6)

    asyncio.run(run_briefly())
    # It is not waited for until it has first acted; from then on, though slow, it sees every tick.
    assert len(tick_times) > 5
    assert np.diff(tick_times[1:]) == pytest.approx(0.02, abs=1e-6)
    assert backend.data.ctrl[backend.model.actuator("dog/neck").id] == 0.1
