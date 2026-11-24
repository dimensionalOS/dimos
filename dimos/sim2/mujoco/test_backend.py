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
from pathlib import Path
import time
from typing import Any, cast
import uuid

import mujoco
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig, Mounts
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.frames import prefix_usd_frames, usd_frame_names
from dimos.robot.assets.usd_utils.layers import graft_usd
from dimos.sim2.actions import CborKeyValueCommand, JointCommand, read_actions, world_action
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.mujoco.backend import MujocoBackend
from dimos.sim2.mujoco.sim import MujocoSim
from dimos.sim2.mujoco.world import SensorMount, compile_world, mount_sensor
from dimos.sim2.sim import sim
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world_observations import WorldMirror, WorldWriter

# linux-aarch64 has no usd-core wheel, and every scene here is a USD layer.
pytest.importorskip("pxr")

SCENE = Path(__file__).parent / "assets" / "scene.usda"
# The toy's base sits here in its own USD, so a spot for it is offset by this much.
TOY_BASE = (9.0, 9.0, 9.0)


def _spot(x: float, y: float, z: float) -> tuple[float, float, float]:
    """The scene frame that puts the toy's base at (x, y, z)."""
    return (x - TOY_BASE[0], y - TOY_BASE[1], z - TOY_BASE[2])


def _scene(**spots: tuple[float, float, float]) -> UsdModel:
    """The test room with a frame at each spot a robot is grafted onto, plus an overhead frame."""
    frames = {name: ("scene", xyz, (0.0, 0.0, 0.0)) for name, xyz in spots.items()}
    return UsdModel.from_file(SCENE).with_frames(
        {"overhead": ("scene", (0.0, 0.0, 3.0), (0.0, 0.0, 0.0)), **frames}
    )


def _composed(scene: UsdModel, robots: dict[str, UsdModel]) -> UsdModel:
    """Each robot grafted onto the scene's frame of its own name, as compose_world does."""
    grafts = {
        robot_id: prefix_usd_frames(robot.usd, robot_id) for robot_id, robot in robots.items()
    }
    return UsdModel.from_layer(graft_usd(scene.usd, grafts))


@pytest.fixture
def robot_model() -> UsdModel:
    return toy_robot()


@pytest.fixture
def world(robot_model) -> UsdModel:
    return _composed(_scene(dog=_spot(0.0, 0.0, 1.0)), {"dog": robot_model})


def test_robots_grafted_into_a_scene_compile_into_one_world(robot_model):
    composed = _composed(
        _scene(left=_spot(-1.0, 0.0, 0.3), right=_spot(1.0, 0.0, 0.3)),
        {"left": robot_model, "right": robot_model},
    )
    robots = [SimulatedRobot("left", robot_model), SimulatedRobot("right", robot_model)]
    model = compile_world(usd_layer(composed), robots).compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.body("left/base").xpos.tolist() == pytest.approx([-1, 0, 0.3])
    assert data.body("right/base").xpos.tolist() == pytest.approx([1, 0, 0.3])
    assert model.joint("right/neck") is not None
    assert model.ncam == 0, "the physics world has no cameras; a camera renders its own copy"


def test_a_robot_the_world_does_not_hold_is_refused(robot_model):
    with pytest.raises(ValueError, match="no body 'base' in the world"):
        compile_world(usd_layer(_scene()), [SimulatedRobot("dog", robot_model)])


def test_a_sensor_mounts_itself_in_a_world_of_its_own(world, robot_model):
    spec = compile_world(usd_layer(world), [SimulatedRobot("dog", robot_model)])
    mount_sensor(spec, SensorMount("camera", "dog/head", "dog/eye", fovy=42.0))
    mount_sensor(spec, SensorMount("camera", "overhead", "world/overhead"))
    mount_sensor(spec, SensorMount("imu", "dog/base", "dog/imu"))
    model = spec.compile()

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.cam_fovy[model.camera("dog/eye").id] == 42.0
    assert data.cam("world/overhead").xpos.tolist() == pytest.approx([0, 0, 3])
    assert data.site("dog/imu").xpos.tolist() == pytest.approx(data.body("dog/base").xpos)
    assert model.nsensor == 0


def test_a_robot_grafted_into_a_scene_is_still_free_to_move(world, robot_model):
    model = compile_world(usd_layer(world), [SimulatedRobot("dog", robot_model)]).compile()

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    started = data.body("dog/base").xpos[2]
    for _ in range(300):
        mujoco.mj_step(model, data)

    assert data.body("dog/base").xpos[2] < started - 0.5, "the dog is welded to the room"


def test_mounting_at_a_frame_the_world_does_not_have_is_rejected():
    spec = compile_world(usd_layer(_scene()))
    with pytest.raises(ValueError, match="no site or body 'nowhere'"):
        mount_sensor(spec, SensorMount("imu", "nowhere", "nowhere"))


def test_a_robot_grafted_under_a_frame_still_reports_its_own_pose(robot_model):
    """The frame a robot hangs off is jointed and shortens to the robot's name; it must not take the robot's place."""
    mount = 2.0
    composed = _composed(_scene(dog=(0.0, 0.0, mount)), {"dog": robot_model})
    backend = MujocoBackend(composed, [SimulatedRobot("dog", robot_model)])
    base = backend.model.body("dog/base")
    held = backend.model.body(int(backend.model.body_rootid[base.id]))
    assert held.id != base.id, "the fixture is not composed; nothing holds the robot"
    mirror = WorldMirror()
    mirror.apply(backend.frame())
    dog = mirror.entity("dog")
    assert dog is not None and dog.pose is not None
    assert dog.pose[:3] == pytest.approx(backend.data.xpos[base.id], abs=1e-6)
    assert backend.data.xpos[held.id][2] == pytest.approx(mount, abs=1e-6)
    assert dog.pose[2] != pytest.approx(mount, abs=1e-3)


def test_the_world_the_backend_runs_is_the_usd_it_was_given(world, robot_model):
    backend = MujocoBackend(world, [SimulatedRobot("dog", robot_model)])
    assert backend.usd == world.usd
    assert backend.describe() == {"backend": "mujoco", "scene": str(usd_layer(world))}


def test_two_robots_with_one_id_are_refused_by_name(world, robot_model):
    with pytest.raises(ValueError, match="duplicate simulated robot ids"):
        MujocoBackend(world, [SimulatedRobot("dog", robot_model)] * 2)


class _RoomConfig(HardwareModuleConfig):
    usd: UsdModel | None = _scene(dog=_spot(0.0, 0.0, 1.0))


class _Room(HardwareModule):
    config: _RoomConfig
    mounts = Mounts("dog")


class _DogConfig(HardwareModuleConfig):
    usd: UsdModel | None = toy_robot()


class _Dog(HardwareModule):
    config: _DogConfig

    @rpc
    def simulated_robot(self) -> SimulatedRobot:
        assert self.config.usd is not None
        return SimulatedRobot(self.namespace, self.config.usd)


def test_the_sim_builds_its_world_from_the_blueprints_hardware():
    blueprint = autoconnect(
        sim({"mujoco": MujocoSim.blueprint()}, default="mujoco"),
        _Room.blueprint(),
        _Dog.blueprint(_mount="dog").namespace("dog"),
    ).solidify()
    # Its own zenoh domain, so no other sim running on this machine answers its RPCs.
    overrides = {"g": {"viewer": "none", "zenoh_domain": f"sim2-test-{uuid.uuid4().hex[:8]}"}}
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides=overrides)
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        sim_module = coordinator.get_instance(MujocoSim)
        assert {"dog", "dog/base", "dog/neck"} <= usd_frame_names(sim_module.get_world_usd())
        mirror = WorldMirror()
        mirror._load(sim_module.snapshot()["entities"], 0)
        assert mirror.entity("dog") is not None
    finally:
        coordinator.stop()


class _Collect:
    def __init__(self) -> None:
        self.world = WorldMirror()

    def offer(self, frame: Any) -> None:
        self.world.apply(frame)

    def close(self) -> None:
        pass


@contextlib.asynccontextmanager
async def _looping(backend: MujocoBackend, published: _Collect) -> Any:
    """MujocoSim's stepping loop, on a backend built here instead of from a blueprint."""
    sim_module = MujocoSim()
    sim_module._backend, sim_module._publisher = backend, cast("Any", published)
    sim_module._first_frame = backend.frame()
    sim_module._built.set()
    running = asyncio.create_task(sim_module._run())
    try:
        yield sim_module
    finally:
        running.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await running
        sim_module.stop()


def test_backend_steps_physics_and_takes_commands(world, robot_model):
    backend = MujocoBackend(world, [SimulatedRobot("dog", robot_model)])
    published = _Collect()

    async def run_briefly() -> None:
        async with _looping(backend, published):
            await asyncio.sleep(0.05)
            # The head is light: a bigger torque spins it up until the solver diverges.
            backend.apply([JointCommand("dog", ("neck",), effort=(0.01,))])
            await asyncio.sleep(0.5)

    asyncio.run(run_briefly())
    world_seen = published.world
    assert world_seen.time > 0.3
    assert world_seen.entity("dog").pose[2] < 0.3
    names, joints = world_seen.joints("dog")
    assert names == ["neck"]
    assert joints[0, 1] > 1.0
    assert world_seen.entity("dog").velocity is not None


def test_frames_report_robots_and_objects_and_actions_drive_them(world, robot_model):
    backend = MujocoBackend(world, [SimulatedRobot("dog", robot_model)])
    backend.apply([JointCommand("dog", ("neck",), effort=(0.5,))])
    backend._apply_pending()
    seen = _Collect()
    seen.offer(backend.frame())
    assert seen.world.entity("dog").cbor["model"] == str(usd_layer(robot_model))
    assert seen.world.joints("dog")[0] == ["neck"]
    assert seen.world.entity("crate").pose[:3] == pytest.approx([0, 2, 0.5])
    assert seen.world.time == pytest.approx(time.time(), abs=5.0)
    assert backend.data.ctrl[backend.model.actuator("dog/neck").id] == 0.5


def test_actions_cross_the_wire_by_id_and_cbor_keys_are_observed_back(world, robot_model):
    backend = MujocoBackend(world, [SimulatedRobot("dog", robot_model)])
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


def _with_a_driven_neck(usd: str, kp: float, kd: float, target: float) -> str:
    """The toy's neck as a driven joint, in standard USD alone (the toy ships as a torque motor)."""
    from pxr import Sdf, Usd, UsdPhysics

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    stage = Usd.Stage.Open(layer)
    joint = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint) and p.GetName() == "neck")
    drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
    drive.CreateStiffnessAttr(kp)
    drive.CreateDampingAttr(kd)
    drive.CreateTargetPositionAttr(target)
    return str(layer.ExportToString())


def test_a_motor_command_drives_a_servo_with_its_own_gains(robot_model):
    driven = UsdModel.from_layer(_with_a_driven_neck(robot_model.usd, 0.01, 0.001, 0.2))
    composed = _composed(_scene(dog=_spot(0.0, 0.0, 1.0)), {"dog": driven})
    backend = MujocoBackend(composed, [SimulatedRobot("dog", driven)])
    assert backend.data.qpos[backend.model.joint("dog/neck").qposadr[0]] == pytest.approx(0.2)
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


def test_closing_the_publisher_while_a_frame_is_in_flight_does_not_wedge_it():
    from threading import Event as ThreadEvent, Thread

    from dimos.sim2.mujoco.sim import _LatestPublisher

    started, release = ThreadEvent(), ThreadEvent()

    def publish(_frame: Any) -> None:
        started.set()
        release.wait(5)

    publisher = _LatestPublisher(publish)
    publisher.offer(cast("Any", object()))
    assert started.wait(5), "the publisher thread never picked the frame up"

    closer = Thread(target=publisher.close)
    closer.start()
    release.set()
    closer.join(5)

    assert not closer.is_alive(), "close() never returned: the publishing thread is wedged"


def test_frames_offered_while_one_is_in_flight_are_merged_not_lost() -> None:
    from threading import Event as ThreadEvent

    from dimos.sim2.mujoco.sim import _LatestPublisher

    writer = WorldWriter()
    writer.add("crate")
    published: list[Any] = []
    in_flight, release, followed = ThreadEvent(), ThreadEvent(), ThreadEvent()

    def publish(frame: Any) -> None:
        published.append(frame)
        if len(published) == 1:
            in_flight.set()
            release.wait(5)
        else:
            followed.set()

    publisher = _LatestPublisher(publish)
    publisher.offer(writer.end_frame(0.0))
    assert in_flight.wait(5), "the publisher thread never picked the frame up"
    writer.set("crate", "note", {"a": 1})
    publisher.offer(writer.end_frame(0.1))
    writer.position("crate", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    publisher.offer(writer.end_frame(0.2))
    release.set()
    assert followed.wait(5), "the frames offered mid-publish were dropped"
    publisher.close()

    mirror = WorldMirror()
    for frame in published:
        mirror.apply(frame)
    crate = mirror.entity("crate")
    assert len(published) == 2
    assert crate is not None and crate.cbor["note"] == {"a": 1}
    assert crate.pose is not None and crate.pose[:3] == pytest.approx([1.0, 0.0, 0.0])
