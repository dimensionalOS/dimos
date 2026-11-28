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
import threading
import time
from typing import Any, cast
import uuid

import mujoco
import numpy as np
from numpy.typing import NDArray
import pytest

# linux-aarch64 has no usd-core wheel, and every scene here is a USD layer.
pytest.importorskip("pxr")

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig, Mounts
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.frames import usd_frame_names
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2 import sim as sim_protocol
from dimos.sim2.mujoco import sim as mujoco_sim
from dimos.sim2.mujoco.backend import MujocoBackend
from dimos.sim2.mujoco.sim import MujocoSim, _LatestPublisher
from dimos.sim2.mujoco.world import compile_world
from dimos.sim2.scene import LOGISTICS, layer_file, scene_usd, toy_robot, usd_layer
from dimos.sim2.sim import (
    ODOM_FRAME,
    ROBOT,
    CborKeyValueCommand,
    Entity,
    JointCommand,
    SimulatedRobot,
    WorldMirror,
    WorldWriter,
    compose_usds,
    read_actions,
    sim,
    world_action,
)

SCENE = Path(__file__).parent / "assets" / "scene.usda"
# The toy's base sits here in its own USD, so a spot for it is offset by this much.
TOY_BASE = (9.0, 9.0, 9.0)


def _spot(x: float, y: float, z: float) -> tuple[float, float, float]:
    """The scene frame that puts the toy's base at (x, y, z)."""
    return (x - TOY_BASE[0], y - TOY_BASE[1], z - TOY_BASE[2])


def _scene(**spots: tuple[float, float, float]) -> UsdModel:
    """The test room with a frame at each spot a robot is mounted on, plus an overhead frame."""
    frames = {name: ("scene", xyz, (0.0, 0.0, 0.0)) for name, xyz in spots.items()}
    return UsdModel.from_file(SCENE).with_frames(
        {"overhead": ("scene", (0.0, 0.0, 3.0), (0.0, 0.0, 0.0)), **frames}
    )


def _composed(scene: UsdModel, robots: dict[str, UsdModel]) -> UsdModel:
    """Each robot mounted on the scene's frame of its own name, its prims under that frame, as compose_world composes them."""
    mounted = [
        (usd_under_namespace(robot.usd, f"scene/{robot_id}"), f"scene/{robot_id}")
        for robot_id, robot in robots.items()
    ]
    return compose_usds([(scene.usd, None), *mounted])


@pytest.fixture
def robot_model() -> UsdModel:
    return toy_robot()


@pytest.fixture
def world(robot_model: UsdModel) -> UsdModel:
    return _composed(_scene(dog=_spot(0.0, 0.0, 1.0)), {"dog": robot_model})


def test_robots_mounted_in_a_scene_compile_into_one_world_free_or_held_by_their_mount(
    robot_model: UsdModel,
) -> None:
    composed = _composed(
        _scene(free=_spot(-1.0, 0.0, 1.0), mounted=_spot(1.0, 0.0, 1.0)),
        {"free": robot_model, "mounted": robot_model},
    )
    robots = [
        SimulatedRobot("free", robot_model),
        SimulatedRobot("mounted", robot_model, free=False),
    ]
    model = compile_world(usd_layer(composed), robots).compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.body("free/base").xpos.tolist() == pytest.approx([-1, 0, 1.0])
    assert data.body("mounted/base").xpos.tolist() == pytest.approx([1, 0, 1.0])
    assert model.joint("mounted/neck") is not None
    assert model.ncam == 0, "the physics world has no cameras; a camera renders its own copy"
    for _ in range(300):
        mujoco.mj_step(model, data)
    assert data.body("free/base").xpos[2] < 0.5, "the free robot is welded to the room"
    assert data.body("mounted/base").xpos[2] > 0.5, "the mounted robot fell off its mount"

    with pytest.raises(ValueError, match=f"no body '{robot_model.root_link}' in the world"):
        compile_world(usd_layer(_scene()), [SimulatedRobot("dog", robot_model)])
    # A spawn point's volume neither collides nor renders.
    room = compile_world(layer_file(scene_usd(LOGISTICS).usd, "logistics"), ids=()).compile()
    names = [room.geom(geom).name for geom in range(room.ngeom)]
    assert names and not [name for name in names if "spawns" in name]


def test_a_backend_runs_the_usd_it_was_given_and_reports_a_robot_at_its_base(
    robot_model: UsdModel,
) -> None:
    mount = 2.0
    composed = _composed(_scene(dog=(0.0, 0.0, mount)), {"dog": robot_model})
    with pytest.raises(ValueError, match="duplicate simulated robot ids"):
        MujocoBackend(composed, [SimulatedRobot("dog", robot_model)] * 2)
    backend = MujocoBackend(composed, [SimulatedRobot("dog", robot_model)])
    assert backend.usd == composed.usd
    # The frame a robot hangs off is jointed and shortens to its name; it must not take its place.
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


class _RoomConfig(HardwareModuleConfig):
    usd: UsdModel = _scene(dog=_spot(0.0, 0.0, 1.0))


class _Room(HardwareModule):
    config: _RoomConfig
    mounts = Mounts("scene/dog")


class _DogConfig(HardwareModuleConfig):
    usd: UsdModel = toy_robot()


class _Dog(HardwareModule):
    config: _DogConfig

    @rpc
    def simulated_robot(self) -> SimulatedRobot:
        return SimulatedRobot(self.namespace, self.config.usd)


def test_the_sim_builds_its_world_from_the_blueprints_hardware() -> None:
    blueprint = autoconnect(
        sim({"mujoco": MujocoSim.blueprint()}, default="mujoco"),
        _Room.blueprint(),
        _Dog.attach(_Room.mounts.dog),
    ).solidify()
    # Its own zenoh domain, so no other sim running on this machine answers its RPCs.
    overrides = {"g": {"viewer": "none", "zenoh_domain": f"sim2-test-{uuid.uuid4().hex[:8]}"}}
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides=overrides)
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        sim_module = coordinator.get_instance(MujocoSim)
        frames = usd_frame_names(sim_module.get_world_usd())
        assert {
            "scene/dog",
            "scene/dog/toy/Geometry/base",
            "scene/dog/toy/Geometry/base/head",
        } <= frames
        mirror = WorldMirror()
        mirror._load(sim_module.snapshot()["entities"], 0)
        # A robot's id is its namespace, the tf frame of the mount it sits on (its name).
        assert mirror.entity("dog") is not None
    finally:
        coordinator.stop()


def test_the_world_usd_is_served_while_physics_is_still_compiling(
    world: UsdModel, monkeypatch: pytest.MonkeyPatch
) -> None:
    compiling, compiled = threading.Event(), threading.Event()

    class _StillCompiling:
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            compiling.set()
            compiled.wait()

        def frame(self) -> None:
            return None

        def close(self) -> None:
            pass

    monkeypatch.setattr(sim_protocol, "compose_world", lambda _hardware: world)
    monkeypatch.setattr(mujoco_sim, "MujocoBackend", _StillCompiling)
    sim_module = MujocoSim()
    building = threading.Thread(target=sim_module.on_system_modules, args=([],))
    building.start()
    try:
        compiling.wait()
        assert sim_module.get_world_usd() == world.usd
    finally:
        compiled.set()
        building.join()
        sim_module.stop()


def _seen(world: WorldMirror, path: str) -> Entity:
    """The entity at ``path``, which the world must have announced."""
    entity = world.entity(path)
    assert entity is not None, f"the world announced no {path!r}"
    return entity


def _pose(world: WorldMirror, path: str) -> NDArray[np.float64]:
    """The pose the world last sent for ``path``."""
    pose = _seen(world, path).pose
    assert pose is not None, f"the world sent no pose for {path!r}"
    return pose


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


async def _ran_for(backend: MujocoBackend, published: _Collect, seconds: float) -> None:
    """Until the published world has run ``seconds`` of sim time, however long that takes on the wall."""
    while not published.world.frame or backend.data.time < seconds:
        await asyncio.sleep(backend.timestep)


def test_a_held_robot_stands_still_on_a_running_clock_until_commanded_then_moves(
    world: UsdModel, robot_model: UsdModel
) -> None:
    """A policy commands on fresh readings, which tick on wall time, so a held world keeps its clock running."""
    backend = MujocoBackend(world, [SimulatedRobot("dog", robot_model, held_until_commanded=True)])
    published = _Collect()

    async def hold_then_drive() -> tuple[float, float, float]:
        async with _looping(backend, published) as sim_module:
            sim_module._held = {"dog"}
            await _ran_for(backend, published, 0.0)
            standing = _pose(published.world, "dog")[2]
            await _ran_for(backend, published, 0.3)
            still = _pose(published.world, "dog")[2]
            # The head is light: a bigger torque spins it up until the solver diverges.
            command = JointCommand("dog", ("neck",), effort=(0.01,))
            sim_module._on_actions(world_action(published.world, command))
            await _ran_for(backend, published, 0.85)
            return standing, still, _pose(published.world, "dog")[2]

    standing, still, fallen = asyncio.run(hold_then_drive())
    assert still == pytest.approx(standing)
    assert fallen < 0.3
    names, joints = published.world.joints("dog")
    assert names == ["neck"]
    assert joints[0, 1] > 1.0
    assert _seen(published.world, "dog").velocity is not None


def test_frames_announce_robots_and_moving_objects_and_actions_cross_the_wire_to_drive_them(
    world: UsdModel, robot_model: UsdModel
) -> None:
    backend = MujocoBackend(world, [SimulatedRobot("dog", robot_model, odom_frame="world")])
    seen = _Collect()
    seen.offer(backend.frame())
    dog = _seen(seen.world, "dog")
    assert dog.cbor[ROBOT] is True
    assert dog.cbor[ODOM_FRAME] == "world"
    assert seen.world.joints("dog")[0] == ["neck"]
    assert _pose(seen.world, "crate")[:3] == pytest.approx([0, 2, 0.5])
    assert seen.world.time == pytest.approx(time.time(), abs=5.0)

    # Velocity and acceleration come from MuJoCo, linear then angular in world axes, without gravity.
    model, data = backend.model, backend.data
    lifted = model.jnt_qposadr[model.body_jntadr[backend._roots["crate"]]] + 2
    data.qpos[lifted] += 1.0
    mujoco.mj_forward(model, data)
    falling_from = data.time
    for _ in range(10):
        mujoco.mj_step(model, data)
    seen.offer(backend.frame())
    crate = _seen(seen.world, "crate")
    assert crate.velocity is not None and crate.acceleration is not None
    # A frame reads the state its last step started from: the velocity a step before data.time.
    fallen_for = data.time - falling_from - model.opt.timestep
    assert crate.velocity[:3] == pytest.approx(model.opt.gravity * fallen_for)
    assert crate.acceleration[:3] == pytest.approx(model.opt.gravity, abs=1e-6)
    assert crate.velocity[3:] == pytest.approx([0, 0, 0])
    assert crate.acceleration[3:] == pytest.approx([0, 0, 0])

    while data.time < falling_from + 2.0:
        mujoco.mj_step(model, data)
    seen.offer(backend.frame())
    crate = _seen(seen.world, "crate")
    assert crate.velocity == pytest.approx(np.zeros(6), abs=1e-3)
    assert crate.acceleration == pytest.approx(np.zeros(6), abs=1e-2)

    actions: list[JointCommand | CborKeyValueCommand] = [
        JointCommand("dog", ("neck",), effort=(0.5,)),
        CborKeyValueCommand("dog", "note", {"a": 1}),
    ]
    servo = JointCommand("dog", ("neck",), position=(0.2,), kp=(10.0,), kd=(1.0,))
    on_the_wire = world_action(seen.world, servo, *actions)
    assert read_actions(on_the_wire, backend.world) == [servo, *actions]
    backend.apply(actions)
    backend._apply_pending()
    seen.offer(backend.frame())
    assert _seen(seen.world, "dog").cbor["note"] == {"a": 1}
    assert backend.data.ctrl[backend.model.actuator("dog/neck").id] == 0.5


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


def test_a_motor_command_drives_a_servo_with_its_own_gains(robot_model: UsdModel) -> None:
    driven = UsdModel.from_layer(_with_a_driven_neck(robot_model.usd, 0.01, 0.001, 0.2))
    composed = _composed(_scene(dog=_spot(0.0, 0.0, 1.0)), {"dog": driven})
    backend = MujocoBackend(composed, [SimulatedRobot("dog", driven)])
    assert backend.data.qpos[backend.model.joint("dog/neck").qposadr[0]] == pytest.approx(0.2)
    published = _Collect()

    async def run_briefly() -> None:
        async with _looping(backend, published):
            await _ran_for(backend, published, 0.05)
            backend.apply(
                [JointCommand("dog", ("neck",), position=(0.5,), kp=(0.05,), kd=(0.005,))]
            )
            await _ran_for(backend, published, 1.05)

    asyncio.run(run_briefly())
    actuator = backend.model.actuator("dog/neck").id
    assert backend.model.actuator_gainprm[actuator, 0] == 0.05
    assert tuple(backend.model.actuator_biasprm[actuator, 1:3]) == (-0.05, -0.005)
    assert published.world.joints("dog")[1][0, 0] == pytest.approx(0.5, abs=0.02)


def test_closing_the_publisher_while_a_frame_is_in_flight_does_not_wedge_it() -> None:
    started, release = threading.Event(), threading.Event()

    def publish(_frame: Any) -> None:
        started.set()
        release.wait(5)

    publisher = _LatestPublisher(publish)
    publisher.offer(cast("Any", object()))
    assert started.wait(5), "the publisher thread never picked the frame up"
    closer = threading.Thread(target=publisher.close)
    closer.start()
    release.set()
    closer.join(5)
    assert not closer.is_alive(), "close() never returned: the publishing thread is wedged"


def test_frames_offered_while_one_is_in_flight_are_merged_not_lost() -> None:
    writer = WorldWriter()
    writer.add("crate")
    published: list[Any] = []
    in_flight, release, followed = threading.Event(), threading.Event(), threading.Event()

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
    writer.body_state("crate", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], [0.0] * 6, [0.0] * 6)
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
