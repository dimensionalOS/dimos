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

import mujoco
import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.frames import prefix_usd_frames
from dimos.robot.assets.usd_utils.layers import graft_usd
from dimos.sim2.actions import CborKeyValueCommand, JointCommand, read_actions, world_action
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.mujoco.backend import MujocoBackend
from dimos.sim2.mujoco.sim import MujocoSim
from dimos.sim2.mujoco.world import SensorMount, compile_world, mount_sensor
from dimos.sim2.sim import sim_module
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.usd_layer import layer_file, usd_layer
from dimos.sim2.world_observations import ACT, WorldMirror

# Every scene here is read out of a USD layer, and a runner without usd-core cannot read one:
# linux-aarch64 has no wheel for it (pyproject's marker says so), so without this the tests
# fail there on dimos's own "Reading or writing USD needs usd-core (pxr)". Their neighbours
# in this directory skip the same way.
pytest.importorskip("pxr")

SCENE = Path(__file__).parent / "assets" / "scene.usda"


@pytest.fixture
def scene() -> UsdModel:
    return UsdModel.from_file(SCENE).with_frames(
        {"overhead": ("scene", (0.0, 0.0, 3.0), (0.0, 0.0, 0.0))}
    )


@pytest.fixture
def robot_model() -> UsdModel:
    return toy_robot()


def test_robots_compile_into_one_world(scene, robot_model):
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
    assert data.site("left/imu").xpos.tolist() == pytest.approx([-1, 0, 0.3])
    assert model.nsensor == 0


def test_a_robot_grafted_into_a_scene_is_still_free_to_move(scene, robot_model):
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
    assert data.body("/scene/overhead").xpos.tolist() == pytest.approx([0, 0, 3])


def test_mounting_at_a_frame_the_world_does_not_have_is_rejected(scene):
    spec = compile_world(usd_layer(scene))
    with pytest.raises(ValueError, match="no site or body 'nowhere'"):
        mount_sensor(spec, SensorMount("imu", "nowhere", "nowhere"))


def test_the_engine_runs_the_robots_the_blueprint_put_in_the_world(scene, robot_model):
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
    sim = MujocoSim(scene=scene, robots=[SimulatedRobot("dog", robot_model)])
    try:
        assert sim._engine().describe() == {"backend": "mujoco", "scene": str(usd_layer(scene))}
    finally:
        sim.stop()


class _Collect:
    busy = False

    def __init__(self) -> None:
        self.world = WorldMirror()
        self.frames: list[Any] = []

    def offer(self, frame: Any) -> None:
        self.world.apply(frame)
        self.frames.append(frame)


@contextlib.asynccontextmanager
async def _looping(backend: MujocoBackend, published: _Collect) -> Any:
    sim = MujocoSim()
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
    assert world.entity("dog").pose[2] < 0.3
    names, joints = world.joints("dog")
    assert names == ["neck"]
    assert joints[0, 1] > 1.0
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


def _with_a_servo_neck(usd: str, kp: float, kd: float, target: float) -> str:
    """The toy's neck as a position servo, which is the only robot shape this test can use.

    A joint says its gains twice: UsdPhysics.DriveAPI is what dimos reads for the start pose,
    and the MjcActuator's affine bias is what MuJoCo servos on -- the backend refuses gains for
    an actuator that is not one. The toy ships as a torque motor because every other test here
    drives it by effort, so this test authors the servo it needs.
    """
    from pxr import Sdf, Usd, UsdPhysics

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    stage = Usd.Stage.Open(layer)
    joint = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint) and p.GetName() == "neck")
    drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
    drive.CreateStiffnessAttr(kp)
    drive.CreateDampingAttr(kd)
    drive.CreateTargetPositionAttr(target)
    actuator = next(p for p in stage.Traverse() if p.GetTypeName() == "MjcActuator")
    doubles = Sdf.ValueTypeNames.DoubleArray
    actuator.CreateAttribute("mjc:gainPrm", doubles).Set([kp, *[0.0] * 9])
    actuator.CreateAttribute("mjc:biasPrm", doubles).Set([0.0, -kp, -kd, *[0.0] * 7])
    actuator.CreateAttribute("mjc:biasType", Sdf.ValueTypeNames.Token).Set("affine")
    return str(layer.ExportToString())


def test_a_motor_command_drives_a_servo_with_its_own_gains(scene, robot_model):
    driven = UsdModel.from_layer(_with_a_servo_neck(robot_model.usd, 0.01, 0.001, 0.2))
    backend = MujocoBackend(scene, [SimulatedRobot("dog", driven)])
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


def test_world_holds_at_each_control_tick_for_its_controller(scene, robot_model):
    robot = SimulatedRobot("dog", robot_model, control_period=0.02)
    backend = MujocoBackend(scene, [robot])
    tick_times: list[float] = []

    class _Controller(_Collect):
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
    assert len(tick_times) > 5
    assert np.diff(tick_times[1:]) == pytest.approx(0.02, abs=1e-6)
    assert backend.data.ctrl[backend.model.actuator("dog/neck").id] == 0.1


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


def test_two_robots_with_one_id_are_refused_by_name(scene, robot_model):
    with pytest.raises(ValueError, match="duplicate simulated robot ids"):
        MujocoSim(
            scene=scene,
            robots=(
                SimulatedRobot("dog", robot_model),
                SimulatedRobot("dog", robot_model),
            ),
        )
