# Copyright 2025-2026 Dimensional Inc.
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


from collections.abc import Iterator
from typing import Any

import numpy as np
import pytest

from dimos.core.hardware_files import hardware_file_topics
from dimos.core.hardware_module import HardwareModule
from dimos.core.transport_factory import make_transport
from dimos.msgs.files.FileInfo import FileInfo
from dimos.robot.assets.model import UsdModel
from dimos.sim2.actions import JointCommand
from dimos.sim2.dimsim.backend import BASE_JOINTS, DimSimBackend
from dimos.sim2.dimsim.kinematic_base import SimKinematicBase
from dimos.sim2.dimsim.sim import DimSimSim
from dimos.sim2.scene import SimScene
from dimos.sim2.sim import compose_usds, fetched_usd
from dimos.sim2.spec import ODOM_FRAME
from dimos.sim2.world_observations import REFUSED, WorldMirror

pytest.importorskip("pxr")

TIMESTEP = 0.02
# The room's frame the bot mounts on, and so the bot's namespace and robot id.
BOT = "room/bot"
# The bot's spot: its base's height above the floor, as a real robot's mount carries it.
SPOT = (0.0, 0.0, 0.2)

_ROOM = """#usda 1.0
(
    defaultPrim = "room"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "room"
{
    def Plane "floor"
    {
        uniform token axis = "Z"
        double length = 20
        double width = 20
    }

    def Cube "wall"
    {
        double size = 2
        float3 xformOp:scale = (2, 0.1, 0.5)
        double3 xformOp:translate = (0, 3, 0.5)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }

    def Cube "crate" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
    )
    {
        float physics:mass = 0.5
        double size = 2
        float3 xformOp:scale = (0.2, 0.2, 0.2)
        double3 xformOp:translate = (1, 0, 0.2)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }
}
"""

_BOT = """#usda 1.0
(
    defaultPrim = "bot"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "bot"
{
    def Cube "base" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
    )
    {
        float physics:mass = 5
        double size = 0.4
    }
}
"""


def _bot() -> SimKinematicBase:
    return SimKinematicBase(
        usd=UsdModel.from_layer(_BOT), attached_to=BOT, instance_name=f"{BOT}/SimKinematicBase"
    )


def _fetched(module: HardwareModule) -> tuple[str, str | None]:
    """``module``'s USD fetched over its hardware_file streams (wired as a blueprint wires them), and its mount."""
    request_topic, response_topic = hardware_file_topics(module.namespace)
    module.hardware_file_request.transport = make_transport(request_topic, FileInfo)
    module.hardware_file_response.transport = make_transport(response_topic, FileInfo)
    module.provide_hardware_files()
    return fetched_usd(module.namespace), module.attached_to()


class _Stepped:
    def __init__(self, backend: DimSimBackend) -> None:
        self.backend, self.world = backend, WorldMirror()
        self.received: list[Any] = []
        self.step(0)

    def apply(self, *actions: Any) -> None:
        self.backend.apply(actions)

    def step(self, steps: int = 1) -> WorldMirror:
        _at, frame = self.backend.advance(steps) if steps else (0.0, self.backend.frame())
        self.world.apply(frame)
        self.received += self.world.received
        return self.world

    def pose(self, name: str) -> list[float]:
        entity = self.world.entity(name)
        assert entity is not None and entity.pose is not None
        return list(entity.pose.tolist())


def _cbor(world: WorldMirror, name: str) -> dict[str, Any]:
    entity = world.entity(name)
    assert entity is not None
    return entity.cbor


@pytest.fixture
def backend() -> Iterator[_Stepped]:
    room = SimScene(usd=UsdModel.from_layer(_ROOM).with_frames({"bot": ("room", SPOT, (0, 0, 0))}))
    bot = _bot()
    try:
        world, robot = (
            compose_usds([_fetched(room), _fetched(bot)]),
            bot.simulated_robot(),
        )
    finally:
        room.stop()
        bot.stop()
    engine = DimSimBackend(world, [robot], timestep=TIMESTEP)
    yield _Stepped(engine)
    engine.close()


def test_the_world_says_what_is_in_it_and_stands_the_robot_on_its_mount(backend: _Stepped) -> None:
    world = backend.step(5)
    assert _cbor(world, "world")["backend"] == "dimsim"
    assert backend.pose(BOT)[:3] == pytest.approx([0.0, 0.0, 0.2], abs=0.02)
    assert backend.pose("crate")[:3] == pytest.approx([1.0, 0.0, 0.2], abs=0.02)


def test_odometry_puts_the_base_where_the_world_does(backend: _Stepped) -> None:
    """Its odometry starts on its mount, which holds odom by an identity edge."""
    bot = _bot()
    try:
        bot._world = backend.world
        assert bot._odom(backend.world.time) is not None
        backend.apply(JointCommand(BOT, BASE_JOINTS, velocity=(1.0, 0.0, 0.0)))
        world = backend.step(50)
        odom = bot._odom(world.time)
    finally:
        bot.stop()
    assert odom is not None
    position = np.add([odom.position.x, odom.position.y, odom.position.z], SPOT)
    assert position.tolist() == pytest.approx(backend.pose(BOT)[:3], abs=1e-6)
    assert backend.pose(BOT)[0] > 0.5


def test_the_base_moves_on_a_base_joint_velocity_and_pushes_a_crate(backend: _Stepped) -> None:
    backend.apply(JointCommand(BOT, BASE_JOINTS, velocity=(1.0, 0.0, 0.0)))
    backend.step(100)
    assert backend.pose(BOT)[0] > 0.5
    assert backend.pose("crate")[0] > 1.1
    assert backend.world.joints(BOT)[1][:, 1].tolist() == [1.0, 0.0, 0.0]


def test_a_moving_body_sends_its_velocity_and_acceleration(backend: _Stepped) -> None:
    backend.apply(JointCommand(BOT, BASE_JOINTS, velocity=(0.0, -0.5, 0.3)))
    backend.step(5)
    bot = backend.world.entity(BOT)
    assert bot is not None and bot.velocity is not None and bot.acceleration is not None
    # Its own y, turned by the yaw it has come round to, in world axes.
    yaw = 0.3 * 5 * TIMESTEP
    assert bot.velocity[:2] == pytest.approx([0.5 * np.sin(yaw), -0.5 * np.cos(yaw)], abs=0.02)
    assert bot.velocity[5] == pytest.approx(0.3, abs=1e-3)
    # Turning at a steady rate swings its velocity round, which is its whole acceleration.
    assert np.linalg.norm(bot.acceleration[:2]) == pytest.approx(0.5 * 0.3, abs=0.02)


def test_the_last_state_sent_of_a_body_that_stopped_is_still(backend: _Stepped) -> None:
    backend.apply(JointCommand(BOT, BASE_JOINTS, velocity=(1.0, 0.0, 0.0)))
    backend.step(40)
    crate = backend.world.entity("crate")
    assert crate is not None and crate.velocity is not None
    assert crate.velocity[0] > 0.1, "the pushed crate sent no velocity"
    backend.apply(JointCommand(BOT, BASE_JOINTS, velocity=(0.0, 0.0, 0.0)))
    # Long enough for the crate to sleep and the base to rise to its character controller's offset.
    backend.step(800)
    for name in (BOT, "crate"):
        entity = backend.world.entity(name)
        assert entity is not None and entity.velocity is not None
        assert entity.velocity.tolist() == pytest.approx([0.0] * 6, abs=1e-3), name


def test_static_geometry_stops_the_base(backend: _Stepped) -> None:
    backend.apply(JointCommand(BOT, BASE_JOINTS, velocity=(0.0, 1.0, 0.0)))
    backend.step(200)
    assert backend.pose(BOT)[1] == pytest.approx(2.7, abs=0.06)


def test_what_a_kinematic_base_cannot_do_is_refused(backend: _Stepped) -> None:
    backend.apply(JointCommand(BOT, ("neck",), position=(1.0,)))
    backend.step()
    assert any(key == REFUSED for _, key, _ in backend.received)


def test_the_robot_is_named_by_its_namespace() -> None:
    bot = _bot()
    try:
        robot = bot.simulated_robot()
    finally:
        bot.stop()
    assert robot.robot_id == BOT
    assert robot.root_body == "bot/base"


def test_stopping_before_the_modules_report_ends_the_wait() -> None:
    """_run waits on this in a thread, and cancelling the task cannot reach a blocked wait."""
    sim = DimSimSim()
    sim.stop()
    assert sim._built.is_set(), "a thread in _built.wait would never return"


def test_the_world_announces_the_frame_the_base_reports_its_odometry_in(
    backend: _Stepped,
) -> None:
    world = backend.step(1)
    bot = _bot()
    try:
        bot._world = world
        odom = bot._odom(world.time)
    finally:
        bot.stop()
    assert odom is not None
    assert _cbor(world, BOT)[ODOM_FRAME] == odom.frame_id
