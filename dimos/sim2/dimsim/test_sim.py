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

import pytest

from dimos.robot.assets.model import UsdModel
from dimos.sim2.actions import CborKeyValueCommand, JointCommand
from dimos.sim2.dimsim.backend import BASE_JOINTS, HELD_BY, DimSimBackend
from dimos.sim2.dimsim.kinematic_base import SimKinematicBase
from dimos.sim2.dimsim.sim import DimSimSim
from dimos.sim2.scene import SimScene
from dimos.sim2.sim import compose_world
from dimos.sim2.world_observations import REFUSED, WorldMirror

pytest.importorskip("pxr")

TIMESTEP = 0.02
REACH = 1.5

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

    def Cube "cup" (
        customData = {
            bool pickable = 1
            string title = "Cup"
        }
    )
    {
        double size = 0.1
        double3 xformOp:translate = (-1, 0, 0.05)
        uniform token[] xformOpOrder = ["xformOp:translate"]
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
        usd=UsdModel.from_layer(_BOT), mount="bot", instance_name="bot/SimKinematicBase"
    )


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
    room = SimScene(
        usd=UsdModel.from_layer(_ROOM).with_frames({"bot": ("room", (0, 0, 0), (0, 0, 0))})
    )
    bot = _bot()
    try:
        world, robot = compose_world([room, bot]), bot.simulated_robot()
    finally:
        room.stop()
        bot.stop()
    engine = DimSimBackend(world, [robot], timestep=TIMESTEP, reach=REACH)
    yield _Stepped(engine)
    engine.close()


def test_the_world_says_what_is_in_it_and_stands_the_robot_on_its_mount(backend: _Stepped) -> None:
    world = backend.step(5)
    assert _cbor(world, "world")["backend"] == "dimsim"
    assert backend.pose("bot")[:3] == pytest.approx([0.0, 0.0, 0.2], abs=0.02)
    assert backend.pose("crate")[:3] == pytest.approx([1.0, 0.0, 0.2], abs=0.02)
    assert _cbor(world, "cup") == {"path": "cup", "title": "Cup", HELD_BY: None}


def test_odometry_puts_the_base_where_the_world_does(backend: _Stepped) -> None:
    """Its odometry frame is its mount's, so the height it stands above that mount stays in it."""
    backend.apply(JointCommand("bot", BASE_JOINTS, velocity=(1.0, 0.0, 0.0)))
    world = backend.step(50)
    bot = _bot()
    try:
        bot._world = world
        odom = bot._odom(world.time)
    finally:
        bot.stop()
    assert odom is not None
    assert [odom.position.x, odom.position.y, odom.position.z] == pytest.approx(
        backend.pose("bot")[:3], abs=1e-6
    )
    assert odom.position.z == pytest.approx(0.2, abs=0.02)


def test_the_base_moves_on_a_base_joint_velocity_and_pushes_a_crate(backend: _Stepped) -> None:
    backend.apply(JointCommand("bot", BASE_JOINTS, velocity=(1.0, 0.0, 0.0)))
    backend.step(100)
    assert backend.pose("bot")[0] > 0.5
    assert backend.pose("crate")[0] > 1.1
    assert backend.world.joints("bot")[1][:, 1].tolist() == [1.0, 0.0, 0.0]


def test_static_geometry_stops_the_base(backend: _Stepped) -> None:
    backend.apply(JointCommand("bot", BASE_JOINTS, velocity=(0.0, 1.0, 0.0)))
    backend.step(200)
    assert backend.pose("bot")[1] == pytest.approx(2.7, abs=0.06)


def test_what_a_kinematic_base_cannot_do_is_refused(backend: _Stepped) -> None:
    backend.apply(JointCommand("bot", ("neck",), position=(1.0,)))
    backend.step()
    assert any(key == REFUSED for _, key, _ in backend.received)


def test_a_picked_up_object_rides_with_its_robot_and_stays_where_it_is_put_down(
    backend: _Stepped,
) -> None:
    backend.apply(CborKeyValueCommand("cup", HELD_BY, "bot"))
    backend.step()
    assert _cbor(backend.world, "cup")[HELD_BY] == "bot"
    backend.apply(JointCommand("bot", BASE_JOINTS, velocity=(0.0, -1.0, 0.0)))
    backend.step(50)
    cup = backend.pose("cup")
    assert cup[1] == pytest.approx(backend.pose("bot")[1], abs=1e-6)
    assert cup[1] < -0.5
    backend.apply(CborKeyValueCommand("cup", HELD_BY, None))
    backend.step(50)
    assert _cbor(backend.world, "cup")[HELD_BY] is None
    assert backend.pose("cup")[:2] == pytest.approx(cup[:2])


def test_an_object_out_of_reach_or_not_pickable_is_refused(backend: _Stepped) -> None:
    backend.apply(CborKeyValueCommand("cup", HELD_BY, "bot"))
    backend.step()
    backend.apply(CborKeyValueCommand("cup", HELD_BY, None))
    backend.apply(JointCommand("bot", BASE_JOINTS, velocity=(3.0, 0.0, 0.0)))
    backend.step(50)
    backend.apply(CborKeyValueCommand("cup", HELD_BY, "bot"))
    backend.apply(CborKeyValueCommand("crate", HELD_BY, "bot"))
    backend.step()
    refused = [value["reason"] for _, key, value in backend.world.received if key == REFUSED]
    assert len(refused) == 2
    assert "out of bot's reach" in refused[0] or "out of bot's reach" in refused[1]


def test_the_robot_is_named_by_its_namespace() -> None:
    bot = _bot()
    robot = bot.simulated_robot()
    bot.stop()
    assert robot.robot_id == "bot"
    assert robot.root_body == "bot/base"


def test_stopping_before_the_modules_report_ends_the_wait() -> None:
    """_run waits on this in a thread, and cancelling the task cannot reach a blocked wait."""
    sim = DimSimSim()
    sim.stop()
    assert sim._built.is_set(), "a thread in _built.wait would never return"
