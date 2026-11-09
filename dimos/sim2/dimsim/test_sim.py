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

"""The DimSim engine: a world that steps, a base that slides, a crate that gets pushed, and refusals."""

from typing import Any

import pytest

from dimos.robot.assets.model import UsdModel
from dimos.sim2.actions import JointCommand
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.dimsim.backend import BASE_JOINTS, DimSimBackend
from dimos.sim2.dimsim.sim import DimSimSim
from dimos.sim2.sim import BACKENDS, sim_module
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world_observations import REFUSED, WorldMirror

# A floor, a wall across +y, and one crate light enough for the toy robot to shove.
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


@pytest.fixture
def scene() -> UsdModel:
    return UsdModel.from_layer(_ROOM)


class _Stepped:
    """The engine, stepped by hand, and the world as its frames say it is."""

    def __init__(self, backend: DimSimBackend) -> None:
        self.backend, self.world = backend, WorldMirror()
        self.received: list[Any] = []

    def apply(self, *actions: Any) -> None:
        self.backend.apply(actions)

    def step(self, steps: int = 1) -> WorldMirror:
        self.world.apply(self.backend.advance(steps)[1])
        self.received += self.world.received
        return self.world

    def x(self, name: str) -> float:
        return float(self.world.entity(name).pose[0])


@pytest.fixture
def backend(scene):
    engine = DimSimBackend(scene, [SimulatedRobot("bot", toy_robot())])
    yield _Stepped(engine)
    engine.close()


def test_the_world_steps_and_says_what_is_in_it(backend):
    world = backend.step(5)
    assert world.time > 0.0
    assert world.entity("world").cbor["backend"] == "dimsim"
    # The robot is the one its description named, and the crate the one the scene did.
    assert world.entity("bot") is not None
    assert world.entity("crate").pose[:3] == pytest.approx([1.0, 0.0, 0.2], abs=0.02)


def test_the_base_moves_on_a_base_joint_velocity_and_pushes_a_crate(backend):
    backend.apply(JointCommand("bot", BASE_JOINTS, velocity=(1.0, 0.0, 0.0)))
    backend.step()
    assert backend.x("crate") == pytest.approx(1.0, abs=0.02)
    for _ in range(100):
        backend.step()
    assert backend.x("bot") > 0.5
    assert backend.x("crate") > 1.1
    # Its base joints report the velocities they hold.
    assert backend.world.joints("bot")[1][:, 1].tolist() == [1.0, 0.0, 0.0]


def test_static_geometry_stops_the_base(backend):
    backend.apply(JointCommand("bot", BASE_JOINTS, velocity=(0.0, 1.0, 0.0)))
    world = backend.step(200)
    # The wall's near face is at y=2.9; the toy is 0.1 half wide, kept 0.05 off by the controller.
    assert world.entity("bot").pose[1] == pytest.approx(2.75, abs=0.05)


def test_what_a_kinematic_base_cannot_do_is_refused(backend):
    backend.step()
    backend.apply(JointCommand("bot", ("neck",), position=(1.0,)))
    backend.step()
    # A kinematic base has no joints but its base joints, and the world said so rather than
    # dropping the command.
    assert any(key == REFUSED for _, key, _ in backend.received)


def test_a_robot_with_a_controller_is_refused_outright(scene):
    with pytest.raises(ValueError, match="control_period"):
        DimSimBackend(scene, [SimulatedRobot("bot", toy_robot(), control_period=0.02)])


def test_the_engine_is_reached_by_name_and_builds_its_world_when_it_runs(scene):
    """`sim_module("dimsim")` names the module and nothing reaches into it."""
    assert BACKENDS["dimsim"] == "dimos.sim2.dimsim.sim.DimSimSim"
    assert sim_module("dimsim", scene=scene) is not None
    sim = DimSimSim(scene=scene, robots=[SimulatedRobot("bot", toy_robot())])
    try:
        engine = sim._engine()
        assert engine.describe()["backend"] == "dimsim"
        assert sim.describe()["robots"] == ["bot"]
        engine.close()
    finally:
        sim.stop()
