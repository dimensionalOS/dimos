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

from pathlib import Path

import pytest

from dimos.sim2.actions import BASE_VELOCITY, CborKeyValueCommand, JointCommand
from dimos.sim2.dimsim_backend import DimsimBackend
from dimos.sim2.module import SimulationConfig
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world_observations import REFUSED, WorldMirror


@pytest.fixture
def scene(tmp_path: Path) -> Path:
    path = tmp_path / "scene.xml"
    path.write_text("""
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 .1"/>
    <geom name="wall" type="box" pos="0 3 .5" size="2 .1 .5"/>
    <body name="box" pos="1 0 .2"><freejoint/><geom type="box" size=".2 .2 .2" mass="2"/></body>
  </worldbody>
</mujoco>
""")
    return path


@pytest.fixture
def robot_model(tmp_path: Path) -> Path:
    path = tmp_path / "base.xml"
    path.write_text("""
<mujoco>
  <worldbody>
    <body name="base"><freejoint/><geom type="box" size=".3 .15 .15" mass="15"/></body>
  </worldbody>
</mujoco>
""")
    return path


class _Stepped:
    """The backend, stepped by hand, and the world as its frames say."""

    def __init__(self, backend: DimsimBackend) -> None:
        self.backend, self.world = backend, WorldMirror()
        self.received: list = []

    def apply(self, *actions) -> None:
        self.backend.apply(actions)

    def step(self, steps: int = 1) -> WorldMirror:
        self.world.apply(self.backend.step(steps))
        self.received += self.world.received
        return self.world

    def x(self, name: str) -> float:
        return float(self.world.entity(name).pose[0])


@pytest.fixture
def backend(scene, robot_model, tmp_path):
    backend = DimsimBackend(
        SimulationConfig(scene=scene), {"bot": SimulatedRobot("bot", robot_model, "base")}, tmp_path
    )
    yield _Stepped(backend)
    backend.close()


def test_the_base_pushes_a_box(backend):
    backend.apply(CborKeyValueCommand("bot", BASE_VELOCITY, [1.0, 0.0, 0.0]))
    backend.step()
    assert backend.x("box") == pytest.approx(1.0, abs=0.01)
    for _ in range(100):
        backend.step()
    assert backend.x("box") > 1.1
    assert backend.x("bot") > 0.5
    assert backend.world.entity("bot").cbor[BASE_VELOCITY] == [1.0, 0.0, 0.0]


def test_static_geometry_stops_the_base(backend):
    backend.apply(CborKeyValueCommand("bot", BASE_VELOCITY, [0.0, 1.0, 0.0]))
    world = backend.step(200)
    # The wall's near face is at y=2.9; the base is 0.15 half wide, kept 0.05 off by the controller.
    assert world.entity("bot").pose[1] == pytest.approx(2.7, abs=0.03)


def test_what_a_kinematic_base_cannot_do_is_refused(backend):
    backend.step()
    backend.apply(JointCommand("bot", ("neck",), position=(1.0,)))
    backend.step()
    # A kinematic base has no joints to drive, and the world said so.
    assert any(key == REFUSED for _, key, _ in backend.received)
