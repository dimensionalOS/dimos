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

import json
from pathlib import Path

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.sim2.actions import (
    Attach,
    Detach,
    Interact,
    JointCommand,
    Remove,
    SetVelocity,
    Spawn,
)
from dimos.sim2.dimsim_backend import DimsimBackend
from dimos.sim2.module import SimulationConfig
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world_change import EVENTS, OBJECT, WorldMirror


@pytest.fixture
def scene(tmp_path: Path) -> Path:
    path = tmp_path / "scene.xml"
    path.write_text("""
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 .1"/>
    <geom name="wall" type="box" pos="0 3 .5" size="2 .1 .5"/>
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
        self.events: list = []

    def apply(self, *actions) -> None:
        self.backend.apply(actions)

    def step(self, steps: int = 1) -> WorldMirror:
        self.world.apply(self.backend.step(steps))
        self.events += self.world.events
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


def test_the_base_pushes_a_spawned_box(backend):
    backend.apply(
        Spawn("box", "box", (0.2, 0.2, 0.2), Pose(1.0, 0.0, 0.2), mass=2.0),
        SetVelocity(
            "bot",
            Twist(linear=[1.0, 0.0, 0.0]),
        ),
    )
    backend.step()
    assert backend.x("box") == pytest.approx(1.0, abs=0.01)
    for _ in range(100):
        backend.step()
    assert backend.x("box") > 1.1
    assert backend.x("bot") > 0.5


def test_static_geometry_stops_the_base(backend):
    backend.apply(SetVelocity("bot", Twist(linear=[0.0, 1.0, 0.0])))
    world = backend.step(200)
    # The wall's near face is at y=2.9; the base is 0.15 half wide, kept 0.05 off by the controller.
    assert world.entity("bot").pose[1] == pytest.approx(2.7, abs=0.03)


def test_spawn_and_remove_change_the_scene_and_mujoco_actions_are_refused(backend):
    first = json.loads(backend.backend.scene_data())
    backend.apply(Spawn("ball", "sphere", (0.1,), Pose(0.0, -1.0, 0.5), mass=1.0))
    spawned = json.loads(backend.backend.scene_data())
    assert spawned["id"] != first["id"]
    assert {"world", "ball"} <= {body["name"] for body in spawned["bodies"]}
    assert [e.path for e in backend.step().tagged(OBJECT)] == ["ball"]
    backend.apply(Remove("ball"), JointCommand("bot", ("neck",), position=(1.0)))
    assert backend.step().entity("ball") is None
    assert "ball" not in {
        body["name"] for body in json.loads(backend.backend.scene_data())["bodies"]
    }
    # A kinematic base has no joints to drive, and the world said so.
    assert any(event == EVENTS["events/refused"] for _, event, _ in backend.events)


def test_attach_carries_a_box_with_the_base_until_detached(backend):
    # Held off to the side, so that let go it drops to the floor rather than onto the base.
    backend.apply(Spawn("box", "box", (0.1, 0.1, 0.1), Pose(0.0, 0.8, 0.6), mass=1.0))
    backend.step()
    backend.apply(Attach("box", "bot"), SetVelocity("bot", Twist(linear=[1.0, 0.0, 0.0])))
    carried = backend.step(50)
    assert backend.x("box") == pytest.approx(backend.x("bot"), abs=0.02)
    assert carried.entity("box").pose[2] == pytest.approx(0.6, abs=0.02)
    backend.apply(Detach("box"), Interact("box", "open"))
    dropped = backend.step(50)
    # Let go, it falls onto the floor and stays behind as the base drives on.
    assert dropped.entity("box").pose[2] < 0.15
    assert backend.x("bot") > backend.x("box") + 0.5
