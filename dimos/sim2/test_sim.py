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

"""What core is held to: it takes descriptions, it finds the world, and it knows no engine."""

from pathlib import Path
import subprocess
import sys

import pytest

from dimos.robot.assets.model import UsdModel
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.sim import BACKENDS, SimulatedWorld, sim_module
from dimos.sim2.spec import SimulatedRobot

TOY = Path(__file__).parent / "assets" / "toy.usda"


@pytest.fixture
def model() -> UsdModel:
    """The dog's model, with its camera's and imu's frames on it; the scene owns any other frame."""
    return toy_robot().with_frames(
        {
            "camera_optical": ("head", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            "imu": ("base", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        }
    )


def _world(**simulated: object) -> SimulatedWorld:
    world = SimulatedWorld()
    world.config_receive(tuple((name, {"simulated": d}) for name, d in simulated.items()))
    return world


def test_a_world_is_the_robots_it_was_told_about_and_nothing_more(model):
    """All core does with a description: keep each robot under the name of whoever sent it.

    Not "which of these is a camera", not "what should be drawn", and not which robot a sensor
    is bolted to. A sensor describes itself to nobody: it reads the composed scene, builds its
    own world from it, and renders from the observations -- so core never learns that anything
    is watching, and an engine is handed robots and a room.
    """
    world = _world(dog=SimulatedRobot("dog", model), other=SimulatedRobot("spot", model))
    try:
        assert {name: d.robot_id for name, d in world.simulated.items()} == {
            "dog": "dog",
            "other": "spot",
        }
    finally:
        world.stop()


@pytest.mark.parametrize(
    "config",
    [
        {"simulatd": SimulatedRobot("dog", toy_robot())},
        {"simulated": 1},
        # A sensor is not something a world is told about any more.
        {"simulated": "camera_optical"},
    ],
)
def test_a_malformed_description_is_rejected(config):
    world = SimulatedWorld()
    try:
        with pytest.raises(TypeError, match="must return"):
            world.config_receive((("dog", config),))
    finally:
        world.stop()


def test_duplicate_robots_are_rejected(model):
    # As the description arrives, rather than once an engine is compiling it: two robots called
    # one name is one set of topics and one tf tree, whatever is going to run them.
    world = SimulatedWorld()
    try:
        with pytest.raises(ValueError, match="duplicate"):
            world.config_receive(
                (
                    ("a", {"simulated": SimulatedRobot("dog", model)}),
                    ("b", {"simulated": SimulatedRobot("dog", model)}),
                )
            )
    finally:
        world.stop()


def test_the_world_is_usd_even_when_it_was_configured_as_a_file():
    """An engine here is only ever handed USD. A path is read, not passed on.

    This is the whole of criterion one, in core: there is no route through which a backend
    receives something that is not a USD layer, so "accepts only a USD" is not a convention the
    backends have to keep to.
    """
    world = SimulatedWorld(scene=TOY)
    try:
        model = world.world()
        assert isinstance(model, UsdModel)
        assert model.root_link == "base"
    finally:
        world.stop()


def test_a_world_nobody_described_is_no_world_at_all():
    # None is core saying "nothing here says what the world is"; what to do about that is the
    # engine's call, not a default scene core picked.
    world = SimulatedWorld(scene=None)
    try:
        assert world.world() is None
    finally:
        world.stop()


def test_an_unknown_engine_is_refused_by_name():
    with pytest.raises(ValueError, match="unknown simulation backend 'bullet'"):
        sim_module("bullet")


def test_core_names_its_engines_and_imports_none_of_them():
    """The only thing core knows about any engine is a string, and that is deliberate.

    Importing sim2's core must not import an engine: a sensor, a blueprint catalog or anything
    that only wants to know what a world is should not pay for MuJoCo, and core holding a module
    object rather than a name would mean it did. In its own process, because by the time this
    suite is running something else has long since imported one.
    """
    assert all(isinstance(path, str) for path in BACKENDS.values())
    code = (
        "import sys; import dimos.sim2.sim;"
        " assert not [m for m in sys.modules if m.startswith('mujoco')], sorted(sys.modules)"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
