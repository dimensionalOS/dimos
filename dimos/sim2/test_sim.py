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
from typing import Protocol

import pytest

from dimos.robot.assets.model import UsdModel
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.sim import (
    BACKENDS,
    SimulatedWorld,
    SimulatedWorldConfig,
    scene_from,
    sim_module,
)
from dimos.sim2.spec import SimulatedRobot, validate

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


def _config(*robots: SimulatedRobot) -> SimulatedWorldConfig:
    """The config a blueprint hands a backend, which is where a world is said to have robots."""
    return SimulatedWorldConfig(robots=robots)


def test_a_world_is_the_robots_it_was_configured_with_and_nothing_more(model):
    """All core says about a description: the robots the blueprint put in this world.

    Not "which of these is a camera", not "what should be drawn", not which robot a sensor is
    bolted to, and not what any robot's gains are -- those are in its layer. A sensor describes
    itself to nobody at all: it reads the composed scene, builds its own world from it, and
    renders from the observations.
    """
    config = _config(SimulatedRobot("dog", model), SimulatedRobot("spot", model))

    assert [d.robot_id for d in config.robots] == ["dog", "spot"]
    # Nothing about how to run them, beyond the name and how often the controller acts.
    assert config.robots[0].control_period is None


def test_duplicate_robots_are_rejected(model):
    # Two robots called one name is one set of topics and one tf tree, whatever runs them.
    with pytest.raises(ValueError, match="duplicate"):
        validate(_config(SimulatedRobot("dog", model), SimulatedRobot("dog", model)).robots)


def test_a_simulated_world_is_a_shape_rather_than_a_base_class():
    """So that a backend can be a native module, which cannot inherit from a Python class."""
    assert issubclass(type(SimulatedWorld), type(Protocol))
    assert set(SimulatedWorld.__annotations__) == {"actions", "world_observations"}


def test_the_world_is_usd_even_when_it_was_configured_as_a_file():
    """A backend here is only ever handed USD. A path is read, not passed on.

    This is the whole of criterion one, in core: there is no route through which a backend
    receives something that is not a USD layer, so "accepts only a USD" is not a convention the
    backends have to keep to.
    """
    model = scene_from(None, None, TOY)

    assert isinstance(model, UsdModel)
    assert model.root_link == "base"


def test_a_world_nobody_described_is_no_world_at_all():
    # None is core saying "nothing here says what the world is"; what to do about that is the
    # backend's call, not a default scene core picked.
    assert scene_from(None, None, None) is None


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
