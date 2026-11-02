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

from pathlib import Path
from typing import Any

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import shared_config
from dimos.sim2.module import Simulation
from dimos.sim2.spec import SimulatedRobot, SimulatedSensor


@pytest.fixture
def model(tmp_path: Path) -> Path:
    """The dog's model, with its camera's and imu's frames as sites; the scene owns any other frame."""
    path = tmp_path / "dog.xml"
    path.write_text(
        '<mujoco><worldbody><body name="base"><body name="head">'
        '<site name="camera_optical"/></body><site name="imu"/></body></worldbody></mujoco>'
    )
    return path


class _SimRobotConfig(ModuleConfig):
    model: Path
    x: float = 0.0


class _SimRobot(Module):
    config: _SimRobotConfig

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        return {
            "simulated": SimulatedRobot("dog", self.config.model, "base", xyz=(self.config.x, 0, 0))
        }


class _Bystander(Module):
    pass


def test_simulation_receives_descriptions_from_the_blueprint(model):
    blueprint = autoconnect(
        _SimRobot.blueprint(model=model, x=2.5), _Bystander.blueprint(), Simulation.blueprint()
    )
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    try:
        assert coordinator.get_instance(Simulation).describe() == {"robots": ["dog"], "sensors": []}
    finally:
        coordinator.stop()


def _simulation(**simulated: object) -> Simulation:
    simulation = Simulation()
    simulation.config_receive(tuple((name, {"simulated": d}) for name, d in simulated.items()))
    return simulation


def test_simulation_describes_its_world(model):
    simulation = _simulation(
        dog=SimulatedRobot("dog", model, "base"),
        cam=SimulatedSensor("camera", "camera_optical"),
        imu=SimulatedSensor("imu", "imu"),
        head=SimulatedSensor("lidar", "head"),
        overhead=SimulatedSensor("camera", "overhead"),
    )
    try:
        simulation.build()
        # A frame is the robot's whose model names it, as a site or a body; any other is the scene's.
        assert simulation.describe() == {
            "robots": ["dog"],
            "sensors": ["dog/camera_optical", "dog/head", "dog/imu", "world/overhead"],
        }
    finally:
        simulation.stop()


@pytest.mark.parametrize(
    "config", [{"simulatd": SimulatedRobot("dog", Path("dog.xml"), "base")}, {"simulated": 1}]
)
def test_a_malformed_description_is_rejected(config):
    simulation = Simulation()
    try:
        with pytest.raises(TypeError, match="must return"):
            simulation.config_receive((("dog", config),))
    finally:
        simulation.stop()


def test_duplicate_robots_are_rejected(model):
    simulation = _simulation(
        a=SimulatedRobot("dog", model, "base"), b=SimulatedRobot("dog", model, "base")
    )
    try:
        with pytest.raises(ValueError, match="duplicate"):
            simulation.build()
    finally:
        simulation.stop()
