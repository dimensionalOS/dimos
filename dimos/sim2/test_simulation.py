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
from dimos.sim2.spec import Mount, SimulatedRobot, SimulatedSensor

_MODEL = Path("robot.xml")


class _SimRobotConfig(ModuleConfig):
    x: float = 0.0


class _SimRobot(Module):
    config: _SimRobotConfig

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        return {"simulated": SimulatedRobot("dog", _MODEL, "base", xyz=(self.config.x, 0, 0))}


class _Bystander(Module):
    pass


def test_simulation_receives_descriptions_from_the_blueprint():
    blueprint = autoconnect(
        _SimRobot.blueprint(x=2.5), _Bystander.blueprint(), Simulation.blueprint()
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


def test_simulation_describes_its_world():
    simulation = _simulation(
        dog=SimulatedRobot("dog", _MODEL, "base"),
        cam=SimulatedSensor("cam", "camera", Mount("head"), robot_id="dog"),
        overhead=SimulatedSensor("overhead", "camera", Mount("world")),
    )
    try:
        simulation.build()
        assert simulation.describe() == {
            "robots": ["dog"],
            "sensors": ["dog/cam", "world/overhead"],
        }
    finally:
        simulation.stop()


@pytest.mark.parametrize(
    "config", [{"simulatd": SimulatedRobot("dog", _MODEL, "base")}, {"simulated": 1}]
)
def test_a_malformed_description_is_rejected(config):
    simulation = Simulation()
    try:
        with pytest.raises(TypeError, match="must return"):
            simulation.config_receive((("dog", config),))
    finally:
        simulation.stop()


@pytest.mark.parametrize(
    "simulated, error",
    [
        (
            {
                "a": SimulatedRobot("dog", _MODEL, "base"),
                "b": SimulatedRobot("dog", _MODEL, "base"),
            },
            "duplicate",
        ),
        ({"imu": SimulatedSensor("imu", "imu", Mount("base"), robot_id="ghost")}, "unknown robots"),
    ],
)
def test_invalid_descriptions_are_rejected(simulated, error):
    simulation = _simulation(**simulated)
    try:
        with pytest.raises(ValueError, match=error):
            simulation.build()
    finally:
        simulation.stop()
