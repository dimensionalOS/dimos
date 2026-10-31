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

import pytest

from dimos.sim2.module import Simulation
from dimos.sim2.spec import Mount, SimulatedRobot, SimulatedSensor

_MODEL = Path("robot.xml")


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
        description = simulation.describe()
        assert description["robots"] == ["dog"]
        assert description["sensors"] == ["dog/cam", "world/overhead"]
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
