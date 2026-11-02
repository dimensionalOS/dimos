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
        description = simulation.describe()
        assert description["robots"] == ["dog"]
        # A frame is the robot's whose model names it, as a site or a body; any other is the scene's.
        assert description["sensors"] == [
            "dog/camera_optical",
            "dog/head",
            "dog/imu",
            "world/overhead",
        ]
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
