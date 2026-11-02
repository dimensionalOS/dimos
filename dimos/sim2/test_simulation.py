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

_MODEL = Path("robot.xml")
# The dog's URDF: a head on a neck joint, a camera fixed 10 cm ahead of the head and an imu on the (renamed) root.
_URDF = """<robot name="dog">
  <link name="base_link"/><link name="head"/><link name="camera_mount"/><link name="camera_optical"/><link name="imu"/>
  <joint name="neck" type="revolute"><origin xyz="0.3 0 0.1"/><parent link="base_link"/><child link="head"/></joint>
  <joint name="m" type="fixed"><origin xyz="0.1 0 0"/><parent link="head"/><child link="camera_mount"/></joint>
  <joint name="o" type="fixed"><origin rpy="0 0 1.5"/><parent link="camera_mount"/><child link="camera_optical"/></joint>
  <joint name="i" type="fixed"><origin xyz="0 0 0.05"/><parent link="base_link"/><child link="imu"/></joint>
</robot>"""
_DESCRIPTION = {"robot_description": {"name": "dog", "urdf": _URDF, "formats": {"mjcf": "dog.xml"}}}


def _simulation(**simulated: object) -> Simulation:
    simulation = Simulation()
    simulation.config_receive(
        (
            *((name, {"simulated": d}) for name, d in simulated.items()),
            ("urdf_manager", _DESCRIPTION),
        )
    )
    return simulation


def test_simulation_describes_its_world():
    simulation = _simulation(
        dog=SimulatedRobot("dog", "base", _MODEL),
        cam=SimulatedSensor("camera", "camera_optical"),
        overhead=SimulatedSensor("camera", "overhead"),
    )
    try:
        description = simulation.describe()
        assert description["robots"] == ["dog"]
        assert description["sensors"] == ["dog/camera_optical", "world/overhead"]
    finally:
        simulation.stop()


def test_sensors_sit_at_their_urdf_frame_on_the_body_holding_it_and_a_robot_takes_its_model_from_it():
    simulation = _simulation(
        dog=SimulatedRobot("dog", "base"),
        cam=SimulatedSensor("camera", "camera_optical"),
        imu=SimulatedSensor("imu", "imu"),
        overhead=SimulatedSensor("camera", "overhead"),
    )
    try:
        dog, cam, imu, overhead = (
            simulation.simulated[n] for n in ("dog", "cam", "imu", "overhead")
        )
        assert dog.model == Path("dog.xml")
        # Up the fixed joints to the head, which the neck moves.
        assert (cam.robot_id, cam.body) == ("dog", "head")
        assert cam.xyz == pytest.approx((0.1, 0, 0)) and cam.rpy == pytest.approx((0, 0, 1.5))
        # The URDF's root is the robot's root body, whatever the URDF calls it.
        assert (imu.robot_id, imu.body, imu.xyz) == ("dog", "base", pytest.approx((0, 0, 0.05)))
        assert (overhead.robot_id, overhead.body) == (None, "overhead")
    finally:
        simulation.stop()


@pytest.mark.parametrize(
    "config", [{"simulatd": SimulatedRobot("dog", "base", _MODEL)}, {"simulated": 1}]
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
                "a": SimulatedRobot("dog", "base", _MODEL),
                "b": SimulatedRobot("dog", "base", _MODEL),
            },
            "duplicate",
        ),
    ],
)
def test_invalid_descriptions_are_rejected(simulated, error):
    simulation = _simulation(**simulated)
    try:
        with pytest.raises(ValueError, match=error):
            simulation.build()
    finally:
        simulation.stop()


def test_a_robot_needs_a_model_or_a_robot_description_naming_one():
    simulation = Simulation()
    try:
        with pytest.raises(ValueError, match="no model"):
            simulation.config_receive((("cat", {"simulated": SimulatedRobot("cat", "base")}),))
    finally:
        simulation.stop()
