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

"""Every Damiao arm has a calibration blueprint: its connection with config writes, and the calibration module."""

from __future__ import annotations

import pytest

from dimos.control.calibration.module import CalibrationModule
from dimos.control.calibration.robots import KNOWN_ROBOTS
from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import resolve_module_refs, stream_name_types
from dimos.core.global_config import global_config
from dimos.robot.all_blueprints import all_blueprints, all_modules
from dimos.robot.get_all_blueprints import load_blueprint
from dimos.robot.manipulators.damiao.connection import DamiaoConnection
from dimos.robot.manipulators.damiao.damiao_blueprint import arm_connection
from dimos.robot.manipulators.dual_makermods_metal.connection import DualMetalConnection
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection
from dimos.robot.manipulators.openarm.connection import OpenArmConnection
from dimos.robot.manipulators.openyam.connection import OpenYamConnection
from dimos.robot.manipulators.seeed_b601_dm.connection import B601DMConnection

# Blueprint: its connection, the robot it calibrates, and the arm it picks.
CALIBRATE: dict[str, tuple[type[DamiaoConnection], str, str | None]] = {
    "openyam-calibrate": (OpenYamConnection, "openyam", None),
    "dual-openyam-calibrate": (DualOpenYamConnection, "dual-openyam", "left"),
    "metal-calibrate": (MetalConnection, "metal", None),
    "dual-metal-calibrate": (DualMetalConnection, "dual-metal", None),
    "b601-dm-calibrate": (B601DMConnection, "b601-dm", None),
    "openarm-calibrate": (OpenArmConnection, "openarm", None),
}


def test_every_known_robot_has_one() -> None:
    assert sorted(robot for _, robot, _ in CALIBRATE.values()) == sorted(KNOWN_ROBOTS)
    assert all_modules["calibration-module"] == "dimos.control.calibration.module.CalibrationModule"


@pytest.mark.parametrize("name", sorted(CALIBRATE))
def test_the_module_commands_the_connection_directly(name: str) -> None:
    connection, robot, arm = CALIBRATE[name]
    assert name in all_blueprints

    blueprint = autoconnect(load_blueprint(name)).solidify()

    assert {atom.module for atom in blueprint.blueprints} == {connection, CalibrationModule}
    [driver] = [atom for atom in blueprint.blueprints if atom.module is connection]
    assert driver.kwargs["config_writes"] is True and driver.kwargs["teach"] is False
    [calibration] = [atom for atom in blueprint.blueprints if atom.module is CalibrationModule]
    assert calibration.kwargs == {"robot": robot, **({"arm": arm} if arm else {})}
    # The module's targets go to the connection's command port, its readings come back.
    streams = dict(stream_name_types(blueprint))
    assert {"position_command", "joint_state"} <= set(streams)
    assert resolve_module_refs(blueprint) == {(calibration.name, "_connection"): driver.name}


def test_calibration_never_runs_on_the_simulated_connection(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    with pytest.raises(ValueError, match="calibration drives the real arm"):
        arm_connection(OpenYamConnection, config_writes=True)
    [mock] = arm_connection(OpenYamConnection).blueprints
    assert mock.module is MockJointConnection
