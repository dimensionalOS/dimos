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

"""Every ``<robot>-calibrate`` blueprint: its connection with config writes, and the calibration module with its robot."""

from __future__ import annotations

import pytest

from dimos.control.calibration._offline_models import require_model
from dimos.control.calibration.module import CalibrationModule, CalibrationModuleConfig
from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom, autoconnect
from dimos.core.coordination.module_coordinator import resolve_module_refs, stream_name_types
from dimos.core.global_config import global_config
from dimos.protocol.encode.cbor import decode, encode
from dimos.robot.all_blueprints import all_blueprints, all_modules
from dimos.robot.get_all_blueprints import load_blueprint
from dimos.robot.manipulators.damiao.connection import DamiaoConnection
from dimos.robot.manipulators.damiao.damiao_blueprint import arm_connection
from dimos.robot.manipulators.openyam.connection import OpenYamConnection

CALIBRATE = sorted(name for name in all_blueprints if name.endswith("-calibrate"))
BENCH = {"floor_z_m": 0.0, "min_x_m": 0.1}


def atoms(name: str) -> tuple[Blueprint, BlueprintAtom, BlueprintAtom]:
    """The solid blueprint, its connection and its calibration module."""
    blueprint = autoconnect(load_blueprint(name)).solidify()
    [driver] = [atom for atom in blueprint.blueprints if issubclass(atom.module, DamiaoConnection)]
    [calibration] = [atom for atom in blueprint.blueprints if atom.module is CalibrationModule]
    assert len(blueprint.blueprints) == 2
    return blueprint, driver, calibration


def test_the_calibration_module_is_registered() -> None:
    assert CALIBRATE
    assert all_modules["calibration-module"] == "dimos.control.calibration.module.CalibrationModule"


@pytest.mark.parametrize("name", CALIBRATE)
def test_the_module_commands_the_connection_directly(name: str) -> None:
    blueprint, driver, calibration = atoms(name)

    assert driver.kwargs["config_writes"] is True and driver.kwargs["teach"] is False
    # The module's targets go to the connection's command port, its readings come back.
    streams = dict(stream_name_types(blueprint))
    assert {"position_command", "joint_state"} <= set(streams)
    assert resolve_module_refs(blueprint) == {(calibration.name, "_connection"): driver.name}


@pytest.mark.parametrize("name", CALIBRATE)
def test_the_blueprint_is_named_after_the_robot_it_hands_the_module(name: str) -> None:
    _blueprint, _driver, calibration = atoms(name)
    robot = calibration.kwargs["robot"]

    assert name == f"{robot.name.replace('_', '-')}-calibrate"
    # It reaches the module's worker as a constructor argument, never a flag.
    # Compared encoded, since comparing the robots would pull their models.
    assert encode(decode(encode(robot))) == encode(robot)
    assert "robot" not in CalibrationModuleConfig.model_fields


@pytest.mark.parametrize("name", CALIBRATE)
def test_every_arm_of_the_robot_resolves_a_spec(name: str) -> None:
    blueprint, _driver, calibration = atoms(name)
    robot = calibration.kwargs["robot"]
    require_model(robot.gravity_model, name)
    settings = {key: value for key, value in calibration.kwargs.items() if key != "robot"}

    for arm_id in robot.arm_ids():
        config = CalibrationModuleConfig(**{**settings, **BENCH, "arm": arm_id.rsplit("/", 1)[-1]})
        assert config.arm_spec(robot).arm_id == arm_id
    if len(robot.arm_ids()) > 1 and "arm" not in settings:
        with pytest.raises(ValueError, match="several arms"):
            CalibrationModuleConfig(**settings, **BENCH).arm_spec(robot)
    with pytest.raises(ValueError, match="has no arm 'up'"):
        CalibrationModuleConfig(**{**settings, **BENCH, "arm": "up"}).arm_spec(robot)


def test_calibration_never_runs_on_the_simulated_connection(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    with pytest.raises(ValueError, match="calibration drives the real arm"):
        arm_connection(OpenYamConnection, config_writes=True)
    [mock] = arm_connection(OpenYamConnection).blueprints
    assert mock.module is MockJointConnection
