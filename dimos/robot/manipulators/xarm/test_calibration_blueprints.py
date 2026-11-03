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

from __future__ import annotations

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.robot.manipulators.xarm.blueprints.calibration import (
    XARM6_CALIBRATION_HOME,
    xarm6_handeye_calibrate,
    xarm6_handeye_calibrate_sim,
)


def _module_names(blueprint) -> set[str]:  # type: ignore[no-untyped-def]
    return {atom.name.lower() for atom in blueprint.active_blueprints}


def test_real_stack_composes_planner_arm_camera_and_calibration() -> None:
    names = _module_names(xarm6_handeye_calibrate)

    assert {
        "manipulationmodule",
        "controlcoordinator",
        "xarmconnection",
        "realsensecamera",
        "handeyecalibrationmodule",
    } <= names


def test_sim_stack_replaces_the_arm_and_camera_with_mujoco() -> None:
    names = _module_names(xarm6_handeye_calibrate_sim)

    assert {
        "manipulationmodule",
        "controlcoordinator",
        "mujocosimmodule",
        "handeyecalibrationmodule",
    } <= names
    assert "xarmconnection" not in names
    assert "realsensecamera" not in names


def test_calibration_fields_override_from_the_command_line() -> None:
    parsed = BlueprintConfigParser(xarm6_handeye_calibrate_sim).parse(
        ["--handeyecalibrationmodule.max-poses", "5", "--handeyecalibrationmodule.dry-run", "true"],
        environ={},
    )

    config = parsed.module_configs["handeyecalibrationmodule"]
    assert config["max_poses"] == 5
    assert config["dry_run"] is True
    assert list(config["home_joints"]) == XARM6_CALIBRATION_HOME
    assert config["optical_frame"] == "wrist_camera_color_optical_frame"
