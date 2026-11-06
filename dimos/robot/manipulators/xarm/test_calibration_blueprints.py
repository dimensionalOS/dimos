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
    XARM6_CHEST_CALIBRATION_HOME,
    xarm6_handeye_calibrate,
    xarm6_handeye_calibrate_chest,
    xarm6_handeye_calibrate_chest_sim,
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


def test_the_bench_is_a_table_box_and_the_view_is_on_by_default() -> None:
    parsed = BlueprintConfigParser(xarm6_handeye_calibrate).parse(
        [
            "--xarmconnection.ip",
            "192.168.1.2",
            "--realsensecamera.serial-number",
            "1",
            "--handeyecalibrationmodule.table-top-z",
            "0.08",
        ],
        environ={},
    )

    config = parsed.module_configs["handeyecalibrationmodule"]
    assert config["table_top_z"] == 0.08
    assert tuple(config["table_x_m"]) == (0.1, 1.0)
    assert parsed.module_configs["manipulationmodule"]["visualization"] == {"backend": "viser"}
    assert parsed.module_configs["manipulationmodule"].get("floor_z") is None


def test_the_chest_stacks_calibrate_a_fixed_camera_against_the_held_plate() -> None:
    real = _module_names(xarm6_handeye_calibrate_chest)
    sim = _module_names(xarm6_handeye_calibrate_chest_sim)

    assert {"xarmconnection", "realsensecamera", "handeyecalibrationmodule"} <= real
    assert {"mujocosimmodule", "handeyecalibrationmodule"} <= sim and "xarmconnection" not in sim
    parsed = BlueprintConfigParser(xarm6_handeye_calibrate_chest).parse(
        [
            "--xarmconnection.ip",
            "192.168.1.2",
            "--realsensecamera.serial-number",
            "1",
            "--handeyecalibrationmodule.nominal-translation",
            "[0.6, -0.5, 0.7]",
        ],
        environ={},
    )
    config = parsed.module_configs["handeyecalibrationmodule"]
    assert config["camera_frame"] == "base"
    assert tuple(config["nominal_translation"]) == (0.6, -0.5, 0.7)
    assert config["board"]["ids_from"] == 100 and config["board"]["squares_x"] == 5
    assert list(config["home_joints"]) == XARM6_CHEST_CALIBRATION_HOME


def test_the_shipped_wrist_result_is_a_passed_link6_mount() -> None:
    from dimos.manipulation.calibration.store import load_hand_eye_file
    from dimos.robot.manipulators.xarm.blueprints.grasp import XARM_CALIBRATION_DIR

    mount = load_hand_eye_file(XARM_CALIBRATION_DIR / "xarm6_wrist_camera.json")
    assert mount is not None
    assert (mount.frame_id, mount.child_frame_id) == ("link6", "camera_link")
    assert abs(mount.translation.x - 0.077) < 0.002
