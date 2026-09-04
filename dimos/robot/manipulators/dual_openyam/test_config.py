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

import pytest

from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_ADAPTER_TYPE,
    DUAL_OPENYAM_ARM_JOINTS,
    DUAL_OPENYAM_HOME_JOINTS,
    DUAL_OPENYAM_JOINTS,
    dual_openyam_hardware,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.openyam.config import OPENYAM_HOME_JOINTS


def test_dual_openyam_model_has_canonical_groups_and_reference_posture() -> None:
    config = dual_openyam_model_config()

    assert config.joint_names == DUAL_OPENYAM_ARM_JOINTS
    assert config.home_joints == DUAL_OPENYAM_HOME_JOINTS
    assert config.max_velocity == pytest.approx(2.0)
    assert [(group.name, group.tip_link) for group in config.planning_groups] == [
        ("left_manipulator", "left_grasp_frame"),
        ("right_manipulator", "right_grasp_frame"),
    ]


def test_dual_openyam_reuses_the_single_arm_home_posture() -> None:
    assert DUAL_OPENYAM_HOME_JOINTS == [*OPENYAM_HOME_JOINTS, *OPENYAM_HOME_JOINTS]


def test_dual_openyam_hardware_defaults_to_complete_mock() -> None:
    hardware = dual_openyam_hardware()

    assert hardware.hardware_id == "dual_openyam"
    assert hardware.adapter_type == "mock_whole_body"
    assert hardware.joints == DUAL_OPENYAM_JOINTS
    assert hardware.adapter_kwargs["initial_positions"] == [
        *DUAL_OPENYAM_HOME_JOINTS,
        0.0,
        0.0,
    ]


def test_dual_openyam_hardware_uses_both_explicit_can_ports() -> None:
    hardware = dual_openyam_hardware(left_can_port="can8", right_can_port="can9")

    assert hardware.adapter_type == DUAL_OPENYAM_ADAPTER_TYPE
    runtime = hardware.adapter_kwargs["runtime_config"]
    assert isinstance(runtime, DamiaoRuntimeConfig)
    assert runtime.bus_devices == {"left": "can8", "right": "can9"}
    assert runtime.gravity_comp is True


@pytest.mark.parametrize(
    ("left", "right"),
    [("can8", None), (None, "can9")],
)
def test_dual_openyam_hardware_rejects_partial_can_configuration(
    left: str | None,
    right: str | None,
) -> None:
    with pytest.raises(ValueError, match="requires both"):
        dual_openyam_hardware(left_can_port=left, right_can_port=right)


def test_dual_openyam_hardware_rejects_duplicate_can_configuration() -> None:
    with pytest.raises(ValueError, match="requires distinct"):
        dual_openyam_hardware(left_can_port="same", right_can_port="same")
