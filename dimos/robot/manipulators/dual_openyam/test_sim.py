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

from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_JOINTS
from dimos.robot.manipulators.dual_openyam.sim import (
    DUAL_OPENYAM_MJCF_ACTUATORS,
    DUAL_OPENYAM_MJCF_JOINTS,
    dual_openyam_sim_spec,
)


def test_sim_spec_binds_every_hardware_joint_including_the_grippers() -> None:
    spec = dual_openyam_sim_spec(tuple(DUAL_OPENYAM_JOINTS))

    assert spec.hardware_joints == tuple(DUAL_OPENYAM_JOINTS)
    assert spec.model_joint_names == DUAL_OPENYAM_MJCF_JOINTS
    assert spec.model_actuator_names == DUAL_OPENYAM_MJCF_ACTUATORS
    # Grippers are ordinary trailing joints, not the single gripper SHM slot.
    assert spec.model_joint_names[-2:] == ("left_left_finger", "right_left_finger")
    assert spec.require_imu is False
    assert spec.require_floating_base is False


def test_sim_spec_rejects_a_joint_list_the_mjcf_cannot_bind() -> None:
    with pytest.raises(ValueError, match="14 hardware"):
        dual_openyam_sim_spec(tuple(DUAL_OPENYAM_JOINTS[:-1]))
