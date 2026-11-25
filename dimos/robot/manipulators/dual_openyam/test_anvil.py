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

"""The torques, limits and gains the Anvil-tuned dual OpenYAM is driven with."""

import numpy as np
import pytest

from dimos.robot.manipulators.damiao.arm import PhysicalModel
from dimos.robot.manipulators.dual_openyam.anvil import (
    ANVIL_OPENYAM_HOME,
    ANVIL_OPENYAM_KD,
    ANVIL_OPENYAM_KP,
    DUAL_OPENYAM_ANVIL,
    anvil_dual_openyam_model_config,
)

# Holding torques, N m, of Anvil's composed OpenYAM v2 model with its gripper,
# as its SDK's own URDF gives them.
ZERO_POSE_TORQUES = [0.0, 2.3287, 9.0162, 2.0535, 0.0, -0.0067]
HOME_POSE_TORQUES = [0.0, 0.5924, 8.9724, 2.0535, 0.0, -0.0067]


@pytest.fixture(scope="module")
def physical() -> PhysicalModel:
    return PhysicalModel(
        DUAL_OPENYAM_ANVIL.gravity_model, DUAL_OPENYAM_ANVIL.arm_joint_names, gravity_comp=True
    )


def test_gravity_matches_anvil_at_the_zero_pose_on_both_arms(physical: PhysicalModel) -> None:
    torques = physical.gravity(np.zeros(12))

    assert torques[:6] == pytest.approx(ZERO_POSE_TORQUES, abs=1e-4)
    assert torques[6:] == pytest.approx(ZERO_POSE_TORQUES, abs=1e-4)


def test_gravity_follows_each_arm_s_own_joints(physical: PhysicalModel) -> None:
    """A swapped side or a degrees-for-radians slip would move the wrong arm's torques."""
    left_home = np.array([*ANVIL_OPENYAM_HOME, *[0.0] * 6])

    torques = physical.gravity(left_home)

    assert torques[:6] == pytest.approx(HOME_POSE_TORQUES, abs=1e-4)
    assert torques[6:] == pytest.approx(ZERO_POSE_TORQUES, abs=1e-4)


def test_gravity_reverses_on_joint2_once_the_upper_arm_passes_vertical(
    physical: PhysicalModel,
) -> None:
    """Joint 2's sign: its torque is positive folded and negative once raised past upright."""
    raised = np.zeros(12)
    raised[1] = np.radians(60.0)

    assert physical.gravity(np.zeros(12))[1] > 0.0
    assert physical.gravity(raised)[1] == pytest.approx(-0.8219, abs=1e-4)


def test_each_bus_is_held_with_anvil_gains() -> None:
    for bus in DUAL_OPENYAM_ANVIL.buses:
        assert bus.kp == ANVIL_OPENYAM_KP
        assert bus.kd == ANVIL_OPENYAM_KD


def test_models_carry_anvil_limits_under_both_namings() -> None:
    control = {joint.name: joint for joint in DUAL_OPENYAM_ANVIL.model.load().joints}
    planning = {
        joint.name: joint
        for joint in anvil_dual_openyam_model_config(robot="arm").model.load().joints
    }

    # Joint 6 is where the DimOS model allowed 15 degrees more than the hardware.
    for joint in (control["left_joint6"], planning["arm/right_joint6"]):
        assert (joint.lower, joint.upper) == pytest.approx((-2.35619, 1.8326))
        assert joint.velocity == pytest.approx(np.radians(720.0))
    assert control["right_joint3"].upper == pytest.approx(4.01426)
    assert planning["arm/left_joint1"].velocity == pytest.approx(np.radians(216.0))
