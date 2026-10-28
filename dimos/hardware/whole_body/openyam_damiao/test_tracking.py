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

"""Software checks for the bounded OpenYAM torque policies."""

from itertools import count

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.openyam_damiao.adapter import OpenYamDamiaoAdapter
from dimos.hardware.whole_body.openyam_damiao.tracking import (
    J3_MOTION_COEFFICIENTS,
    BoundedHoldingCorrection,
    J3MotionCorrection,
)
from dimos.hardware.whole_body.openyam_damiao.trajectory import CosineJointTrajectory
from dimos.hardware.whole_body.spec import MotorCommand, MotorState


def test_holding_correction_is_bounded_and_freezes_inside_exit_band() -> None:
    policy = BoundedHoldingCorrection()
    error = np.zeros(6)
    error[2] = np.radians(1.0)
    velocity = np.zeros(6)
    base_torque = np.zeros(6)
    for _ in range(1000):
        policy.update(error, velocity, velocity, base_torque, 0.01, eligible=True)
    assert 0 < policy.bias[2] <= 5.0

    held = policy.bias.copy()
    error[2] = np.radians(0.12)
    for _ in range(1000):
        policy.update(error, velocity, velocity, base_torque, 0.01, eligible=True)
    np.testing.assert_array_equal(policy.bias, held)

    error[2] = np.radians(1.0)
    velocity[2] = 0.04
    policy.update(error, velocity, velocity, base_torque, 0.01, eligible=True)
    np.testing.assert_array_equal(policy.bias, held)


def test_holding_correction_stops_at_torque_guard() -> None:
    policy = BoundedHoldingCorrection()
    error = np.zeros(6)
    error[2] = np.radians(1.0)
    base_torque = np.zeros(6)
    base_torque[2] = 17.0
    for _ in range(100):
        policy.update(error, np.zeros(6), np.zeros(6), base_torque, 0.01, eligible=True)
    assert policy.bias[2] == 0.0


def test_openyam_adapter_requires_fresh_feedback_for_tracking(mocker: MockerFixture) -> None:
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False, tracking_comp=True)
    )
    adapter._last_arm_states = [MotorState(q=0.0, dq=0.0) for _ in range(6)]
    adapter._last_arm_state_time = 10.0
    ticks = count()
    mocker.patch(
        "dimos.hardware.whole_body.openyam_damiao.tracking_adapter.time.monotonic",
        side_effect=lambda: 10.01 + 0.01 * next(ticks),
    )
    commands = [MotorCommand(q=0.0, dq=0.0, kp=80.0, kd=5.0) for _ in range(6)]
    commands[2] = MotorCommand(q=0.03, dq=0.0, kp=80.0, kd=5.0)
    for _ in range(100):
        adapter._last_arm_state_time = 10.01 + 0.01 * next(ticks)
        correction = adapter._tracking_torques(commands, np.zeros(6))
    assert correction[2] > 0

    adapter._last_arm_state_time = 9.0
    np.testing.assert_array_equal(adapter._tracking_torques(commands, np.zeros(6)), np.zeros(6))
    assert adapter._holding_corrections["arm"].bias[2] == 0.0


def test_tracking_torque_reaches_motor_command(mocker: MockerFixture) -> None:
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False, tracking_comp=True)
    )
    adapter._connected = True
    adapter._active = True
    arm = mocker.Mock()
    adapter._arms = {"arm": arm}
    adapter._grippers = {"gripper": mocker.Mock()}
    adapter._robot = mocker.Mock()
    adapter._gravity_torques = mocker.Mock(return_value=np.zeros(6))
    adapter._last_arm_states = [MotorState(q=0.0, dq=0.0) for _ in range(6)]
    adapter._last_arm_state_time = 10.0
    ticks = count()
    mocker.patch(
        "dimos.hardware.whole_body.openyam_damiao.tracking_adapter.time.monotonic",
        side_effect=lambda: 10.01 + 0.01 * next(ticks),
    )
    commands = [MotorCommand(q=0.0, dq=0.0, kp=80.0, kd=5.0) for _ in range(6)]
    commands[2] = MotorCommand(q=0.03, dq=0.0, kp=80.0, kd=5.0)
    commands.append(MotorCommand(q=0.5))
    for _ in range(20):
        adapter._last_arm_state_time = 10.01 + 0.01 * next(ticks)
        assert adapter.write_motor_commands(commands)
    rows = arm.mit_control.call_args.args[0]
    assert rows[2, 4] > 0.0
    assert rows[0, 4] == 0.0


def test_j3_motion_model_enforces_tested_envelope_and_slew() -> None:
    policy = J3MotionCorrection(J3_MOTION_COEFFICIENTS["left"])
    policy.begin(0.0, 1)
    output = policy.update(0.28, 0.02, 0.0, 0.01, 6.0, 0.01)
    assert 0 < output <= 0.08
    with pytest.raises(ValueError, match="outside the measured envelope"):
        policy.update(0.5, 0.02, 0.0, 0.01, 6.0, 0.01)


def test_j3_motion_handoff_reaches_commands_and_preserves_delivered_torque(
    mocker: MockerFixture,
) -> None:
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(
            gravity_comp=False, tracking_comp=True, openyam_anvil_parity=True
        )
    )
    adapter._active = True
    adapter._connected = True
    adapter._arms = {"arm": mocker.Mock()}
    adapter._grippers = {"gripper": mocker.Mock()}
    adapter._robot = mocker.Mock()
    adapter._gravity_torques = mocker.Mock(return_value=np.zeros(6))
    adapter._last_arm_states = [MotorState(q=0.28, dq=0.0) for _ in range(6)]
    adapter._last_arm_state_time = 10.0
    mocker.patch(
        "dimos.hardware.whole_body.openyam_damiao.tracking_adapter.time.monotonic",
        return_value=10.01,
    )
    commands = [MotorCommand(q=0.28, dq=0.0, kp=80.0, kd=5.0) for _ in range(6)]
    commands[2] = MotorCommand(q=0.285, dq=0.02, ddq=0.0, kp=80.0, kd=5.0)
    commands.append(MotorCommand(q=0.5))

    adapter.begin_j3_motion("arm", "left", 1)
    assert adapter.write_motor_commands(commands)
    rows = adapter._arms["arm"].mit_control.call_args.args[0]
    assert rows[2, 3] == pytest.approx(0.016)
    assert rows[2, 4] > 0
    delivered = adapter._motion_corrections["arm"].delivered
    adapter.end_j3_motion("arm")
    assert adapter._holding_corrections["arm"].bias[2] == delivered
    assert not adapter._motion_corrections["arm"].active


def test_cosine_profile_reaches_endpoint_with_zero_velocity() -> None:
    profile = CosineJointTrajectory(0.18, 0.38, 0.04)
    assert profile.duration == 7.0
    assert profile.sample(0) == (0.18, 0.0, 0.0)
    q, velocity, acceleration = profile.sample(profile.duration)
    assert q == pytest.approx(0.38)
    assert abs(velocity) < 1e-12
    assert abs(acceleration) < 1e-12
    with pytest.raises(ValueError, match="measured envelope"):
        CosineJointTrajectory(0.02, 0.22, 0.04)


def test_motion_rejects_stale_feedback_before_sending_torque(mocker: MockerFixture) -> None:
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(
            gravity_comp=False, tracking_comp=True, openyam_anvil_parity=True
        )
    )
    adapter._active = True
    adapter._last_arm_states = [MotorState(q=0.28, dq=0.0) for _ in range(6)]
    adapter._last_arm_state_time = 1.0
    mocker.patch(
        "dimos.hardware.whole_body.openyam_damiao.tracking_adapter.time.monotonic",
        return_value=10.0,
    )
    adapter.begin_j3_motion("arm", "left", 1)
    commands = [MotorCommand(q=0.28, dq=0.0, ddq=0.0) for _ in range(6)]
    with pytest.raises(RuntimeError, match="stale"):
        adapter._tracking_torques(commands, np.zeros(6))
    assert adapter._holding_corrections["arm"].bias[2] == 0.0
