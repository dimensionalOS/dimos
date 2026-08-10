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

"""GRIPPER-SPEC 3.5 regression: gripper values reach the adapter unchanged."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.task import ControlMode
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.robot.manipulators.piper.config import make_piper_hardware


def _connected() -> tuple[ConnectedHardware, MagicMock]:
    component = make_piper_hardware()
    adapter = MagicMock(spec=ManipulatorAdapter)
    adapter.read_joint_positions.return_value = [0.0] * 7
    adapter.read_joint_velocities.return_value = [0.0] * 7
    adapter.read_joint_efforts.return_value = [0.0] * 7
    adapter.set_control_mode.return_value = True
    adapter.write_joint_positions.return_value = True
    return ConnectedHardware(adapter, component), adapter


@pytest.mark.parametrize("commanded", [0.0, 0.04, 0.08])
def test_gripper_value_reaches_the_adapter_unchanged(commanded: float) -> None:
    hardware, adapter = _connected()

    assert hardware.write_command({"arm/gripper": commanded}, ControlMode.POSITION)

    sent = adapter.write_joint_positions.call_args.args[0]
    assert sent[-1] == pytest.approx(commanded), (
        f"wrapper altered the gripper value: emitted {commanded}, adapter got {sent[-1]}"
    )


def test_one_call_carries_arm_and_gripper_together() -> None:
    hardware, adapter = _connected()
    arm = {f"arm/joint{i + 1}": 0.1 * (i + 1) for i in range(6)}

    assert hardware.write_command({**arm, "arm/gripper": 0.06}, ControlMode.POSITION)

    adapter.write_joint_positions.assert_called_once()
    sent = adapter.write_joint_positions.call_args.args[0]
    assert sent == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.06])
    assert (
        not hasattr(adapter, "write_gripper_position") or not adapter.write_gripper_position.called
    )


def test_read_round_trips_the_gripper_unconverted() -> None:
    hardware, adapter = _connected()
    adapter.read_joint_positions.return_value = [0.0] * 6 + [0.055]

    assert hardware.read_state()["arm/gripper"].position == pytest.approx(0.055)


def test_velocity_writes_carry_all_joints() -> None:
    hardware, adapter = _connected()
    adapter.write_joint_velocities.return_value = True

    assert hardware.write_command(
        {f"arm/joint{i + 1}": 0.0 for i in range(6)}, ControlMode.VELOCITY
    )

    assert len(adapter.write_joint_velocities.call_args.args[0]) == 7
