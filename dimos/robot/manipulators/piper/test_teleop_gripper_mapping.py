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

from unittest.mock import MagicMock

from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.task import ControlMode
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.robot.manipulators.piper.config import make_piper_hardware


def test_connected_piper_hardware_converts_normalized_gripper_to_native_position() -> None:
    component = make_piper_hardware(
        gripper_open_position=0.07,
        gripper_closed_position=0.0,
    )
    adapter = MagicMock(spec=ManipulatorAdapter)
    adapter.read_joint_positions.return_value = [0.0] * 6
    adapter.read_gripper_position.return_value = 0.0
    adapter.set_control_mode.return_value = True
    adapter.write_joint_positions.return_value = True
    adapter.write_gripper_position.return_value = True
    hardware = ConnectedHardware(adapter, component)

    assert hardware.write_command({"arm/gripper": 0.0}, ControlMode.POSITION)
    assert hardware.write_command({"arm/gripper": 1.0}, ControlMode.POSITION)

    assert adapter.write_gripper_position.call_args_list == [
        ((0.0,), {}),
        ((0.07,), {}),
    ]
