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

import math

import pytest

from dimos.simulation.behavior.control import RobotControl
from dimos.simulation.behavior.types import ControlMode


def test_transfer_discards_commands_and_holds_measured_position():
    control = RobotControl(0.2)
    control.set_velocity((1, 0, 0), 1)
    control.set_joints(["arm"], [0.8], {"arm": (-1, 1)})
    control.transfer(ControlMode.PRIMITIVE, {"arm": 0.2})
    control.set_velocity((2, 0, 0), 2)
    control.set_joints(["arm"], [0.9], {"arm": (-1, 1)})
    control.transfer(ControlMode.DIMOS, {"arm": 0.3})
    assert control.get_velocity(2) == (0, 0, 0)
    assert control.targets == {"arm": 0.3}


@pytest.mark.parametrize(
    "names,values",
    [(["unknown"], [0]), (["a"], [2]), (["a", "a"], [0, 1]), (["a"], []), (["a"], [math.nan])],
)
def test_invalid_joint_update_is_atomic(names, values):
    control = RobotControl(0.2)
    control.transfer(ControlMode.DIMOS, {"a": 0.1})
    with pytest.raises(ValueError):
        control.set_joints(names, values, {"a": (-1, 1)})
    assert control.targets == {"a": 0.1}


def test_native_and_velocity_commands_expire():
    control = RobotControl(0.2)
    control.set_velocity((1, 2, 3), 10)
    assert control.get_velocity(10.1) == (1, 2, 3)
    assert control.get_velocity(10.3) == (0, 0, 0)
    control.transfer(ControlMode.NATIVE, {})
    control.set_action([0.5], [(-1, 1)], 20)
    assert control.get_action(20.1) == [0.5]
    assert control.get_action(20.3) is None


def test_native_action_validation_preserves_last_valid_command():
    control = RobotControl(0.2)
    control.transfer(ControlMode.NATIVE, {})
    control.set_action([0.3], [(-1, 1)], 1)
    with pytest.raises(ValueError):
        control.set_action([float("inf")], [(-1, 1)], 1)
    assert control.get_action(1) == [0.3]
