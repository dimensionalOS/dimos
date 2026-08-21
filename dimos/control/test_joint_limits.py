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

from types import SimpleNamespace

import pytest

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.joint_limits import resolve_velocity_limits
from dimos.hardware.manipulators.spec import JointLimits


def test_resolver_uses_adapter_limits_scale_and_component_override() -> None:
    component = HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.MANIPULATOR,
        joints=["arm/j1", "arm/j2"],
        gripper_joints=["arm/gripper"],
        joint_velocity_limits={"arm/j2": 2.0, "arm/gripper": 0.4},
    )
    adapter = SimpleNamespace(
        get_limits=lambda: JointLimits(
            position_lower=[-1.0, -1.0],
            position_upper=[1.0, 1.0],
            velocity_max=[4.0, 5.0],
        )
    )
    connected = SimpleNamespace(component=component, adapter=adapter)

    resolved = resolve_velocity_limits(
        ["arm/j1", "arm/j2", "arm/gripper"],
        {"arm": connected},
        speed_scale=0.5,
    )

    assert resolved == {"arm/j1": 2.0, "arm/j2": 1.0, "arm/gripper": 0.2}


@pytest.mark.parametrize("speed_scale", [0.0, -1.0, 1.1, float("inf"), float("nan")])
def test_resolver_rejects_invalid_speed_scale(speed_scale: float) -> None:
    with pytest.raises(ValueError, match="speed_scale"):
        resolve_velocity_limits([], {}, speed_scale=speed_scale)
