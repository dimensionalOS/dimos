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

from dimos.control.components import HardwareComponent, HardwareType, make_joints


def test_hardware_component_does_not_classify_joint_semantics() -> None:
    component = HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.MANIPULATOR,
        joints=[*make_joints("arm", 6), "arm/tool_joint"],
    )

    assert not hasattr(component, "arm_joints")
    assert not hasattr(component, "gripper_joints")
    assert not hasattr(component, "gripper_dof")
