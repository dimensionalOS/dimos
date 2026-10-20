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


from dimos.imitation.policy.dreamlabs.module import DreamlabsControlSpec, DreamlabsModule
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.blueprints.dreamlabs import dreamlabs_dual_openyam
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS
from dimos.spec.utils import spec_annotation_compliance


def test_complete_arm_and_gripper_trajectory_uses_one_coordinator() -> None:
    coordinator, policy = dreamlabs_dual_openyam.blueprints
    assert coordinator.module is DualOpenYamCoordinator
    assert policy.module is DreamlabsModule
    [task] = coordinator.kwargs["tasks"]
    assert task.joint_names == DUAL_OPENYAM_JOINTS
    assert task.name == "joint_trajectory"
    assert spec_annotation_compliance(DualOpenYamCoordinator, DreamlabsControlSpec)
