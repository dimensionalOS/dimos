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


"""Dreamlabs rollout using the dual OpenYAM coordinator and existing image ports."""

from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.imitation.policy.dreamlabs.module import DreamlabsModule
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS

# A single bounded trajectory includes both normalized grippers. Do not add a
# second coordinator to an already running station; compose this policy instead.
dreamlabs_dual_openyam = autoconnect(
    DualOpenYamCoordinator.blueprint(
        tasks=[joint_trajectory_task(DUAL_OPENYAM_JOINTS)],
    ),
    DreamlabsModule.blueprint(),
)
