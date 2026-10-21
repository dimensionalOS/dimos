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
from dimos.imitation.policy.dreamlabs.camera import DreamlabsCameraModule
from dimos.imitation.policy.dreamlabs.module import DreamlabsModule
from dimos.imitation.policy.dreamlabs.requirements import check_planning_dependencies
from dimos.imitation.policy.dreamlabs.rollout import DreamlabsRolloutModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.config import dual_openyam_model_config
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS

# Protocol-only: no camera, driver, coordinator, or action executor is launched.
dreamlabs_dual_openyam = autoconnect(DreamlabsModule.blueprint())

# Device paths are required deployment config. CAN omitted means existing mock hardware.
dreamlabs_dual_openyam_manual = (
    autoconnect(
        DreamlabsCameraModule.blueprint(instance_name="OverheadCamera", transform=None),
        DreamlabsCameraModule.blueprint(instance_name="LeftWristCamera", transform=None),
        DreamlabsCameraModule.blueprint(instance_name="RightWristCamera", transform=None),
        DualOpenYamCoordinator.blueprint(
            instance_name="ControlCoordinator",
            tasks=[joint_trajectory_task(DUAL_OPENYAM_JOINTS)],
        ),
        DreamlabsRolloutModule.blueprint(),
        ManipulationModule.blueprint(
            model=dual_openyam_model_config(),
            # View actual state without a second interactive execution surface.
            visualization=ViserVisualizationConfig(panel_enabled=False),
        ),
    )
    .remappings(
        [
            ("OverheadCamera", "color_image", "overhead_image"),
            ("LeftWristCamera", "color_image", "left_wrist_image"),
            ("RightWristCamera", "color_image", "right_wrist_image"),
        ]
    )
    .requirements(check_planning_dependencies)
)
