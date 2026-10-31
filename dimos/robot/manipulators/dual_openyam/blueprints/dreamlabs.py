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

"""Dreamlabs joint and EEF backends on the shared policy rollout runtime."""

from typing import Literal

from dimos.control.tasks.eef_trajectory_task.eef_trajectory_task import eef_trajectory_task
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.imitation.policy.dreamlabs.camera import DreamlabsCameraModule
from dimos.imitation.policy.dreamlabs.config import DreamlabsBackendConfig
from dimos.imitation.policy.dreamlabs.requirements import check_planning_dependencies
from dimos.imitation.policy.module import policy_module
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.config import dual_openyam_model_config
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_ARM_JOINTS,
    DUAL_OPENYAM_GRIPPER_JOINTS,
    DUAL_OPENYAM_JOINTS,
)


def build_dual_openyam_dreamlabs(*, mode: Literal["joint", "eef"] = "joint") -> Blueprint:
    """Build an idle, preview-first rollout; camera devices remain deployment config."""
    execution = (
        eef_trajectory_task(
            DUAL_OPENYAM_ARM_JOINTS,
            robot_model=dual_openyam_model_config(),
            target_frames=("left_grasp_frame", "right_grasp_frame"),
            gripper_joint_names=DUAL_OPENYAM_GRIPPER_JOINTS,
            max_joint_velocity_rad_s=2.0,
        )
        if mode == "eef"
        else joint_trajectory_task(DUAL_OPENYAM_JOINTS)
    )
    return (
        autoconnect(
            DreamlabsCameraModule.blueprint(instance_name="OverheadCamera", transform=None),
            DreamlabsCameraModule.blueprint(instance_name="LeftWristCamera", transform=None),
            DreamlabsCameraModule.blueprint(instance_name="RightWristCamera", transform=None),
            DualOpenYamCoordinator.blueprint(
                instance_name="ControlCoordinator",
                tasks=(
                    [joint_trajectory_task(DUAL_OPENYAM_JOINTS, priority=20), execution]
                    if mode == "eef"
                    else [execution]
                ),
            ),
            policy_module(
                backend="dreamlabs",
                instance_name="policy",
                policy_path="dreamlabs/dream-1-beta",
                task="fold the towel",
                robot_type="dual_openyam",
                joint_names=list(DUAL_OPENYAM_JOINTS),
                dreamlabs=DreamlabsBackendConfig(mode=mode),
                image_mapping={
                    "overhead_image": "overhead",
                    "left_wrist_image": "wrist_l",
                    "right_wrist_image": "wrist_r",
                },
            ),
            ManipulationModule.blueprint(
                model=dual_openyam_model_config(),
                visualization=ViserVisualizationConfig(),
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


dreamlabs_dual_openyam_joint = autoconnect(build_dual_openyam_dreamlabs())
dreamlabs_dual_openyam_eef = autoconnect(build_dual_openyam_dreamlabs(mode="eef"))
