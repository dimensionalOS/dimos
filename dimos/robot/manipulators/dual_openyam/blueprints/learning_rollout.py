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

"""Released ABC-DiT rollout for the complete Dual OpenYAM entity."""

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.transport import pSHMTransport
from dimos.imitation.cameras import CameraDevice, profile_cameras
from dimos.imitation.policy.abc.module import DualOpenYamAbcPolicy
from dimos.imitation.policy.module import (
    POLICY_ROLLOUT_INSTANCE_NAME,
    POLICY_ROLLOUT_TASK_NAME,
)
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.learning import ABC_JOINTS, DUAL_OPENYAM_ABC_IO


def build_dual_openyam_abc_rollout(
    *,
    artifact: str,
    task: str,
    cameras: dict[str, CameraDevice],
    device: str | None = None,
    quest_control: bool = False,
    left_can_port: str | None = None,
    right_can_port: str | None = None,
) -> Blueprint:
    """Build three-camera ABC rollout; a synthetic top view is never accepted."""
    if quest_control:
        raise ValueError("Quest takeover is not defined for Dual OpenYAM ABC rollout")
    camera_blueprints, camera_remappings = profile_cameras(DUAL_OPENYAM_ABC_IO, cameras)
    blueprint = autoconnect(
        DualOpenYamAbcPolicy.blueprint(
            instance_name=POLICY_ROLLOUT_INSTANCE_NAME,
            artifact=artifact,
            task=task,
            device=device,
            trajectory_task_name=POLICY_ROLLOUT_TASK_NAME,
        ),
        DualOpenYamCoordinator.blueprint(
            instance_name="ControlCoordinator",
            left_can_port=left_can_port,
            right_can_port=right_can_port,
            tasks=[
                TaskConfig(
                    name=POLICY_ROLLOUT_TASK_NAME,
                    type="trajectory",
                    joint_names=list(ABC_JOINTS),
                    priority=10,
                    params={"start_position_tolerance": 0.05},
                )
            ],
        ),
        *camera_blueprints,
    ).remappings(camera_remappings)
    return blueprint.transports(
        {
            (stream, Image): pSHMTransport.spec(
                f"/{stream}",
                default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE,
            )
            for stream in ("top_image", "left_wrist_image", "right_wrist_image")
        }
    )
