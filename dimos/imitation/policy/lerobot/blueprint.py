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

"""OpenYAM ACT policy rollout with Quest override and wrist observations."""

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import pSHMTransport
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule
from dimos.imitation.policy.rollout_controller import QuestRolloutControllerModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    OpenYamTeleopCoordinator,
    _openyam_quest_model,
    _openyam_quest_pink,
    openyam_quest_tasks,
)
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS
from dimos.teleop.quest.quest_extensions import ArmTeleopModule

learning_rollout_quest_openyam = (
    autoconnect(
        LeRobotPolicyModule.blueprint(
            joint_names=OPENYAM_JOINTS,
            fps=30.0,
            robot_type="openyam",
        ),
        QuestRolloutControllerModule.blueprint(),
        ArmTeleopModule.blueprint(),
        OpenYamTeleopCoordinator.blueprint(
            instance_name="ControlCoordinator",
            tasks=openyam_quest_tasks(include_policy=True),
        ),
        CameraModule.blueprint(
            instance_name="WristCamera",
            webcam=WebcamConfig(
                camera_index=0,
                width=640,
                height=480,
                fps=30.0,
                frame_id_prefix="wrist",
            ),
            frame_id="wrist_camera_link",
        ),
        ManipulationModule.blueprint(
            model=_openyam_quest_model,
            kinematics=_openyam_quest_pink,
            visualization={"backend": "viser"},
        ),
    )
    .remappings(
        [
            (ArmTeleopModule, "right_controller_output", "right_cartesian_command"),
            (LeRobotPolicyModule, "joint_command", "policy_joint_command"),
        ]
    )
    .transports(
        {
            ("color_image", Image): pSHMTransport.spec(
                "/color_image",
                default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE,
            )
        }
    )
)
