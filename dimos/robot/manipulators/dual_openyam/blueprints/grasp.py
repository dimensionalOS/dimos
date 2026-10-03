# Copyright 2025-2026 Dimensional Inc.
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
"""The dual OpenYAM grasping stack, on hardware by default.

```bash
dimos run dual-openyam-grasp --left-can-port follower_l --right-can-port follower_r \
    --realsensecamera.serial-number <SERIAL>
dimos run dual-openyam-grasp                                 # in-memory arms, no CAN
```

The port of the xArm grasp stack: coordinator, planner, pick-and-place, scene
registration and a heuristic grasp provider, with a fixed depth camera over
the table instead of a wrist camera. Both arms are planning groups with their
own grippers, so ``pick_object`` takes ``left_manipulator`` or
``right_manipulator``.
"""

from __future__ import annotations

from dataclasses import replace
import math

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    DualOpenYamCoordinator,
    dual_openyam_gripper_task,
    dual_openyam_trajectory_task,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_SIDES,
    dual_openyam_model_config,
)

# {side}_grasp_frame sits 5.9 cm behind the fingertip midpoint; plan to the tips.
DUAL_OPENYAM_TCP_OFFSET = (0.0535023, 0.0, -0.0295359)

# Pink's defaults do not converge on this model; the WebXR teleop uses these.
DUAL_OPENYAM_GRASP_PINK = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=1.0,
)

# Fixed camera on the centre post, measured from the point midway between the
# arm bases (x forward, y toward the left arm, z up). Placeholder until the
# rig is calibrated against its table markers.
DUAL_OPENYAM_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=0.35, y=0.0, z=0.70),
    rotation=Quaternion(0.0, 0.5, 0.0, 0.8660254),  # xyzw, pitched 60 deg down
    frame_id="world",
    child_frame_id="camera_link",
)

DUAL_OPENYAM_GRASP_PROMPTS = [
    "soup can",
    "mustard bottle",
    "cracker box",
    "banana",
    "plate",
    "toothpaste",
    "toy block",
    "mug",
    "marker",
    "towel",
]


def dual_openyam_grasp_model_config() -> RobotModelConfig:
    """Planning model in the world frame with a fingertip TCP per arm."""
    config = dual_openyam_model_config(base_pose=PoseStamped(frame_id="world"))
    model = config.model
    for side in DUAL_OPENYAM_SIDES:
        model = model.with_fixed_frame(
            f"{side}_tcp", f"{side}_grasp_frame", xyz=DUAL_OPENYAM_TCP_OFFSET
        )
    config.model = model
    config.planning_groups = [
        replace(group, tip_link=f"{group.name.split('_')[0]}_tcp")
        for group in config.planning_groups
    ]
    return config


dual_openyam_grasp = autoconnect(
    planner(
        model=dual_openyam_grasp_model_config(),
        kinematics=DUAL_OPENYAM_GRASP_PINK,
        default_speed_scale=0.25,
        static_transforms=[DUAL_OPENYAM_CAMERA_TRANSFORM],
        visualization={"backend": "viser"},
        world_frame="world",
    ),
    ManipulationSkills.blueprint(),
    PickAndPlaceModule.blueprint(planning_frame="world", pregrasp_along_tool_z=True),
    # Same reason for the half turn about Y; the extra yaws matter because the
    # two arms accept different wrist bands over the same object.
    HeuristicGraspModule.blueprint(tool_rotation_rpy=(0.0, math.pi, 0.0), yaw_candidates=8),
    RealSenseCamera.blueprint(width=640, height=480, fps=30, enable_pointcloud=True),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        detector_backend="moondream",
        segmentation_backend="edgetam",
        detect_on_request=True,
        distance_threshold=0.08,
        min_detections_for_permanent=3,
        max_distance=1.5,
        use_aabb=True,
        max_obstacle_width=0.06,
    ),
    DualOpenYamCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=[
            dual_openyam_trajectory_task(),
            dual_openyam_gripper_task("left"),
            dual_openyam_gripper_task("right"),
        ],
    ),
)
