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

"""Alfred whole-robot planning model: FlowBase + pillar lift + bimanual OpenArm v2.0 + sensors.

Frames (see FDEIssues/alfred-urdf/DECISION_cad_to_urdf_path.md):
  base_link   FlowBase odometry origin: centre of the four caster kingpins, on the floor,
              X forward / Y left / Z up (i2rt flow_base_controller body frame).
  lift_joint  prismatic +Z, 0..0.5 m, zero = mechanical bottom stop. Coordinator name ``pillar/lift``.
  openarm_{left,right}_base_link  vendor v2.0 flange frames at (0.140, ±0.092, 0.947 + lift).
  <corner>_caster_joint / <corner>_wheel_joint  (alfred_v2 only) powered-caster steer (+Z at kingpin) / drive (+Y axle).
  mid360_link Livox O frame, lidar on a 22 deg forward-leaning wedge.  camera_{front,back}_* RealSense frames.
"""

from __future__ import annotations

from pathlib import Path

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.diy.alfred.caster_kinematics import caster_coordinator_joints, caster_urdf_joints
from dimos.robot.diy.alfred.pillar_connection import PILLAR_LIFT_JOINT
from dimos.robot.manipulators._modeling import base_pose
from dimos.robot.manipulators.openarm.config import (
    OPENARM_DESCRIPTION_ROOT,
    OPENARM_GRIPPER_COLLISION_EXCLUSIONS,
    OPENARM_SIDES,
    openarm_arm_joints,
    openarm_urdf_joints,
)
from dimos.utils.data import LfsPath

# LFS archive data/.lfs/alfred_description.tar.gz (built by FDEIssues/alfred-urdf/build_alfred_urdf.py --v1 --arms v20).
ALFRED_DESCRIPTION_ROOT = LfsPath("alfred_description")
ALFRED_PACKAGE_PATHS: dict[str, Path] = {
    "alfred_description": ALFRED_DESCRIPTION_ROOT,
    "openarm_description": OPENARM_DESCRIPTION_ROOT,
}
ALFRED_URDF = (
    ALFRED_DESCRIPTION_ROOT / "urdf" / "alfred_v1.urdf"
)  # forks/wheels welded to base_link
ALFRED_V2_URDF = (
    ALFRED_DESCRIPTION_ROOT / "urdf" / "alfred_v2.urdf"
)  # + 8 caster joints (steer/drive)
ALFRED_MODEL = RobotModel.from_file(ALFRED_URDF, package_paths=ALFRED_PACKAGE_PATHS)
ALFRED_V2_MODEL = RobotModel.from_file(ALFRED_V2_URDF, package_paths=ALFRED_PACKAGE_PATHS)

ALFRED_LIFT_URDF_JOINT = "lift_joint"
ALFRED_LIFT_TRAVEL_M = 0.5

ALFRED_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    *OPENARM_GRIPPER_COLLISION_EXCLUSIONS,
    # flanges bolted to the carriage adapters, sensors bolted to their brackets
    *[
        ("lift_link", f"openarm_{side}_{link}")
        for side in OPENARM_SIDES
        for link in ("base_link", "link1")
    ],
    ("lift_link", "camera_front_link"),
    ("base_link", "lift_link"),
    ("base_link", "camera_back_link"),
    ("base_link", "mid360_link"),
]


def alfred_urdf_joints(wheels: bool = False) -> list[str]:
    joints = [ALFRED_LIFT_URDF_JOINT, *openarm_urdf_joints("left"), *openarm_urdf_joints("right")]
    return [*joints, *caster_urdf_joints()] if wheels else joints


def alfred_sim_model_config(name: str = "alfred", wheels: bool = False) -> RobotModelConfig:
    """One planning robot: lift + both arms (+ 8 caster joints with ``wheels``), so collision
    exclusions can span them. Caster joints are state-only (no planning group)."""
    joint_names = alfred_urdf_joints(wheels)
    caster_mapping = (
        dict(zip(caster_coordinator_joints(), caster_urdf_joints(), strict=True)) if wheels else {}
    )
    return RobotModelConfig(
        name=name,
        model=ALFRED_V2_MODEL if wheels else ALFRED_MODEL,
        base_pose=base_pose(),
        joint_names=joint_names,
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="lift",
                joint_names=(ALFRED_LIFT_URDF_JOINT,),
                base_link="base_link",
                tip_link="lift_link",
            ),
            *[
                PlanningGroupDefinition(
                    name=f"{side}_manipulator",
                    joint_names=tuple(openarm_urdf_joints(side)),
                    base_link="lift_link",
                    tip_link=f"openarm_{side}_grasp_frame",
                )
                for side in OPENARM_SIDES
            ],
        ],
        collision_exclusion_pairs=ALFRED_COLLISION_EXCLUSIONS,
        auto_convert_meshes=True,
        max_velocity=0.5,
        max_acceleration=1.0,
        joint_name_mapping={
            PILLAR_LIFT_JOINT: ALFRED_LIFT_URDF_JOINT,
            **caster_mapping,
            **{
                coordinator_name: urdf_name
                for side in OPENARM_SIDES
                for coordinator_name, urdf_name in zip(
                    openarm_arm_joints(side), openarm_urdf_joints(side), strict=True
                )
            },
        },
        tf_extra_links=[
            "mid360_link",
            "camera_back_color_optical_frame",
            "camera_front_color_optical_frame",
        ],
        home_joints=[0.0] * len(joint_names),
    )
