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

"""Alfred whole-robot planning model: FlowBase, pillar lift, bimanual OpenArm v2.0, sensors.

The URDFs come from the LFS archive alfred_description (built from the Onshape CAD by the
bundled build_alfred_urdf.py). base_link is the FlowBase odometry origin on the floor,
+X forward. Canonical joint names are the coordinator names: pillar/lift is zero at the top
limit switch with positive up, so its range is -0.500..-0.002 m, matching the pillar
firmware; openarm_{side}_joint{1..7} are the same on the arms; casters/* (alfred_v2 only)
are display joints driven by CasterKinematics.
"""

from __future__ import annotations

from pathlib import Path

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.diy.alfred.caster_kinematics import caster_coordinator_joints, caster_urdf_joints
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_HOME_POSITION_M,
    PILLAR_LIFT_JOINT,
    PILLAR_MAX_POSITION_M,
    PILLAR_MIN_POSITION_M,
)
from dimos.robot.manipulators.openarm.config import (
    OPENARM_DESCRIPTION_ROOT,
    OPENARM_GRIPPER_COLLISION_EXCLUSIONS,
    OPENARM_SIDES,
    openarm_urdf_joints,
)
from dimos.utils.data import LfsPath

ALFRED_DESCRIPTION_ROOT = LfsPath("alfred_description")
ALFRED_PACKAGE_PATHS: dict[str, Path] = {
    "alfred_description": ALFRED_DESCRIPTION_ROOT,
    "openarm_description": OPENARM_DESCRIPTION_ROOT,
}
ALFRED_V1_URDF = ALFRED_DESCRIPTION_ROOT / "urdf" / "alfred_v1.urdf"  # forks/wheels welded
ALFRED_V2_URDF = ALFRED_DESCRIPTION_ROOT / "urdf" / "alfred_v2.urdf"  # + 8 caster joints

ALFRED_LIFT_URDF_JOINT = "lift_joint"
ALFRED_LIFT_LOWER_M = PILLAR_MIN_POSITION_M  # -0.500, bottom stop
ALFRED_LIFT_UPPER_M = PILLAR_MAX_POSITION_M  # -0.002, just under the top switch
ALFRED_LIFT_LINK = "lift_link"

_CASTER_RENAMES = dict(zip(caster_urdf_joints(), caster_coordinator_joints(), strict=True))

# Joint velocity limits come from the URDF; acceleration is not in URDF, so one default.
ALFRED_JOINT_ACCELERATION_LIMIT = 1.0
ALFRED_V1_MODEL = (
    RobotModel.from_file(ALFRED_V1_URDF, package_paths=ALFRED_PACKAGE_PATHS)
    .with_default_joint_acceleration_limit(ALFRED_JOINT_ACCELERATION_LIMIT)
    .with_renamed_joints({ALFRED_LIFT_URDF_JOINT: PILLAR_LIFT_JOINT})
)
ALFRED_V2_MODEL = (
    RobotModel.from_file(ALFRED_V2_URDF, package_paths=ALFRED_PACKAGE_PATHS)
    .with_default_joint_acceleration_limit(ALFRED_JOINT_ACCELERATION_LIMIT)
    .with_renamed_joints({ALFRED_LIFT_URDF_JOINT: PILLAR_LIFT_JOINT, **_CASTER_RENAMES})
)

ALFRED_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    *OPENARM_GRIPPER_COLLISION_EXCLUSIONS,
    # flanges bolted to the carriage adapters, sensors bolted to their brackets
    *[
        (ALFRED_LIFT_LINK, f"openarm_{side}_{link}")
        for side in OPENARM_SIDES
        for link in ("base_link", "link1")
    ],
    (ALFRED_LIFT_LINK, "camera_front_link"),
    ("base_link", ALFRED_LIFT_LINK),
    ("base_link", "camera_back_link"),
    ("base_link", "mid360_link"),
]

# Below this, with the arms hanging straight down, the right gripper enters the lidar module.
ALFRED_LIFT_SAFE_MIN_M = -0.35


def alfred_arm_joints() -> list[str]:
    return [*openarm_urdf_joints("left"), *openarm_urdf_joints("right")]


def alfred_joint_names(wheels: bool = False) -> list[str]:
    """Canonical (coordinator) joint names: lift, both arms, then the casters with ``wheels``."""
    joints = [PILLAR_LIFT_JOINT, *alfred_arm_joints()]
    return [*joints, *caster_coordinator_joints()] if wheels else joints


def alfred_planning_groups() -> list[PlanningGroupDefinition]:
    """Lift (metres) and one group per arm (radians); mixing units in one group misweights paths."""
    return [
        PlanningGroupDefinition(
            name="lift",
            joint_names=(PILLAR_LIFT_JOINT,),
            base_link="base_link",
            tip_link=ALFRED_LIFT_LINK,
        ),
        *[
            PlanningGroupDefinition(
                name=f"{side}_manipulator",
                joint_names=tuple(openarm_urdf_joints(side)),
                base_link=ALFRED_LIFT_LINK,
                tip_link=f"openarm_{side}_grasp_frame",
            )
            for side in OPENARM_SIDES
        ],
    ]


def alfred_model_config(
    *,
    wheels: bool = False,
    tf_extra_links: list[str] | None = None,
) -> RobotModelConfig:
    """One planning robot so collision exclusions can span lift and arms.

    tf_extra_links defaults to none: the ManipulationModule publishes them under a fixed
    world frame, a second tf root next to a navigation tree.
    """
    joint_names = alfred_joint_names(wheels)
    home_joints = [0.0] * len(joint_names)
    home_joints[joint_names.index(PILLAR_LIFT_JOINT)] = PILLAR_HOME_POSITION_M
    return RobotModelConfig(
        model=ALFRED_V2_MODEL if wheels else ALFRED_V1_MODEL,
        joint_names=joint_names,
        base_link="base_link",
        planning_groups=alfred_planning_groups(),
        collision_exclusion_pairs=ALFRED_COLLISION_EXCLUSIONS,
        auto_convert_meshes=True,
        tf_extra_links=list(tf_extra_links or []),
        home_joints=home_joints,
    )


def alfred_rerun_urdf(wheels: bool = False) -> Path:
    """Materialize the model for Rerun's URDF loaders, which cannot resolve package:// URIs."""
    loaded = (ALFRED_V2_MODEL if wheels else ALFRED_V1_MODEL).load()
    out = Path(loaded.source_path).with_name(".rerun") / Path(loaded.source_path).name
    out.parent.mkdir(exist_ok=True)
    out.write_text(loaded.xml)
    return out


def alfred_sim_model_config(wheels: bool = False) -> RobotModelConfig:
    """The sim flavour also publishes the sensor links on tf; there is no nav tree to clash with."""
    return alfred_model_config(
        wheels=wheels,
        tf_extra_links=[
            "mid360_link",
            "camera_back_color_optical_frame",
            "camera_front_color_optical_frame",
        ],
    )
