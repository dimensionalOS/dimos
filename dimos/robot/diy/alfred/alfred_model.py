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
firmware; openarm_{side}_joint{1..7} are the same on the arms. alfred_v2's eight caster
joints are not coordinator joints: the urdf carries the links so they can be animated for
display some other way, but nothing here drives them.
"""

from __future__ import annotations

from pathlib import Path

from dimos.control.tasks.holonomic_pose_follower_task.holonomic_pose_follower_task import (
    DEFAULT_ARTIFACT_PATH as _GO2_FOLLOWER_ARTIFACT,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import PlanarBaseDefinition, RobotModel
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
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Alfred's measured plant model, from a characterization run against the real
# FlowBase. Config, not a build output: nothing in this branch regenerates it.
# Lives beside the robot it describes, not in the follower task's artifact dir.
ALFRED_FOLLOWER_ARTIFACT = str(Path(__file__).resolve().parent / "alfred_posedomain.json")


def alfred_follower_artifact() -> str:
    """Alfred's own plant model when it is present; the Go2's when it is not.

    The follower raises on a missing artifact, so this resolves at blueprint-build
    time rather than hardcoding a path: a checkout without the artifact still
    starts - loudly, on a quadruped's gains, which it will overshoot on.
    """
    if Path(ALFRED_FOLLOWER_ARTIFACT).exists():
        return ALFRED_FOLLOWER_ARTIFACT
    logger.warning(
        "Alfred has no tuned artifact; the follower falls back to the Go2's plant model "
        "and will run hot and overshoot. Restore it from the branch it was characterized "
        "on before driving the base at speed.",
        expected=ALFRED_FOLLOWER_ARTIFACT,
    )
    return _GO2_FOLLOWER_ARTIFACT


ALFRED_DESCRIPTION_ROOT = LfsPath("alfred_description")
ALFRED_PACKAGE_PATHS: dict[str, Path] = {
    "alfred_description": ALFRED_DESCRIPTION_ROOT,
    "openarm_description": OPENARM_DESCRIPTION_ROOT,
}
ALFRED_V1_URDF = ALFRED_DESCRIPTION_ROOT / "urdf" / "alfred_v1.urdf"  # forks/wheels welded
# alfred_v2 adds eight display-only caster joints, so ``wheels`` selects the urdf
# without changing the joint set.
ALFRED_V2_URDF = ALFRED_DESCRIPTION_ROOT / "urdf" / "alfred_v2.urdf"

ALFRED_LIFT_URDF_JOINT = "lift_joint"
ALFRED_LIFT_LOWER_M = PILLAR_MIN_POSITION_M  # -0.500, bottom stop
ALFRED_LIFT_UPPER_M = PILLAR_MAX_POSITION_M  # -0.002, just under the top switch
ALFRED_LIFT_LINK = "lift_link"

# Measured off alfred_v1.urdf's collision scene at the home pose; test_alfred_nav
# re-derives them so they cannot rot. The MLS footprint is circular and Alfred is
# holonomic, so the honest hard clearance is the circumscribed radius.
ALFRED_FOOTPRINT_RADIUS_M = 0.373  # base_link -> worst xy corner
ALFRED_INSCRIBED_RADIUS_M = 0.255  # base_link -> nearest face
# Top of the collision scene: the headroom a cell needs to be standable.
ALFRED_HEIGHT_M = 1.86

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
    .with_renamed_joints({ALFRED_LIFT_URDF_JOINT: PILLAR_LIFT_JOINT})
)

# The FlowBase as three synthetic planning coordinates. Deliberately slower than
# free navigation - the arms are out. TUNE ON HARDWARE.
ALFRED_BASE_VELOCITY_LIMITS = (0.5, 0.5, 1.0)  # vx, vy m/s; wz rad/s
ALFRED_BASE_ACCELERATION_LIMITS = (1.0, 1.0, 2.0)
ALFRED_PLANAR_BASE = PlanarBaseDefinition(
    velocity_limits=ALFRED_BASE_VELOCITY_LIMITS,
    acceleration_limits=ALFRED_BASE_ACCELERATION_LIMITS,
    root_link="alfred_planar_base_root",
    joint_names=("alfred/base_x", "alfred/base_y", "alfred/base_yaw"),
)
ALFRED_V1_PLANAR_MODEL = ALFRED_V1_MODEL.with_planar_base(ALFRED_PLANAR_BASE)
ALFRED_V2_PLANAR_MODEL = ALFRED_V2_MODEL.with_planar_base(ALFRED_PLANAR_BASE)

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


def alfred_joint_names() -> list[str]:
    """Canonical (coordinator) joint names: the lift, then both arms."""
    return [PILLAR_LIFT_JOINT, *alfred_arm_joints()]


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
    joint_names = alfred_joint_names()
    # pillar/lift is zero at the top limit switch, so an all-zero home is out of range.
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


def alfred_planar_joint_names() -> list[str]:
    """Planning joints with the base in front, matching the planar model's order."""
    return [*ALFRED_PLANAR_BASE.joint_names, *alfred_joint_names()]


def alfred_planar_model_config(
    *,
    wheels: bool = False,
    tf_extra_links: list[str] | None = None,
) -> RobotModelConfig:
    """The whole-body model: planar base, lift and both arms in one planning robot.

    Same model as ``alfred_model_config`` with three synthetic base coordinates ahead of
    it, so the planner can decide to drive as part of reaching. The base is unbounded in
    translation and periodic in yaw, so it adds no reachability limit of its own; what
    bounds it is the velocity and acceleration in ``ALFRED_PLANAR_BASE``.
    """
    joint_names = alfred_planar_joint_names()
    home_joints = [0.0] * len(joint_names)
    home_joints[joint_names.index(PILLAR_LIFT_JOINT)] = PILLAR_HOME_POSITION_M
    return RobotModelConfig(
        model=ALFRED_V2_PLANAR_MODEL if wheels else ALFRED_V1_PLANAR_MODEL,
        joint_names=joint_names,
        base_link=ALFRED_PLANAR_BASE.root_link,
        planning_groups=[
            *alfred_planning_groups(),
            PlanningGroupDefinition(
                name="moving_base",
                joint_names=ALFRED_PLANAR_BASE.joint_names,
                base_link=ALFRED_PLANAR_BASE.root_link,
            ),
        ],
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
