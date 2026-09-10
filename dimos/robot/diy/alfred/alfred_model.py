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

The URDFs live in the LFS archive ``data/.lfs/alfred_description.tar.gz`` (built from the
Onshape CAD by ``alfred_description/build_alfred_urdf.py``). ``base_link`` is the FlowBase
odometry origin (centre of the four caster kingpins, on the floor, +X forward, +Y left).

Joint conventions (canonical names are the ControlCoordinator hardware names):

* ``pillar/lift`` (URDF ``lift_joint``): prismatic +Z. The pillar Nano firmware zeroes on the
  TOP limit switch with positive = up, so every reachable position is negative,
  ``[-0.500, -0.002]`` m. The URDF uses the same convention: q = 0 is the top stop (carriage
  top = underside of the pillar-fixed front-camera mount), q = -0.5 the bottom stop.
* ``openarm_{side}_joint{1..7}``: identical in the URDF and on the OpenArm hardware.
* ``casters/<corner>_steer`` / ``casters/<corner>_drive`` (``alfred_v2`` only): state-only
  display joints driven by :class:`~dimos.robot.diy.alfred.caster_kinematics.CasterKinematics`.
"""

from __future__ import annotations

from pathlib import Path

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.diy.alfred.caster_kinematics import caster_coordinator_joints, caster_urdf_joints
from dimos.robot.diy.alfred.pillar_connection import (
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

ALFRED_V1_MODEL = RobotModel.from_file(
    ALFRED_V1_URDF, package_paths=ALFRED_PACKAGE_PATHS
).with_renamed_joints({ALFRED_LIFT_URDF_JOINT: PILLAR_LIFT_JOINT})
ALFRED_V2_MODEL = RobotModel.from_file(
    ALFRED_V2_URDF, package_paths=ALFRED_PACKAGE_PATHS
).with_renamed_joints({ALFRED_LIFT_URDF_JOINT: PILLAR_LIFT_JOINT, **_CASTER_RENAMES})

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

# With the lift at its bottom stop and the arms hanging straight down, the right gripper
# overlaps the lidar/ZED module box on the base (CAD: ~3 cm); the planner refuses that zone.
# Collision-free for lift >= this value in the all-zero arm pose.
ALFRED_LIFT_SAFE_MIN_M = -0.35


def alfred_arm_joints() -> list[str]:
    return [*openarm_urdf_joints("left"), *openarm_urdf_joints("right")]


def alfred_joint_names(wheels: bool = False) -> list[str]:
    """Canonical (coordinator) joint names: lift, both arms, then the casters with ``wheels``."""
    joints = [PILLAR_LIFT_JOINT, *alfred_arm_joints()]
    return [*joints, *caster_coordinator_joints()] if wheels else joints


def alfred_planning_groups() -> list[PlanningGroupDefinition]:
    """``lift`` (metres) and one group per arm (radians), kept separate on purpose: a mixed
    group would weight a 0.1 m lift move like a 0.1 rad wrist move in path length and timing."""
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
    """One planning robot (lift + both arms, + 8 state-only caster joints with ``wheels``) so
    collision exclusions can span them.

    ``tf_extra_links`` defaults to none: the ManipulationModule publishes them under a fixed
    ``world`` frame, which would plant a second tf root next to a navigation ``map`` tree.
    """
    joint_names = alfred_joint_names(wheels)
    return RobotModelConfig(
        model=ALFRED_V2_MODEL if wheels else ALFRED_V1_MODEL,
        joint_names=joint_names,
        base_link="base_link",
        planning_groups=alfred_planning_groups(),
        collision_exclusion_pairs=ALFRED_COLLISION_EXCLUSIONS,
        auto_convert_meshes=True,
        max_velocity=0.5,
        max_acceleration=1.0,
        tf_extra_links=list(tf_extra_links or []),
        home_joints=[0.0] * len(joint_names),
    )


def alfred_rerun_urdf(wheels: bool = False) -> Path:
    """A materialized copy of the model for Rerun's URDF loaders (yourdfpy, rerun.urdf).

    Those load a file straight from disk and cannot resolve ``package://`` URIs, so this
    writes ``RobotModel.load().xml`` (absolute mesh paths, coordinator joint names) next to
    the source URDF and returns its path. Rewritten on every call: cheap, and always in
    step with the archive.
    """
    loaded = (ALFRED_V2_MODEL if wheels else ALFRED_V1_MODEL).load()
    out = Path(loaded.source_path).with_name(".rerun") / Path(loaded.source_path).name
    out.parent.mkdir(exist_ok=True)
    out.write_text(loaded.xml)
    return out


def alfred_sim_model_config(wheels: bool = False) -> RobotModelConfig:
    """The sim/viser flavour: also publishes the sensor links on tf (no nav tree to clash with)."""
    return alfred_model_config(
        wheels=wheels,
        tf_extra_links=[
            "mid360_link",
            "camera_back_color_optical_frame",
            "camera_front_color_optical_frame",
        ],
    )
