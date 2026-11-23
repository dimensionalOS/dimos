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

"""Franka Research 3 (FR3) robot model and planning config."""

from __future__ import annotations

from pathlib import Path

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.gripper import GripperConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import joint_names

FRANKA_DESCRIPTION_REPO = "https://github.com/frankarobotics/franka_description"
FRANKA_DESCRIPTION_REF = "7aeeddc449edf8d62b594f9e36a81da53e7796f9"
_FRANKA_REPO = RobotDescriptionSource(url=FRANKA_DESCRIPTION_REPO, ref=FRANKA_DESCRIPTION_REF)
FR3_MODEL_PATH = _FRANKA_REPO / "robots" / "fr3" / "fr3.urdf.xacro"
FRANKA_PACKAGE_PATHS: dict[str, Path] = {"franka_description": _FRANKA_REPO / "."}

#: Every link and joint in the description is named behind this, e.g. "fr3_joint1".
FR3_PREFIX = "fr3_"
#: The description's root link, which fr3_link0 is fixed to.
FR3_ROOT_LINK = "base"
FR3_DOF = 7
#: The Franka Hand's driven finger, 0.0 (closed) to 0.04 m (open); finger_joint2 mimics it.
FR3_FINGER_JOINT = f"{FR3_PREFIX}finger_joint1"
FR3_FINGER_OPEN = 0.04

#: Each arm joint's top speed (rad/s), from franka_description's fr3/joint_limits.yaml.
FR3_VELOCITY_MAX = (2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26)
#: The share of each joint's top speed the arm is planned and driven at. Franka's
#: own async_position_control example uses the same 25%.
FR3_DEFAULT_SPEED_FRACTION = 0.25

#: Franka's "ready" pose: elbow up, flange pointing down, rad.
FR3_READY = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]

# The Hand's links that may touch a grasped object, and the link pairs the
# description has in contact by construction.
FR3_HAND_PAD_LINKS = (f"{FR3_PREFIX}leftfinger", f"{FR3_PREFIX}rightfinger")
FR3_HAND_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    (f"{FR3_PREFIX}hand", f"{FR3_PREFIX}leftfinger"),
    (f"{FR3_PREFIX}hand", f"{FR3_PREFIX}rightfinger"),
    (f"{FR3_PREFIX}leftfinger", f"{FR3_PREFIX}rightfinger"),
    (f"{FR3_PREFIX}link7", f"{FR3_PREFIX}hand"),
]


# Where the Franka Hand's parts sit along its approach, from fr3_hand_tcp (0.1034 m
# out of the hand), metres, from franka_description: the pads' centre (finger
# frame z 0.04525 at hand z 0.0584) is level with the TCP, the fingertips reach
# 0.0095 m past it, and the hand's body ends 0.0374 m behind it (hand.stl reaches
# z 0.066). Heuristic grasps aim the pads with these.
FR3_HEURISTIC_AIM = {
    "pad_behind_tcp": 0.0,
    "fingertip_reach": 0.0095,
    "palm_behind_tcp": 0.0374,
}


def fr3_wrist_camera_transform() -> Transform:
    """Where the wrist RealSense D435i sits on the Franka Hand: fr3_hand -> camera_link.

    Hand-eye calibrated on Dimensional's first FR3 (2026-11-22, after its camera
    cable was re-taped and the camera shifted 5 mm and 1 deg): 15 views of a
    ChArUco board (8x5, 30 mm squares), closed-form solve then bundle adjustment
    over all 420 corners; 0.59 px reprojection RMS, about 0.5 mm and 0.05 deg
    1-sigma. The same as the "right" arm's in ``tabletop_bimanual``.
    camera_link is RealSense's body frame on the depth imager (x out of the
    lens, y to the camera's left, z up), as dimOS's RealSense module places it.
    The camera looks down the hand's +z with its long side along the hand's y.
    Recalibrate if the camera or its mount is moved.
    """
    return Transform(
        translation=Vector3(x=0.0531, y=-0.0161, z=0.0656),
        rotation=Quaternion(0.715223, -0.016805, 0.698639, 0.008832),  # xyzw
        frame_id=f"{FR3_PREFIX}hand",
        child_frame_id="camera_link",
    )


def fr3_joint_names(robot: str | None = None) -> list[str]:
    """The arm joints, e.g. "fr3_joint1", or "arm/fr3_joint1" under ``robot``."""
    names = joint_names(FR3_DOF, prefix=f"{FR3_PREFIX}joint")
    return [f"{robot}/{name}" for name in names] if robot else names


def fr3_model(
    *,
    hand: bool = True,
    robot: str | None = None,
    speed_fraction: float = FR3_DEFAULT_SPEED_FRACTION,
) -> RobotModel:
    """The FR3's URDF, from Franka's franka_description.

    Args:
        hand: Whether the Franka Hand is on the flange.
        robot: Name the moving joints under this robot, as the coordinator
            wants them, e.g. "arm" gives "arm/fr3_joint1". Links keep their names.
        speed_fraction: Scale every arm joint's velocity limit by this, in
            (0, 1]. The connection drives the arm at the same share, so plans
            and the arm keep pace.
    """
    if not 0.0 < speed_fraction <= 1.0:
        raise ValueError(f"speed_fraction must be in (0, 1], got {speed_fraction}")
    model = RobotModel.from_file(
        FR3_MODEL_PATH,
        package_paths=FRANKA_PACKAGE_PATHS,
        xacro_args={"hand": "true" if hand else "false", "ee_id": "franka_hand"},
    ).with_joint_velocity_limits(
        {
            name: limit * speed_fraction
            for name, limit in zip(fr3_joint_names(), FR3_VELOCITY_MAX, strict=True)
        }
    )
    if not robot:
        return model
    moving = fr3_joint_names()
    if hand:
        moving.append(FR3_FINGER_JOINT)
    return model.with_renamed_joints({joint: f"{robot}/{joint}" for joint in moving})


def fr3_gripper_config(robot: str | None = None) -> GripperConfig:
    """Which Franka Hand links may touch a grasped object, and how it closes."""
    finger = f"{robot}/{FR3_FINGER_JOINT}" if robot else FR3_FINGER_JOINT
    return GripperConfig(
        pad_links=FR3_HAND_PAD_LINKS,
        finger_joints={finger: (FR3_FINGER_OPEN, 0.0)},
    )


def make_fr3_model_config(
    *,
    robot: str | None = None,
    hand: bool = True,
    speed_fraction: float = FR3_DEFAULT_SPEED_FRACTION,
    base_pose: PoseStamped | None = None,
    home_joints: list[float] | None = None,
    pre_grasp_offset: float = 0.10,
    tf_extra_links: list[str] | None = None,
) -> RobotModelConfig:
    """The FR3's planning model, from ``fr3_model``; tool frame is the Hand's TCP.

    ``tf_extra_links`` are links published on tf besides the tip, e.g.
    ``fr3_hand``, which the wrist camera's transform hangs off.
    """
    names = fr3_joint_names(robot)
    tip_link = f"{FR3_PREFIX}hand_tcp" if hand else f"{FR3_PREFIX}link8"
    return RobotModelConfig(
        model=fr3_model(
            hand=hand, robot=robot, speed_fraction=speed_fraction
        ).with_default_joint_acceleration_limit(2.0),
        base_pose=base_pose if base_pose is not None else PoseStamped(),
        joint_names=names,
        # The description's root: franka_description fixes fr3_link0 to a "base" link.
        base_link=FR3_ROOT_LINK,
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(names),
                base_link=FR3_ROOT_LINK,
                tip_link=tip_link,
            )
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=FR3_HAND_COLLISION_EXCLUSIONS if hand else [],
        gripper=fr3_gripper_config(robot) if hand else None,
        home_joints=home_joints or list(FR3_READY),
        pre_grasp_offset=pre_grasp_offset,
        tf_extra_links=tf_extra_links or [],
    )
