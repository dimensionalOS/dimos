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

"""Two FR3s clamped to a tabletop: the shared planning model and its calibration.

This is not Franka's bimanual product or any fixed dual-arm mount. It is two
ordinary, independent FR3s, each with its own control box and FCI address, whose
bases are clamped to the same table wherever they were put. Where one base sits
relative to the other is measured, not designed: the numbers below come from
calibrating Dimensional's setup (2026-11-22) and must be re-measured whenever
either base is unclamped or moved.

Seen from behind the arms, looking the way they face, the "right" arm is arm 1
(the first FR3, FCI 10.42.0.230) and the "left" arm is arm 2 (10.43.0.53). The
world frame is the right arm's base frame. The bases are about 78 cm apart, each
turned inward, so the arms share the table between them.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from pydantic import BaseModel
from scipy.spatial.transform import Rotation

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.franka.config import (
    FR3_DEFAULT_SPEED_FRACTION,
    FR3_FINGER_JOINT,
    FR3_HAND_COLLISION_EXCLUSIONS,
    FR3_PREFIX,
    FR3_READY,
    FR3_ROOT_LINK,
    fr3_joint_names,
    fr3_model,
)

#: The two arms, as the coordinator and planner name them: joints "right/fr3_joint1" ...
SIDES = ("right", "left")
#: The shared root link, at the right arm's base.
TABLETOP_ROOT_LINK = "tabletop"

#: Where the left arm's base sits in the right arm's base frame: metres, and roll,
#: pitch, yaw (extrinsic xyz, rad). Both bases sit flat on one level table, so the
#: offset is planar: x, y and a turn about the vertical. Fitted to every corner of two
#: ChArUco boards taped to the table as both wrist cameras saw them (bimanual_cal.py):
#: 4.3 mm RMS, 5.7 mm worst. The left arm, sent 5 cm above a board corner that only
#: the right arm had measured, landed centred on it and within about 1 mm of height.
#: Each arm sees the flat table tilted by 2 to 3 deg in its own frame, so heights far
#: from a base carry about a centimetre of error; perception fits the tabletop itself.
#: Values for any other table come from a JSON file (``TabletopCalibration``) named
#: by the environment variable ``DIMOS_FR3_TABLETOP_CALIBRATION``.


class _Pose(BaseModel):
    xyz: tuple[float, float, float]
    xyzw: tuple[float, float, float, float]


class _Placement(BaseModel):
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


class TabletopCalibration(BaseModel):
    """One tabletop pair's measured geometry: the left base in the right base's frame,
    each wrist camera on its Hand (``<side>/fr3_hand -> <side>_wrist_camera_link``),
    and the tripod camera in the world (``world -> scene_camera_link``)."""

    left_base: _Placement
    wrist_cameras: dict[str, _Pose]
    scene_camera: _Pose


#: Dimensional's setup, measured 2026-11-22 (see the notes on each value below).
DIMENSIONAL_CALIBRATION = TabletopCalibration(
    left_base=_Placement(xyz=(0.1597, 0.7579, 0.0), rpy=(0.0, 0.0, -0.39176)),
    wrist_cameras={
        "right": _Pose(
            xyz=(0.0531, -0.0161, 0.0656), xyzw=(0.715223, -0.016805, 0.698639, 0.008832)
        ),
        "left": _Pose(
            xyz=(0.0532, -0.0201, 0.0623), xyzw=(0.724977, -0.009629, 0.688661, 0.007858)
        ),
    },
    scene_camera=_Pose(
        xyz=(0.9972, 0.1926, 1.3128), xyzw=(-0.456538, 0.029794, 0.883769, 0.098172)
    ),
)

CALIBRATION_ENV = "DIMOS_FR3_TABLETOP_CALIBRATION"


def load_calibration(path: str | os.PathLike[str] | None = None) -> TabletopCalibration:
    """The calibration from ``path``, else from the file ``$DIMOS_FR3_TABLETOP_CALIBRATION``
    names, else Dimensional's."""
    source = path if path is not None else os.environ.get(CALIBRATION_ENV)
    if not source:
        return DIMENSIONAL_CALIBRATION
    return TabletopCalibration.model_validate(json.loads(Path(source).read_text()))


CALIBRATION = load_calibration()
LEFT_BASE_XYZ = CALIBRATION.left_base.xyz
LEFT_BASE_RPY = CALIBRATION.left_base.rpy

#: The ready pose for both arms, right then left.
TABLETOP_BIMANUAL_READY = [*FR3_READY, *FR3_READY]

#: Each wrist camera and its mount as one box, in the camera's camera_link frame
#: (x out of the lens, y to its left, z up): the D435i body (90 x 25 x 25 mm,
#: centred 17.5 mm to the right of camera_link) and the bracket back to the hand
#: (toward -z), padded 5 mm all round. The planner and the guard keep the other arm
#: off it; a knock moves the camera and spoils its calibration.
WRIST_CAMERA_BOX_CENTRE = (-0.0095, -0.0175, -0.011)
WRIST_CAMERA_BOX_SIZE = (0.04, 0.105, 0.057)
_WRIST_CAMERA_URDF = """<?xml version="1.0"?>
<robot name="wrist_camera">
  <link name="wrist_camera">
    <visual><origin xyz="{c}"/><geometry><box size="{s}"/></geometry></visual>
    <collision><origin xyz="{c}"/><geometry><box size="{s}"/></geometry></collision>
  </link>
</robot>
""".format(
    c=" ".join(str(v) for v in WRIST_CAMERA_BOX_CENTRE),
    s=" ".join(str(v) for v in WRIST_CAMERA_BOX_SIZE),
)


def arm_joint_names(side: str) -> list[str]:
    """One arm's seven joints, e.g. "left/fr3_joint1"."""
    _check(side)
    return fr3_joint_names(side)


def finger_joint(side: str) -> str:
    """One arm's driven Franka Hand finger, e.g. "left/fr3_finger_joint1"."""
    _check(side)
    return f"{side}/{FR3_FINGER_JOINT}"


def link(side: str, name: str) -> str:
    """One arm's link, e.g. link("left", "hand") is "left/fr3_hand"."""
    _check(side)
    return f"{side}/{FR3_PREFIX}{name}"


def wrist_camera_link(side: str) -> str:
    """The box standing in for one arm's wrist camera and mount, e.g. "left/wrist_camera"."""
    _check(side)
    return f"{side}/wrist_camera"


def _check(side: str) -> None:
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}, got {side!r}")


def tabletop_bimanual_model(*, speed_fraction: float = FR3_DEFAULT_SPEED_FRACTION) -> RobotModel:
    """Both FR3s with Franka Hands and wrist cameras, under one root at the right arm's base."""
    arm = fr3_model(speed_fraction=speed_fraction)
    camera = RobotModel.from_urdf_string(_WRIST_CAMERA_URDF)
    model = (
        RobotModel.from_root_link(TABLETOP_ROOT_LINK, name="fr3_tabletop_bimanual")
        .with_attached_model(
            arm, parent_link=TABLETOP_ROOT_LINK, joint_name="right_mount", prefix="right/"
        )
        .with_attached_model(
            arm,
            parent_link=TABLETOP_ROOT_LINK,
            joint_name="left_mount",
            prefix="left/",
            xyz=LEFT_BASE_XYZ,
            rpy=LEFT_BASE_RPY,
        )
    )
    for side in SIDES:
        mount = wrist_camera_transform(side)
        q = mount.rotation
        rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")
        model = model.with_attached_model(
            camera,
            parent_link=link(side, "hand"),
            joint_name=f"{side}_wrist_camera_mount",
            prefix=f"{side}/",
            xyz=(mount.translation.x, mount.translation.y, mount.translation.z),
            rpy=(float(rpy[0]), float(rpy[1]), float(rpy[2])),
        )
    return model


def make_tabletop_bimanual_model_config(
    *,
    speed_fraction: float = FR3_DEFAULT_SPEED_FRACTION,
    tf_extra_links: list[str] | None = None,
) -> RobotModelConfig:
    """One collision world with both arms, and a planning group per arm.

    Every plan, for either arm or both, is checked against the other arm in the
    same world. The groups are "right" and "left", each from its own base to its
    Hand's TCP, plus "both" for moving the two together.
    """
    groups = [
        PlanningGroupDefinition(
            name=side,
            joint_names=tuple(arm_joint_names(side)),
            base_link=f"{side}/{FR3_ROOT_LINK}",
            tip_link=link(side, "hand_tcp"),
        )
        for side in SIDES
    ]
    both = tuple(name for side in SIDES for name in arm_joint_names(side))
    return RobotModelConfig(
        model=tabletop_bimanual_model(
            speed_fraction=speed_fraction
        ).with_default_joint_acceleration_limit(2.0),
        joint_names=list(both),
        base_link=TABLETOP_ROOT_LINK,
        planning_groups=[
            *groups,
            PlanningGroupDefinition(name="both", joint_names=both, base_link=TABLETOP_ROOT_LINK),
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=[
            *(
                (f"{side}/{a}", f"{side}/{b}")
                for side in SIDES
                for a, b in FR3_HAND_COLLISION_EXCLUSIONS
            ),
            # Each camera's bracket is bolted to its own hand, beside the fingers.
            *(
                (wrist_camera_link(side), link(side, part))
                for side in SIDES
                for part in ("link7", "hand", "leftfinger", "rightfinger")
            ),
        ],
        home_joints=list(TABLETOP_BIMANUAL_READY),
        tf_extra_links=tf_extra_links or [],
    )


def wrist_camera_transform(side: str) -> Transform:
    """Where each arm's wrist RealSense D435i sits on its Franka Hand: <side>/fr3_hand ->
    <side>_wrist_camera_link.

    Hand-eye calibrated on Dimensional's setup (2026-11-22), 15 views of a ChArUco
    board each, closed-form solve then bundle adjustment: right 0.59 px, left
    0.57 px reprojection RMS, about 0.5 mm and 0.05 deg 1-sigma. The two mounts
    are the same part and agree to about 4 mm and 1 deg. Recalibrate a camera if it
    or its mount is moved: the right arm's shifted 5 mm and 1 deg when its cable
    was re-taped.
    """
    _check(side)
    pose = CALIBRATION.wrist_cameras[side]
    xyz, xyzw = pose.xyz, pose.xyzw
    return Transform(
        translation=Vector3(*xyz),
        rotation=Quaternion(*xyzw),
        frame_id=link(side, "hand"),
        child_frame_id=f"{side}_wrist_camera_link",
    )


def scene_camera_transform(world_frame: str = "world") -> Transform:
    """Where the tripod RealSense looking over the table sits: world -> scene_camera_link.

    ``world_frame`` is the frame the right arm's base, and so the tabletop root, is
    placed at.

    From a 4x3 ChArUco marker taped to the table, which both wrist cameras located
    and the tripod camera sees (2026-11-22): about 1.3 m above the table, in front
    of the arms, looking down 54 deg. The tripod is not fixed; re-measure after it
    moves.
    """
    pose = CALIBRATION.scene_camera
    return Transform(
        translation=Vector3(*pose.xyz),
        rotation=Quaternion(*pose.xyzw),
        frame_id=world_frame,
        child_frame_id="scene_camera_link",
    )
