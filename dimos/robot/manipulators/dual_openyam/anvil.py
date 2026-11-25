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

"""The dual OpenYAM driven with the numbers Anvil ships for its OpenYAM v2.

The arms are Anvil's hardware, so this variant takes Anvil's mass model for
gravity compensation, its joint limits, its joint gains and its joint speeds,
all as its SDK (anvil-robotics 2.0.0b9) states them for the OpenYAM v2 with
the Anvil gripper. Kinematics, IK and planning keep the DimOS model; both
models share joint axes and zero pose.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators._modeling import named_for_robot
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM,
    DUAL_OPENYAM_SIDES,
    dual_openyam_arm_joints,
    dual_openyam_control_model,
    dual_openyam_model_config,
)

#: Position gains per arm joint, joint1 to joint6.
ANVIL_OPENYAM_KP = [40.0, 65.0, 65.0, 25.0, 12.0, 12.0]
#: Velocity gains per arm joint, joint1 to joint6.
ANVIL_OPENYAM_KD = [3.0, 4.5, 4.5, 0.7, 0.2, 0.2]
#: Position limits per arm joint, rad.
ANVIL_OPENYAM_JOINT_LIMITS = (
    (-2.61799, 3.14159),
    (0.0, 3.66519),
    (0.0, 4.01426),
    (-1.65806, 1.65806),
    (-1.5708, 1.5708),
    (-2.35619, 1.8326),
)
#: Fastest each arm joint may be commanded, rad/s (216 deg/s and 720 deg/s).
ANVIL_OPENYAM_MAX_JOINT_SPEED = (
    *[math.radians(216.0)] * 3,
    *[math.radians(720.0)] * 3,
)
#: The posture IK leans towards, rad (0, 50, 50, -40, 0, 0 degrees).
ANVIL_OPENYAM_REST_POSTURE = [math.radians(angle) for angle in (0.0, 50.0, 50.0, -40.0, 0.0, 0.0)]
#: Where one arm goes when sent home, rad (0, 31.5, 32.8, -1.3, 0, 0 degrees).
ANVIL_OPENYAM_HOME = [math.radians(angle) for angle in (0.0, 31.5, 32.8, -1.3, 0.0, 0.0)]
#: The pose an arm folds to before power is cut: every joint at zero.
ANVIL_OPENYAM_REST = [0.0] * 6
#: The pose passed through between rest and home so the forearm lifts first, rad.
ANVIL_OPENYAM_HOME_VIA = [math.radians(angle) for angle in (0.0, 12.0, 26.0, 15.0, 0.0, 0.0)]


@dataclass(frozen=True, slots=True)
class _Body:
    """One rigid body of the mass model and the joint that carries it.

    Attributes:
        name: The link name without its side.
        xyz: The joint's origin in the parent link, m.
        rpy: The joint's orientation in the parent link, rad.
        mass: The body's mass, kg.
        com: Its centre of mass in the link frame, m.
        inertia: ixx, ixy, ixz, iyy, iyz, izz about the centre of mass, kg m^2.
        joint: The arm joint that turns this body about its z axis, 1 to 6,
            or ``None`` for a body fixed to its parent.
        parent: The parent link's name without its side.
    """

    name: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    mass: float
    com: tuple[float, float, float]
    inertia: tuple[float, float, float, float, float, float]
    joint: int | None
    parent: str


# The OpenYAM v2 arm with the Anvil gripper, as Anvil's gravity compensation reads it.
_BODIES = (
    _Body(
        "base",
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        0.822546,
        (-0.0000722812, 0.0000252572, 0.0159932),
        (0.00169415, 0.000000243507, -0.000000131515, 0.00066093, -0.000000911171, 0.00155626),
        None,
        "mount",
    ),
    _Body(
        "link1",
        (0.0, 0.0, 0.0664),
        (0.0, 0.0, 1.5708),
        0.12415,
        (-0.00192861, -0.00795735, 0.0208176),
        (0.0000598038, -0.0000045902, 0.000007411, 0.000169215, 0.0000168342, 0.000156182),
        1,
        "base",
    ),
    _Body(
        "link2",
        (0.000025, -0.02, 0.0409),
        (1.5708, 0.0, 1.5708),
        1.24434,
        (0.156500918, 0.0169758057, 0.000293073),
        (0.000683304, 0.00000000199503, 0.00000000368337, 0.0121902, 0.0000000032, 0.0121318),
        2,
        "link1",
    ),
    _Body(
        "link3",
        (0.264, 0.0, 0.0),
        (-3.14159, 0.0, 0.0),
        1.22947334,
        (-0.0963507918, -0.0355265402, -0.0000771271338),
        (0.00120255, -0.000772865, -0.0000148502, 0.0100088, -0.00000919678, 0.0100264),
        3,
        "link2",
    ),
    _Body(
        "link4",
        (-0.245, -0.0600003, 0.0),
        (0.0, 0.0, 0.0),
        0.673213752,
        (-0.0172346674, -0.0265532195, 0.000106646061),
        (0.000845853, -0.000363695, -0.0000103957, 0.000713327, -0.00000468386, 0.00108188),
        4,
        "link3",
    ),
    _Body(
        "link5",
        (-0.0739968, -0.0395003, 0.0000244738),
        (-1.5708, 1.5708, 0.0),
        0.39491887,
        (0.00087215762, -0.0075561134, 0.0408136765),
        (0.000222572, -0.000000360083, -0.000000227026, 0.000194234, -0.00001249, 0.000171365),
        5,
        "link4",
    ),
    _Body(
        "link6",
        (0.0, 0.0353, 0.0395),
        (-1.5708, 0.0, 0.0),
        0.573229843134,
        (0.00125289685124, 0.0111037437594, -0.06596259975),
        (0.000649266, 0.0000100035, -0.00000369568, 0.000517148, 0.0000722837, 0.00034546),
        6,
        "link5",
    ),
    _Body(
        "hand",
        (0.0, 0.0, 0.05),
        (0.0, 0.0, 1.5708),
        0.7,
        (0.0, 0.002, 0.03),
        (0.0002473, 0.000001, 0.000001, 0.00001763, 0.000001, 0.0002521),
        None,
        "link6",
    ),
    _Body(
        "right_finger",
        (0.0, -0.006, 0.015),
        (0.0, 0.0, 0.0),
        0.03602545343277134,
        (0.0064528, -0.01702, 0.0219685),
        (0.000002375, 0.000001, 0.000001, 0.000002375, 0.000001, 0.00000075),
        None,
        "hand",
    ),
    _Body(
        "left_finger",
        (0.0, 0.006, 0.015),
        (0.0, 0.0, 0.0),
        0.03602545343277134,
        (0.0064528, 0.01702, 0.0219685),
        (0.000002375, 0.000001, 0.000001, 0.000002375, 0.000001, 0.00000075),
        None,
        "hand",
    ),
)
_ROOT_LINK = "dual_openyam_base"
# Where each arm's base sits on the shared root, m; gravity does not depend on it.
_MOUNT_Y = {"left": 0.31, "right": -0.31}


def _triple(values: tuple[float, float, float]) -> str:
    return " ".join(repr(value) for value in values)


def _body_xml(side: str, body: _Body) -> str:
    link = f"{side}_anvil_{body.name}"
    parent = _ROOT_LINK if body.parent == "mount" else f"{side}_anvil_{body.parent}"
    xyz = (0.0, _MOUNT_Y[side], 0.0) if body.parent == "mount" else body.xyz
    ixx, ixy, ixz, iyy, iyz, izz = body.inertia
    if body.joint is None:
        joint = f'<joint name="{link}_fixed" type="fixed">'
        motion = ""
    else:
        lower, upper = ANVIL_OPENYAM_JOINT_LIMITS[body.joint - 1]
        speed = ANVIL_OPENYAM_MAX_JOINT_SPEED[body.joint - 1]
        joint = f'<joint name="{side}_joint{body.joint}" type="revolute">'
        motion = (
            '<axis xyz="0 0 1"/>'
            f'<limit lower="{lower!r}" upper="{upper!r}" velocity="{speed!r}" effort="0"/>'
        )
    return (
        f'<link name="{link}"><inertial>'
        f'<origin xyz="{_triple(body.com)}" rpy="0 0 0"/><mass value="{body.mass!r}"/>'
        f'<inertia ixx="{ixx!r}" ixy="{ixy!r}" ixz="{ixz!r}" '
        f'iyy="{iyy!r}" iyz="{iyz!r}" izz="{izz!r}"/>'
        "</inertial></link>"
        f'{joint}<parent link="{parent}"/><child link="{link}"/>'
        f'<origin xyz="{_triple(xyz)}" rpy="{_triple(body.rpy)}"/>{motion}</joint>'
    )


def anvil_dual_openyam_gravity_urdf() -> str:
    """The two arms as bare masses on joints named like the DimOS model's, as URDF text."""
    bodies = "".join(_body_xml(side, body) for side in DUAL_OPENYAM_SIDES for body in _BODIES)
    return f'<robot name="dual_openyam_anvil_masses"><link name="{_ROOT_LINK}"/>{bodies}</robot>'


def _with_anvil_limits(model: RobotModel) -> RobotModel:
    """Anvil's position and speed limits on the twelve arm joints, by their URDF names."""
    speeds: dict[str, float] = {}
    for side in DUAL_OPENYAM_SIDES:
        for name, (lower, upper), speed in zip(
            dual_openyam_arm_joints(side),
            ANVIL_OPENYAM_JOINT_LIMITS,
            ANVIL_OPENYAM_MAX_JOINT_SPEED,
            strict=True,
        ):
            model = model.with_joint_position_limits(name, lower=lower, upper=upper)
            speeds[name] = speed
    return model.with_joint_velocity_limits(speeds)


def anvil_dual_openyam_model_config(*, robot: str | None = None) -> RobotModelConfig:
    """The planning model with Anvil's joint limits and speeds."""
    config = dual_openyam_model_config()
    limited = config.model_copy(update={"model": _with_anvil_limits(config.model)})
    return named_for_robot(limited, robot)


#: The dual OpenYAM with Anvil's mass model, limits and gains. Its own name keeps
#: a calibration fitted against the other mass model from being applied to it.
DUAL_OPENYAM_ANVIL = DamiaoRobot(
    buses=[
        replace(bus, kp=list(ANVIL_OPENYAM_KP), kd=list(ANVIL_OPENYAM_KD))
        for bus in DUAL_OPENYAM.buses
    ],
    model=_with_anvil_limits(dual_openyam_control_model()),
    gravity_model=RobotModel.from_urdf_string(anvil_dual_openyam_gravity_urdf()),
    base_link=DUAL_OPENYAM.base_link,
    joint_names=list(DUAL_OPENYAM.joint_names),
    home_joints=[*ANVIL_OPENYAM_HOME, *ANVIL_OPENYAM_HOME],
    name="dual_openyam_anvil",
)
