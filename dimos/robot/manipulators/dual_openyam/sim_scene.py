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

"""The Dual OpenYAM workcell as a MuJoCo model, composed from the robot's own URDF.

The arms come from the URDF the planner and the gravity compensation use, so
the simulated rig cannot drift from the planning model. That URDF has no
actuators and no table, so this module adds what a simulation needs: actuators
that repeat the motors' MIT loop, jaws that slide and grip through pads, the
table, the objects on it and the rig's cameras. It takes the URDF as it comes:
with fixed fingers or sliding ones, with or without collision geometry.

``scene_spec`` builds the workcell for any Damiao robot whose arms are
described by :class:`ArmLayout`; ``dual_openyam_workcell`` is the Dual OpenYAM's.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise
import math
from typing import Any
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.robot.assets.model import LoadedRobotModel, RobotModel
from dimos.robot.manipulators.damiao.robot import DamiaoBus, DamiaoMotor, DamiaoRobot

# The table's height and the overhead camera's pose are the grasp stack's: the
# planner publishes that camera pose on tf, and the cloud has to agree with it.
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    DUAL_OPENYAM_CAMERA_TRANSFORM,
    DUAL_OPENYAM_TABLE_TOP_Z,
)
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM, DUAL_OPENYAM_SIDES

#: A position or a direction, (x, y, z).
Vector = tuple[float, float, float]

#: The model's cameras, named after the ones they stand in for.
OVERHEAD_CAMERA = "overhead_camera"
WRIST_CAMERAS = {side: f"{side}_wrist_camera" for side in DUAL_OPENYAM_SIDES}
#: Vertical field of view of a D405, degrees.
CAMERA_FOVY = 58.0

#: The free bodies standing on the table.
WORKCELL_OBJECTS = ("cup", "bottle", "rubiks_cube")

#: How far each jaw slides from closed to fully open, metres: the rig's pads
#: stand 98 mm apart at full open.
FINGER_TRAVEL = 0.049

# The jaw drive: stiffness (N/m) and damping (N s/m) on the jaws' mean travel,
# and the most each pad squeezes with (N). Chosen for a firm close in about
# half a second; none of the three is measured on the rig.
_GRIP_STIFFNESS = 2000.0
_GRIP_DAMPING = 300.0
_GRIP_FORCE = 15.0
# What grips, as (centre, half size) boxes in the gripper link's frame with the
# jaws closed, read off the left tip's mesh: the flat nose, then the shank
# behind it. The right tip mirrors y.
_PAD_BOXES: tuple[tuple[Vector, Vector], ...] = (
    ((0.0, -0.003, -0.1324), (0.013, 0.003, 0.0144)),
    ((0.0, -0.008, -0.0965), (0.022, 0.006, 0.0215)),
)
_PAD_FRICTION = (1.0, 0.05, 0.0001)
# A stiff contact, so a gripped object neither sinks into the pads nor creeps.
_PAD_SOLREF = (0.004, 1.0)
_PAD_SOLIMP = (0.95, 0.99, 0.001)

# The MIT torque range of each stock Damiao motor type, Nm. A motor on a
# custom firmware profile has its own.
_MOTOR_TORQUE = {"DM4340": 28.0, "DM4310": 10.0}

# Collision bit of the world. An arm collides with the world and with the other
# arms, not with itself: its links collide as convex hulls, and the hulls of
# neighbouring links overlap.
_WORLD = 1
# MuJoCo draws geom groups 0 to 2 by default, so the cameras never see group 3.
_COLLISION_ONLY_GROUP = 3

# Static boxes as (lower corner, upper corner), metres in the workcell frame,
# read off the overhead camera's view of the rig: the white table, the black
# plate the arm bases are bolted to, and the wall behind the table.
_TABLE = ((-0.30, -0.65, DUAL_OPENYAM_TABLE_TOP_Z - 0.04), (0.64, 0.65, DUAL_OPENYAM_TABLE_TOP_Z))
_MOUNT = ((-0.15, -0.50, DUAL_OPENYAM_TABLE_TOP_Z), (0.13, 0.50, 0.0))
_WALL = ((0.64, -0.90, -0.75), (0.66, 0.90, 0.90))
_FLOOR_Z = -0.75

# Where each object stands, (x, y) in the workcell frame: the cup and the bottle
# from a scan of the rig, the cube from its place in the overhead view.
_CUP_XY = (0.33, 0.28)
_BOTTLE_XY = (0.41, 0.05)
_CUBE_XY = (0.37, -0.09)
_CUBE_YAW = math.radians(30.0)
# The cup's rim and foot radii and its height, metres: the rig's scan saw it
# 9.5 cm wide and 10.5 cm tall.
_CUP_RIM_RADIUS, _CUP_FOOT_RADIUS, _CUP_HEIGHT = 0.047, 0.030, 0.108
# How many sides a turned mesh has.
_LATHE_SEGMENTS = 48
_CUBE_HALF = 0.0285
# Each face's nine stickers, row by row, keyed by the face's outward normal.
_CUBE_FACES = {
    (0, 0, 1): "RBOYRGYOW",
    (0, 0, -1): "WGBOOYRWG",
    (1, 0, 0): "GYRGGRBWO",
    (-1, 0, 0): "BOWYBBGRY",
    (0, 1, 0): "OWGRYBWGR",
    (0, -1, 0): "YRBWWOOBG",
}
_STICKER_RGBA = {
    "W": (0.95, 0.95, 0.95, 1.0),
    "Y": (0.95, 0.85, 0.10, 1.0),
    "R": (0.80, 0.08, 0.08, 1.0),
    "O": (0.95, 0.45, 0.05, 1.0),
    "B": (0.05, 0.25, 0.75, 1.0),
    "G": (0.05, 0.60, 0.20, 1.0),
}

# The rig's arms are black; the URDF colours every part differently.
_ARM_RGBA = (0.10, 0.10, 0.11, 1.0)

# A wrist D405 on top of its gripper body: how far its lens sits above the
# gripper link's origin and out along the fingers (metres), how far it tips
# towards the fingertips, and half its housing's depth, width and height.
# Placed by eye on the Dual OpenYAM for a URDF without the camera; one that has
# it says where it is.
_WRIST_CAMERA_ABOVE = 0.060
_WRIST_CAMERA_OUT = 0.055
_WRIST_CAMERA_TILT = math.radians(20.0)
_D405_HALF_SIZE = (0.0115, 0.021, 0.021)

# MuJoCo's camera frame (x right, y up, looking along -z) in a camera_link's.
_MUJOCO_CAMERA_IN_LINK = Rotation.from_matrix([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])


@dataclass(frozen=True)
class ArmLayout:
    """One arm of a robot's URDF: the links, directions and sizes the workcell needs.

    Directions are unit vectors and positions metres, in the frame named.

    Attributes:
        prefix: Every link of this arm starts with it and no link of another
            arm does; "" for a URDF of one arm.
        gripper: The link the wrist camera rides on.
        fingers_along: Which way the fingers point, in the gripper link's frame.
        top_along: Which way the gripper's top faces, where the wrist camera
            sits, in the gripper link's frame; square to ``fingers_along``.
        fingertips: The two finger links.
        finger_axes: The axis each finger slides along, in the frame of the
            link it hangs from. A finger the URDF fixes slides along it here.
        finger_travel: Where the first finger stands along its axis with the
            jaw (closed, fully open). The second stands where the URDF's mimic
            tag puts it, or at the same place along its own axis. A wrong
            order opens the jaw when it is told to close.
        pads: What grips, as (centre, half size) boxes in the frame of the
            link the fingers hang from, with the jaw closed, for the first
            finger; the second finger's mirror y.
        wrist_camera: The name of the camera on the gripper.
        camera_above: How far the wrist camera's lens sits from the gripper
            link's origin along ``top_along``, metres. A URDF that has the
            camera's box puts the lens there instead.
        camera_out: How far the lens sits along ``fingers_along``, metres;
            negative is behind the origin.
        armature: The motor's rotor inertia felt at each arm joint, kg m^2,
            in bus order; empty for none. Without it, light wrist links on
            stiff motors can shake.
    """

    prefix: str
    gripper: str
    fingers_along: Vector
    top_along: Vector
    fingertips: tuple[str, str]
    finger_axes: tuple[Vector, Vector]
    finger_travel: tuple[float, float]
    pads: tuple[tuple[Vector, Vector], ...]
    wrist_camera: str
    camera_above: float = _WRIST_CAMERA_ABOVE
    camera_out: float = _WRIST_CAMERA_OUT
    armature: tuple[float, ...] = ()


@dataclass(frozen=True)
class SimWorkcell:
    """A robot's arms in the rig's workcell.

    Attributes:
        name: What messages call the robot, e.g. "Dual OpenYAM".
        urdf: The URDF the arms are built from, with every link and mesh; its
            root is the workcell's origin.
        arms: One layout per bus of the robot, in bus order.
        root_position: Where the URDF's root stands on the Dual OpenYAM's
            rig, metres from the middle of the top of the plate its arms are
            bolted to, x towards the table and z up. The root stays the
            workcell's origin: the table, its objects and the overhead camera
            are placed around it. Too low a root sinks the arms into the plate.
    """

    name: str
    urdf: RobotModel
    arms: tuple[ArmLayout, ...]
    root_position: Vector = (0.0, 0.0, 0.0)


def _dual_openyam_arm(side: str) -> ArmLayout:
    # The fingers slide along the gripper link's y axis, as joint7 and joint8
    # of i2rt's yam.urdf do: zero is closed and negative opens. A dual URDF
    # that fixes its fingers has them freed; one that slides them keeps its
    # own joints.
    return ArmLayout(
        prefix=f"{side}_",
        gripper=f"{side}_gripper",
        fingers_along=(0.0, 0.0, -1.0),
        top_along=(-1.0, 0.0, 0.0),
        fingertips=(f"{side}_tip_left", f"{side}_tip_right"),
        finger_axes=((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        finger_travel=(0.0, -FINGER_TRAVEL),
        pads=_PAD_BOXES,
        wrist_camera=WRIST_CAMERAS[side],
    )


#: Both arms of the Dual OpenYAM, built from its mass-bearing URDF.
DUAL_OPENYAM_WORKCELL = SimWorkcell(
    name="Dual OpenYAM",
    urdf=DUAL_OPENYAM.gravity_model,
    arms=tuple(_dual_openyam_arm(side) for side in DUAL_OPENYAM_SIDES),
)


def dual_openyam_workcell(robot: DamiaoRobot) -> mujoco.MjSpec:
    """The rig as an editable MuJoCo model: both arms, the table, objects and cameras.

    A jaw's actuator is commanded in metres of mean finger travel, 0 closed
    to ``-FINGER_TRAVEL`` open.

    Args:
        robot: The Dual OpenYAM, for its motors and MIT gains.
    """
    return scene_spec(robot, DUAL_OPENYAM_WORKCELL)


def scene_spec(robot: DamiaoRobot, workcell: SimWorkcell) -> mujoco.MjSpec:
    """The workcell as an editable MuJoCo model: the robot's arms, the table, objects and cameras.

    Actuators are in the robot's joint order, each arm joint under its own
    name and each jaw under its gripper joint's, so a simulator that lists
    actuators lists ``robot.joint_names``. A jaw's actuator is commanded in
    the fingers' own unit, metres along their axes.

    Args:
        robot: The robot, for its motors and MIT gains, one bus per arm.
        workcell: Where the robot's arms are in its URDF.
    """
    urdf = workcell.urdf.load()
    spec = _robot_spec(urdf)
    arms = list(zip(workcell.arms, robot.buses, strict=True))
    for index, (arm, bus) in enumerate(arms):
        _dress_arm(spec, arm, _arm_bits(index, len(arms)), robot.gravity)
        _add_arm_actuators(spec, arm, bus)
    for index, (arm, bus) in enumerate(arms):
        _add_jaw(spec, urdf, arm, bus, _arm_bits(index, len(arms)))
        _add_wrist_camera(spec, arm)
    root = np.asarray(workcell.root_position)
    _add_workcell(spec, root)
    _add_objects(spec, root)
    return spec


def _arm_bits(index: int, arms: int) -> tuple[int, int]:
    """The MuJoCo contype and conaffinity of the arm at ``index`` of ``arms``.

    Each arm has a bit of its own and touches the world and every other arm.
    """
    own = _WORLD << (index + 1)
    every_arm = sum(_WORLD << (other + 1) for other in range(arms))
    return own, _WORLD | (every_arm - own)


def _robot_spec(model: LoadedRobotModel) -> mujoco.MjSpec:
    """A URDF as MuJoCo reads it, every link and mesh kept."""
    urdf = ET.fromstring(model.xml)
    # A URDF's own settings are for loading it from its folder; these replace them.
    for own in urdf.findall("mujoco"):
        urdf.remove(own)
    settings = ET.SubElement(urdf, "mujoco")
    # MuJoCo's URDF importer otherwise drops the visual meshes, merges fixed
    # links such as the fingertips into their parents, and forgets mesh folders.
    ET.SubElement(
        settings, "compiler", discardvisual="false", fusestatic="false", strippath="false"
    )
    # The implicit integrator keeps the stiff joint drives stable. Elliptic
    # friction cones with a raised impedance ratio keep a held object from
    # sliding out of the jaws.
    ET.SubElement(settings, "option", integrator="implicitfast", cone="elliptic", impratio="10")
    return mujoco.MjSpec.from_string(ET.tostring(urdf, encoding="unicode"))


def _dress_arm(spec: mujoco.MjSpec, arm: ArmLayout, bits: tuple[int, int], gravity: bool) -> None:
    """Make one arm's links collide, and weigh nothing if the robot compensates gravity.

    ``bits`` are the arm's MuJoCo contype and conaffinity. Without gravity
    compensation the arm sags under its own weight, as the robot does.
    """
    contype, conaffinity = bits
    fingertips = set(arm.fingertips)
    for body in spec.bodies:
        if body.name == "world" or not body.name.startswith(arm.prefix):
            continue
        body.gravcomp = float(gravity)
        for geom in body.geoms:
            geom.rgba = _ARM_RGBA
        for geom in _colliders(body):
            # The fingertips collide through their pads instead: each tip's
            # hull reaches across the gap the jaws close on.
            touches = body.name not in fingertips
            geom.contype, geom.conaffinity = (contype, conaffinity) if touches else (0, 0)


def _colliders(body: Any) -> list[Any]:
    """The geoms a link of the URDF collides through.

    Its collision geometry, or in a URDF without any its visual mesh. A
    collision mesh is not drawn, because the link draws its visual one. A
    collision box, such as a camera bracket's, is all the URDF has of that
    part and stays drawn.
    """
    visuals = [geom for geom in body.geoms if not geom.contype]
    colliders = [geom for geom in body.geoms if geom.contype]
    for geom in colliders:
        if visuals and geom.meshname:
            geom.group = _COLLISION_ONLY_GROUP
    return colliders or visuals


def _motor_torque(motor: DamiaoMotor) -> float:
    """The most torque a motor's MIT command reaches, Nm."""
    return motor.profile.t_max if motor.profile is not None else _MOTOR_TORQUE[motor.type]


def _add_arm_actuators(spec: mujoco.MjSpec, arm: ArmLayout, bus: DamiaoBus) -> None:
    """Drive each arm joint as its motor is: stiffness, damping, gravity torque and rotor."""
    if arm.armature and len(arm.armature) != len(bus.arm):
        raise ValueError(f"bus {bus.name!r} needs one armature per arm motor")
    for index, (motor, kp, kd) in enumerate(zip(bus.arm, bus.kp, bus.kd, strict=True)):
        torque = _motor_torque(motor)
        joint = spec.joint(motor.name)
        if arm.armature:
            joint.armature = arm.armature[index]
        # Gravity compensation counts as motor torque, as on the rig, so it
        # shows in the reported effort and shares the motor's torque range.
        joint.actgravcomp = True
        joint.actfrcrange = (-torque, torque)
        _add_position_actuator(
            spec,
            motor.name,
            mujoco.mjtTrn.mjTRN_JOINT,
            motor.name,
            stiffness=kp,
            damping=kd,
            ctrlrange=(float(joint.range[0]), float(joint.range[1])),
            force=torque,
        )


def _add_position_actuator(
    spec: mujoco.MjSpec,
    name: str,
    transmission: int,
    target: str,
    *,
    stiffness: float,
    damping: float,
    ctrlrange: tuple[float, float],
    force: float,
) -> None:
    """force = stiffness * (ctrl - position) - damping * velocity, within +-force."""
    actuator = spec.add_actuator(name=name, target=target, trntype=transmission)
    actuator.set_to_position(kp=stiffness, kv=damping)
    actuator.ctrlrange = ctrlrange
    actuator.forcerange = (-force, force)


def _add_jaw(
    spec: mujoco.MjSpec,
    urdf: LoadedRobotModel,
    arm: ArmLayout,
    bus: DamiaoBus,
    bits: tuple[int, int],
) -> None:
    """Let one gripper's two fingers slide and drive them together through its gripper joint.

    ``urdf`` is the URDF the spec was read from, for how its second finger
    follows the first. ``bits`` are the arm's MuJoCo contype and conaffinity.
    """
    if bus.gripper is None:
        raise ValueError(f"bus {bus.name!r} has no gripper to simulate")
    contype, conaffinity = bits
    closed = arm.finger_travel[0]
    fingers: list[str] = []
    for name, axis, mirror_y in zip(arm.fingertips, arm.finger_axes, (1.0, -1.0), strict=True):
        tip = spec.body(name)
        # The tip's frame in the frame it hangs from, turned as the URDF turns it.
        w, x, y, z = tip.quat
        turned = Rotation.from_quat([x, y, z, w])
        # A URDF with sliding fingers brings this joint; a fixed finger is freed.
        slides = tip.joints or [
            tip.add_joint(
                name=f"{tip.name}_slide",
                type=mujoco.mjtJoint.mjJNT_SLIDE,
                axis=turned.inv().apply(axis),
                range=tuple(sorted(arm.finger_travel)),
            )
        ]
        if fingers:
            _count_travel_as_the_first(slides[0], fingers[0], urdf)
        fingers.append(slides[0].name)
        # The joint moves the tip along its axis from where the URDF puts it.
        closed_at = np.asarray(tip.pos) + closed * turned.apply(slides[0].axis)
        mirror = np.array([1.0, mirror_y, 1.0])
        for index, (centre, half) in enumerate(arm.pads):
            pad = tip.add_geom(
                name=f"{tip.name}_pad{index}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=half,
                pos=turned.inv().apply(np.asarray(centre) * mirror - closed_at),
                quat=_wxyz(turned.inv()),
                contype=contype,
                conaffinity=conaffinity,
                group=_COLLISION_ONLY_GROUP,
                condim=4,
                priority=1,
                friction=_PAD_FRICTION,
                solref=_PAD_SOLREF,
            )
            pad.solimp[:3] = _PAD_SOLIMP
    # One motor drives both fingers: the equality holds their travels equal, and
    # the tendon lets a single actuator command and read their mean.
    gripper = bus.gripper.joint
    tendon = spec.add_tendon(name=gripper)
    for finger in fingers:
        tendon.wrap_joint(finger, 0.5)
    spec.add_equality(
        name=f"{gripper}_fingers",
        type=mujoco.mjtEq.mjEQ_JOINT,  # type: ignore[attr-defined]
        objtype=mujoco.mjtObj.mjOBJ_JOINT,
        name1=fingers[0],
        name2=fingers[1],
    )
    travel = spec.joint(fingers[0]).range
    _add_position_actuator(
        spec,
        gripper,
        mujoco.mjtTrn.mjTRN_TENDON,
        gripper,
        stiffness=_GRIP_STIFFNESS,
        damping=_GRIP_DAMPING,
        ctrlrange=(float(travel[0]), float(travel[1])),
        # Each finger takes half the tendon's force.
        force=2.0 * _GRIP_FORCE,
    )


def _count_travel_as_the_first(slide: Any, first: str, urdf: LoadedRobotModel) -> None:
    """Make the second finger's joint read the same travel as the first finger's.

    A URDF finger that copies the first with a multiplier of -1 slides along
    the same axis the other way; its axis and range are turned round here, so
    the two stand at the same travel, each along its own axis.

    Raises:
        ValueError: If the URDF has the finger copy the first in any other way.
    """
    joint = urdf.get_joint(slide.name)
    if joint is None or joint.mimic is None:
        return
    leader, multiplier, offset = joint.mimic
    if leader != first or abs(multiplier) != 1.0 or offset != 0.0:
        raise ValueError(f"{slide.name!r} must copy {first!r} exactly or mirrored")
    if multiplier < 0:
        slide.axis = -np.asarray(slide.axis)
        lower, upper = slide.range
        slide.range = (-upper, -lower)


def _wxyz(rotation: Rotation) -> list[float]:
    """MuJoCo's wxyz quaternion for a rotation."""
    x, y, z, w = rotation.as_quat()
    return [w, x, y, z]


def _camera_quat(link: Rotation) -> list[float]:
    """MuJoCo's wxyz orientation for a camera whose camera_link is turned by ``link``."""
    x, y, z, w = (link * _MUJOCO_CAMERA_IN_LINK).as_quat()
    return [w, x, y, z]


def _housing(spec: mujoco.MjSpec, camera: str) -> Any:
    """The box a URDF stands that camera in, or None if it has none.

    The planner keeps clear of the box; the camera sits inside it, so it
    collides and is not drawn.
    """
    box = spec.geom(camera)
    if box is not None:
        box.group = _COLLISION_ONLY_GROUP
    return box


def _add_wrist_camera(spec: mujoco.MjSpec, arm: ArmLayout) -> None:
    """A D405 on top of the gripper, looking along the fingers."""
    along, top = np.asarray(arm.fingers_along), np.asarray(arm.top_along)
    forward = math.cos(_WRIST_CAMERA_TILT) * along - math.sin(_WRIST_CAMERA_TILT) * top
    lens = arm.camera_above * top + arm.camera_out * along
    box = _housing(spec, arm.wrist_camera)
    if box is not None:
        # The camera's back is flat on its bracket, the box's z face: it looks
        # along the box's -z, from the housing in the middle of the box.
        w, x, y, z = box.quat
        forward = -Rotation.from_quat([x, y, z, w]).as_matrix()[:, 2]
        lens = np.asarray(box.pos) + forward * _D405_HALF_SIZE[0]
    left = np.cross(top, along)
    link = Rotation.from_matrix(np.column_stack([forward, left, np.cross(forward, left)]))
    gripper = spec.body(arm.gripper)
    gripper.add_camera(name=arm.wrist_camera, pos=lens, quat=_camera_quat(link), fovy=CAMERA_FOVY)
    x, y, z, w = link.as_quat()
    # Drawn behind the lens so the overhead camera sees it. It collides with
    # nothing: a planning model with wrist cameras brings the box that does.
    gripper.add_geom(
        name=f"{arm.wrist_camera}_housing",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=_D405_HALF_SIZE,
        pos=lens - forward * _D405_HALF_SIZE[0],
        quat=[w, x, y, z],
        rgba=_ARM_RGBA,
        contype=0,
        conaffinity=0,
    )


def _add_box(
    body: Any,
    name: str,
    corners: tuple[Sequence[float], Sequence[float]],
    rgba: Sequence[float],
    root: NDArray[np.float64],
) -> None:
    """A box between two corners on the rig, with the URDF's root at ``root``."""
    lower, upper = np.asarray(corners[0]), np.asarray(corners[1])
    body.add_geom(
        name=name,
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(upper - lower) / 2,
        pos=(upper + lower) / 2 - root,
        rgba=rgba,
    )


def _add_workcell(spec: mujoco.MjSpec, root: NDArray[np.float64]) -> None:
    """The table, the arms' mounting plate, the wall, a floor, light and the overhead camera.

    Placed as on the rig, with the URDF's root at ``root``.
    """
    world = spec.worldbody
    _add_box(world, "table", _TABLE, (0.93, 0.93, 0.95, 1.0), root)
    _add_box(world, "mount", _MOUNT, (0.05, 0.05, 0.06, 1.0), root)
    _add_box(world, "wall", _WALL, (0.80, 0.80, 0.78, 1.0), root)
    # Catches whatever is knocked off the table.
    world.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=(2.0, 2.0, 0.1),
        pos=np.array([0.0, 0.0, _FLOOR_Z]) - root,
        rgba=(0.35, 0.35, 0.37, 1.0),
    )
    world.add_light(
        pos=np.array([0.3, 0.0, 1.5]) - root, dir=(0.1, 0.1, -1.0), diffuse=(0.7, 0.7, 0.7)
    )
    spec.visual.headlight.ambient = (0.35, 0.35, 0.35)
    spec.visual.headlight.diffuse = (0.3, 0.3, 0.3)
    # The calibrated camera_link (x forward, y left, z up): it looks at the
    # table from between the arms, pitched 57 degrees down.
    mount = DUAL_OPENYAM_CAMERA_TRANSFORM
    world.add_camera(
        name=OVERHEAD_CAMERA,
        pos=mount.translation.to_numpy() - root,
        quat=_camera_quat(Rotation.from_quat(mount.rotation.to_numpy())),
        fovy=CAMERA_FOVY,
    )
    _housing(spec, OVERHEAD_CAMERA)


def _add_object(
    spec: mujoco.MjSpec,
    name: str,
    xy: tuple[float, float],
    height: float,
    root: NDArray[np.float64],
    yaw: float = 0.0,
) -> Any:
    """A free body standing on the table, its frame ``height`` above the table top.

    ``xy`` is where it stands on the rig, with the URDF's root at ``root``.
    """
    body = spec.worldbody.add_body(
        name=name,
        pos=np.array([*xy, DUAL_OPENYAM_TABLE_TOP_Z + height]) - root,
        quat=(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)),
    )
    body.add_freejoint(name=name)
    return body


def _add_decoration(body: Any, **geom: Any) -> None:
    """A geom that is only drawn: it touches nothing and weighs nothing."""
    body.add_geom(contype=0, conaffinity=0, density=0.0, **geom)


def _lathe(spec: mujoco.MjSpec, name: str, profile: Sequence[tuple[float, float]]) -> str:
    """A mesh turned about z through (radius, height) points; a zero radius closes it.

    Faces look outwards where the profile climbs and inwards where it
    descends, so one profile draws a hollow vessel's outside and inside.
    """
    angles = np.linspace(0.0, 2.0 * math.pi, _LATHE_SEGMENTS, endpoint=False)
    vertices: list[tuple[float, float, float]] = []
    rings: list[list[int]] = []
    for radius, height in profile:
        first = len(vertices)
        if radius == 0.0:
            vertices.append((0.0, 0.0, height))
            rings.append([first] * _LATHE_SEGMENTS)
        else:
            vertices.extend(
                (radius * math.cos(angle), radius * math.sin(angle), height) for angle in angles
            )
            rings.append(list(range(first, first + _LATHE_SEGMENTS)))
    faces: list[tuple[int, int, int]] = []
    for lower, upper in pairwise(rings):
        for here in range(_LATHE_SEGMENTS):
            there = (here + 1) % _LATHE_SEGMENTS
            if lower[here] != lower[there]:
                faces.append((lower[here], lower[there], upper[there]))
            if upper[here] != upper[there]:
                faces.append((lower[here], upper[there], upper[here]))
    mesh = spec.add_mesh(name=name)
    mesh.uservert = np.asarray(vertices, dtype=np.float32).ravel()
    mesh.userface = np.asarray(faces, dtype=np.int32).ravel()
    # Its mass is in its surface: a cup is a shell, and a surface that is only
    # drawn may be open and have no volume.
    mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL  # type: ignore[attr-defined]
    return name


def _material(spec: mujoco.MjSpec, name: str, rgba: Sequence[float], shine: float) -> str:
    """A surface colour with a highlight, 0 matt to 1 glossy."""
    material = spec.add_material(name=name)
    material.rgba = rgba
    material.specular = material.shininess = shine
    return name


def _add_objects(spec: mujoco.MjSpec, root: NDArray[np.float64]) -> None:
    """The cup, the bottle and the Rubik's cube, at the rig's sizes and places.

    Placed as on the rig, with the URDF's root at ``root``.
    """
    cylinder = mujoco.mjtGeom.mjGEOM_CYLINDER
    mesh = mujoco.mjtGeom.mjGEOM_MESH
    box = mujoco.mjtGeom.mjGEOM_BOX

    # A red party cup, wider at its rim than at its foot. It is drawn hollow,
    # so the overhead camera looks into it, and collides as its solid hull.
    rim, foot, height = _CUP_RIM_RADIUS, _CUP_FOOT_RADIUS, _CUP_HEIGHT
    cup = _add_object(spec, "cup", _CUP_XY, 0.0, root)
    cup.add_geom(
        name="cup",
        type=mesh,
        meshname=_lathe(spec, "cup_hull", [(0.0, 0.0), (foot, 0.0), (rim, height), (0.0, height)]),
        mass=0.03,
        group=_COLLISION_ONLY_GROUP,
    )
    _add_decoration(
        cup,
        name="cup_outside",
        type=mesh,
        meshname=_lathe(spec, "cup_outside", [(0.0, 0.0), (foot, 0.0), (rim, height)]),
        material=_material(spec, "cup_red", (0.78, 0.05, 0.08, 1.0), 0.6),
    )
    wall = 0.0015
    _add_decoration(
        cup,
        name="cup_inside",
        type=mesh,
        meshname=_lathe(
            spec, "cup_inside", [(rim, height), (rim - wall, height), (foot, 0.004), (0.0, 0.004)]
        ),
        material=_material(spec, "cup_white", (0.96, 0.93, 0.93, 1.0), 0.3),
    )

    # A dark metal bottle, 6 cm wide and 17 cm tall with its cap. It is drawn
    # with its shoulder and collides as two cylinders, body and neck.
    bottle = _add_object(spec, "bottle", _BOTTLE_XY, 0.0, root)
    bottle.add_geom(
        name="bottle",
        type=cylinder,
        size=(0.03, 0.065, 0.0),
        pos=(0.0, 0.0, 0.065),
        mass=0.28,
        group=_COLLISION_ONLY_GROUP,
    )
    bottle.add_geom(
        name="bottle_neck",
        type=cylinder,
        size=(0.016, 0.02, 0.0),
        pos=(0.0, 0.0, 0.15),
        mass=0.02,
        group=_COLLISION_ONLY_GROUP,
    )
    shoulder = [(0.03, 0.112), (0.027, 0.124), (0.016, 0.142), (0.0145, 0.15)]
    _add_decoration(
        bottle,
        name="bottle_body",
        type=mesh,
        meshname=_lathe(
            spec, "bottle_body", [(0.0, 0.0), (0.027, 0.0), (0.03, 0.004), *shoulder, (0.0, 0.15)]
        ),
        material=_material(spec, "bottle_metal", (0.05, 0.06, 0.08, 1.0), 0.9),
    )
    _add_decoration(
        bottle,
        name="bottle_cap",
        type=mesh,
        meshname=_lathe(
            spec, "bottle_cap", [(0.0, 0.15), (0.016, 0.15), (0.016, 0.168), (0.0, 0.17)]
        ),
        material=_material(spec, "bottle_cap", (0.12, 0.12, 0.13, 1.0), 0.7),
    )
    _add_decoration(
        bottle,
        name="bottle_label",
        type=cylinder,
        size=(0.0303, 0.028, 0.0),
        pos=(0.0, 0.0, 0.06),
        material=_material(spec, "bottle_label", (0.10, 0.30, 0.42, 1.0), 0.3),
    )

    # A 5.7 cm Rubik's cube: a black core with nine stickers drawn on each face.
    cube = _add_object(spec, "rubiks_cube", _CUBE_XY, _CUBE_HALF, root, _CUBE_YAW)
    cube.add_geom(
        name="rubiks_cube",
        type=box,
        size=(_CUBE_HALF,) * 3,
        mass=0.09,
        rgba=(0.03, 0.03, 0.03, 1.0),
    )
    cell = 2 * _CUBE_HALF / 3
    for face, stickers in _CUBE_FACES.items():
        normal = np.array(face, dtype=float)
        across = np.roll(normal, 1)
        up = np.cross(normal, across)
        for index, colour in enumerate(stickers):
            column, row = index % 3 - 1, index // 3 - 1
            _add_decoration(
                cube,
                type=box,
                size=np.abs(normal) * 0.0003 + (np.abs(across) + np.abs(up)) * (cell / 2 - 0.0012),
                pos=normal * (_CUBE_HALF + 0.0002) + (across * column + up * row) * cell,
                rgba=_STICKER_RGBA[colour],
            )
