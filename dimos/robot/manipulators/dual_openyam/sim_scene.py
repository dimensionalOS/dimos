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
the simulated rig cannot drift from the planning model. That URDF has fixed
fingers, no collision geometry and no environment, so this module adds what a
simulation needs: actuators that repeat the motors' MIT loop, sliding jaws
with gripping pads, the table, the objects on it and the rig's three cameras.
"""

from __future__ import annotations

from collections.abc import Sequence
import math
from typing import Any
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.manipulators.damiao.robot import DamiaoBus, DamiaoRobot
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_SIDES

#: Table top in the workcell frame, metres (rig calibration 2026-10-15).
DUAL_OPENYAM_TABLE_TOP_Z = -0.0346

#: The model's cameras, named after the ones they stand in for.
OVERHEAD_CAMERA = "overhead_camera"
WRIST_CAMERAS = {side: f"{side}_wrist_camera" for side in DUAL_OPENYAM_SIDES}
#: Vertical field of view of a D405, degrees.
CAMERA_FOVY = 58.0

#: The overhead D405's camera_link (x forward, y left, z up) in the workcell
#: frame, metres and an xyzw quaternion, from the rig calibration of 2026-10-15.
#: It looks at the table from between the arms, pitched 57 degrees down.
OVERHEAD_CAMERA_XYZ = (-0.01684, 0.01262, 0.41508)
OVERHEAD_CAMERA_XYZW = (-0.02168, 0.47733, -0.00152, 0.87845)

#: The free bodies standing on the table.
WORKCELL_OBJECTS = ("cup", "bottle", "rubiks_cube")

#: How far each jaw slides from closed to fully open, metres: the rig's pads
#: stand 98 mm apart at full open.
FINGER_TRAVEL = 0.049

# The dual URDF fixes the fingers that i2rt's yam.urdf slides along the gripper
# link's y axis (joint7 and joint8). Zero is closed and negative opens.
_FINGER_AXES = {"left": (0.0, 1.0, 0.0), "right": (0.0, -1.0, 0.0)}
# The jaw drive: stiffness (N/m) and damping (N s/m) on the jaws' mean travel,
# and the most each pad squeezes with (N). Chosen for a firm close in about
# half a second; none of the three is measured on the rig.
_GRIP_STIFFNESS = 2000.0
_GRIP_DAMPING = 300.0
_GRIP_FORCE = 15.0
# What grips, as (centre, half size) boxes in the gripper link's frame with the
# jaws closed, read off the left tip's mesh: the flat nose, then the shank
# behind it. The right tip mirrors y.
_PAD_BOXES = (
    ((0.0, -0.003, -0.1324), (0.013, 0.003, 0.0144)),
    ((0.0, -0.008, -0.0965), (0.022, 0.006, 0.0215)),
)
_PAD_FRICTION = (1.0, 0.05, 0.0001)
# A stiff contact, so a gripped object neither sinks into the pads nor creeps.
_PAD_SOLREF = (0.004, 1.0)
_PAD_SOLIMP = (0.95, 0.99, 0.001)

# The MIT torque range of each Damiao motor type, Nm.
_MOTOR_TORQUE = {"DM4340": 28.0, "DM4310": 10.0}

# Collision bits. An arm collides with the world and with the other arm, not
# with itself: its links collide as the convex hulls of their visual meshes,
# and the hulls of neighbouring links overlap.
_WORLD, _LEFT, _RIGHT = 1, 2, 4
_ARM_BITS = {"left": (_LEFT, _WORLD | _RIGHT), "right": (_RIGHT, _WORLD | _LEFT)}
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

# A wrist D405 on top of its gripper body, in the gripper link's frame (fingers
# along -z, top along -x): where its lens sits, how far it tips towards the
# fingertips, and half its housing's depth, width and height. Placed by eye;
# the rig's wrist mounts are not calibrated.
_WRIST_CAMERA_XYZ = (-0.060, 0.0, -0.055)
_WRIST_CAMERA_TILT = math.radians(20.0)
_D405_HALF_SIZE = (0.0115, 0.021, 0.021)

# MuJoCo's camera frame (x right, y up, looking along -z) in a camera_link's.
_MUJOCO_CAMERA_IN_LINK = Rotation.from_matrix([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])


def dual_openyam_workcell(robot: DamiaoRobot) -> mujoco.MjSpec:
    """The rig as an editable MuJoCo model: both arms, the table, objects and cameras.

    Actuators are in the robot's joint order, each arm joint under its own
    name and each jaw under its gripper joint's, so a simulator that lists
    actuators lists ``robot.joint_names``. A jaw's actuator is commanded in
    metres of mean finger travel, 0 closed to ``-FINGER_TRAVEL`` open.

    Args:
        robot: The Dual OpenYAM, for its URDF, motors and MIT gains.
    """
    spec = _robot_spec(robot)
    arms = list(zip(DUAL_OPENYAM_SIDES, robot.buses, strict=True))
    for side, bus in arms:
        _dress_arm(spec, side)
        _add_arm_actuators(spec, bus)
    for side, bus in arms:
        _add_jaw(spec, side, bus)
        _add_wrist_camera(spec, side)
    _add_workcell(spec)
    _add_objects(spec)
    return spec


def _robot_spec(robot: DamiaoRobot) -> mujoco.MjSpec:
    """The robot's mass-bearing URDF as MuJoCo reads it, every link and mesh kept."""
    urdf = ET.fromstring(robot.gravity_model.load().xml)
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


def _dress_arm(spec: mujoco.MjSpec, side: str) -> None:
    """Make one arm's links collide and weigh nothing, as gravity compensation leaves them."""
    contype, conaffinity = _ARM_BITS[side]
    fingertips = {f"{side}_tip_{jaw}" for jaw in _FINGER_AXES}
    for body in spec.bodies:
        if not body.name.startswith(f"{side}_"):
            continue
        body.gravcomp = 1.0
        for geom in body.geoms:
            geom.rgba = _ARM_RGBA
            # The fingertips collide through their pads instead: each tip's
            # hull reaches across the gap the jaws close on.
            if body.name not in fingertips:
                geom.contype, geom.conaffinity = contype, conaffinity


def _add_arm_actuators(spec: mujoco.MjSpec, bus: DamiaoBus) -> None:
    """Drive each arm joint as its motor is: stiffness, damping and gravity torque."""
    for motor, kp, kd in zip(bus.arm, bus.kp, bus.kd, strict=True):
        torque = _MOTOR_TORQUE[motor.type]
        joint = spec.joint(motor.name)
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


def _add_jaw(spec: mujoco.MjSpec, side: str, bus: DamiaoBus) -> None:
    """Free one gripper's two fingers and drive them together through its gripper joint."""
    if bus.gripper is None:
        raise ValueError(f"bus {bus.name!r} has no gripper to simulate")
    contype, conaffinity = _ARM_BITS[side]
    fingers = []
    for jaw, axis in _FINGER_AXES.items():
        tip = spec.body(f"{side}_tip_{jaw}")
        finger = f"{tip.name}_slide"
        tip.add_joint(
            name=finger,
            type=mujoco.mjtJoint.mjJNT_SLIDE,
            axis=axis,
            range=(-FINGER_TRAVEL, 0.0),
        )
        fingers.append(finger)
        # The tip links are translations of the gripper link's frame.
        mirror = np.array([1.0, axis[1], 1.0])
        for index, (centre, half) in enumerate(_PAD_BOXES):
            pad = tip.add_geom(
                name=f"{tip.name}_pad{index}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=half,
                pos=np.asarray(centre) * mirror - np.asarray(tip.pos),
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
    _add_position_actuator(
        spec,
        gripper,
        mujoco.mjtTrn.mjTRN_TENDON,
        gripper,
        stiffness=_GRIP_STIFFNESS,
        damping=_GRIP_DAMPING,
        ctrlrange=(-FINGER_TRAVEL, 0.0),
        # Each finger takes half the tendon's force.
        force=2.0 * _GRIP_FORCE,
    )


def _camera_quat(link: Rotation) -> list[float]:
    """MuJoCo's wxyz orientation for a camera whose camera_link is turned by ``link``."""
    x, y, z, w = (link * _MUJOCO_CAMERA_IN_LINK).as_quat()
    return [w, x, y, z]


def _add_wrist_camera(spec: mujoco.MjSpec, side: str) -> None:
    """A D405 on top of the gripper, looking along the fingers."""
    forward = np.array([math.sin(_WRIST_CAMERA_TILT), 0.0, -math.cos(_WRIST_CAMERA_TILT)])
    left = np.array([0.0, -1.0, 0.0])
    link = Rotation.from_matrix(np.column_stack([forward, left, np.cross(forward, left)]))
    lens = np.asarray(_WRIST_CAMERA_XYZ)
    gripper = spec.body(f"{side}_gripper")
    gripper.add_camera(
        name=WRIST_CAMERAS[side], pos=lens, quat=_camera_quat(link), fovy=CAMERA_FOVY
    )
    x, y, z, w = link.as_quat()
    # Drawn behind the lens so the overhead camera sees it; it collides with
    # nothing, like the planning model, which has no wrist cameras.
    gripper.add_geom(
        name=f"{WRIST_CAMERAS[side]}_housing",
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
) -> None:
    lower, upper = np.asarray(corners[0]), np.asarray(corners[1])
    body.add_geom(
        name=name,
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(upper - lower) / 2,
        pos=(upper + lower) / 2,
        rgba=rgba,
    )


def _add_workcell(spec: mujoco.MjSpec) -> None:
    """The table, the arms' mounting plate, the wall, a floor, light and the overhead camera."""
    world = spec.worldbody
    _add_box(world, "table", _TABLE, (0.93, 0.93, 0.95, 1.0))
    _add_box(world, "mount", _MOUNT, (0.05, 0.05, 0.06, 1.0))
    _add_box(world, "wall", _WALL, (0.80, 0.80, 0.78, 1.0))
    # Catches whatever is knocked off the table.
    world.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=(2.0, 2.0, 0.1),
        pos=(0.0, 0.0, _FLOOR_Z),
        rgba=(0.35, 0.35, 0.37, 1.0),
    )
    world.add_light(pos=(0.3, 0.0, 1.5), dir=(0.1, 0.1, -1.0), diffuse=(0.7, 0.7, 0.7))
    spec.visual.headlight.ambient = (0.35, 0.35, 0.35)
    spec.visual.headlight.diffuse = (0.3, 0.3, 0.3)
    world.add_camera(
        name=OVERHEAD_CAMERA,
        pos=OVERHEAD_CAMERA_XYZ,
        quat=_camera_quat(Rotation.from_quat(OVERHEAD_CAMERA_XYZW)),
        fovy=CAMERA_FOVY,
    )


def _add_object(
    spec: mujoco.MjSpec, name: str, xy: tuple[float, float], height: float, yaw: float = 0.0
) -> Any:
    """A free body standing on the table, its frame ``height`` above the table top."""
    body = spec.worldbody.add_body(
        name=name,
        pos=(*xy, DUAL_OPENYAM_TABLE_TOP_Z + height),
        quat=(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)),
    )
    body.add_freejoint(name=name)
    return body


def _add_decoration(body: Any, **geom: Any) -> None:
    """A geom that is only drawn: it touches nothing and weighs nothing."""
    body.add_geom(contype=0, conaffinity=0, density=0.0, **geom)


def _add_objects(spec: mujoco.MjSpec) -> None:
    """The cup, the bottle and the Rubik's cube, at the rig's sizes and places."""
    cylinder = mujoco.mjtGeom.mjGEOM_CYLINDER
    box = mujoco.mjtGeom.mjGEOM_BOX

    # A plastic cup, 8 cm wide and 10 cm tall, drawn open at the top.
    cup = _add_object(spec, "cup", _CUP_XY, 0.05)
    cup.add_geom(
        name="cup", type=cylinder, size=(0.04, 0.05, 0.0), mass=0.03, rgba=(0.80, 0.08, 0.10, 1.0)
    )
    _add_decoration(
        cup,
        name="cup_inside",
        type=cylinder,
        size=(0.035, 0.0005, 0.0),
        pos=(0.0, 0.0, 0.0501),
        rgba=(0.95, 0.80, 0.82, 1.0),
    )

    # A bottle, 6 cm wide and 17 cm tall with its neck.
    bottle = _add_object(spec, "bottle", _BOTTLE_XY, 0.0)
    bottle.add_geom(
        name="bottle",
        type=cylinder,
        size=(0.03, 0.065, 0.0),
        pos=(0.0, 0.0, 0.065),
        mass=0.28,
        rgba=(0.05, 0.06, 0.08, 1.0),
    )
    bottle.add_geom(
        name="bottle_neck",
        type=cylinder,
        size=(0.016, 0.02, 0.0),
        pos=(0.0, 0.0, 0.15),
        mass=0.02,
        rgba=(0.04, 0.04, 0.05, 1.0),
    )
    _add_decoration(
        bottle,
        name="bottle_label",
        type=cylinder,
        size=(0.0303, 0.03, 0.0),
        pos=(0.0, 0.0, 0.06),
        rgba=(0.10, 0.30, 0.42, 1.0),
    )

    # A 5.7 cm Rubik's cube: a black core with nine stickers drawn on each face.
    cube = _add_object(spec, "rubiks_cube", _CUBE_XY, _CUBE_HALF, _CUBE_YAW)
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
