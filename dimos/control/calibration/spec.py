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

"""What the calibration math needs to know about one arm, derived from its model.

Unless given, ``build_arm_spec`` derives: ``measured_joints`` (joints whose model gravity
torque over their range spans more than ``torque_resolution_nm``, for a Damiao motor its MIT
frame's), ``gravity_links`` (child links of the arm
joints at or below a measured joint; pinocchio merges rigidly attached links into them, see
``merged_links``), ``joint_ranges`` (the URDF limits), ``max_torque_nm`` (the URDF effort,
or for a Damiao motor its datasheet continuous torque), and a ``WorkspaceGuard``. Workspace
coordinates are metres from ``base_frame`` along the model root's axes, so z is height.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
import hashlib
import itertools
import math
from pathlib import PurePosixPath
import re
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.robot.manipulators.damiao.robot import DamiaoMotor, DamiaoRobot

Vector3 = tuple[float, float, float]

CONTINUOUS_RANGE_RAD = (-math.pi, math.pi)
# Torque range of the stock Damiao MIT frames by motor type (the driver's T_MAX).
DAMIAO_STOCK_TORQUE_LIMIT_NM: dict[str, float] = {"DM4310": 10.0, "DM4340": 28.0, "DM8009": 54.0}
# Rated continuous torque by motor type, from the Damiao datasheets: what a pose may hold.
DAMIAO_CONTINUOUS_TORQUE_NM: dict[str, float] = {"DM4310": 3.0, "DM4340": 9.0, "DM8009": 20.0}
# Width of the torque field of a Damiao MIT frame (protocol): its resolution is 2 T_MAX / 4095.
MIT_TORQUE_BITS = 12
TIP_FRAME_HINTS = ("grasp", "tcp")

_ARM_ID = re.compile(r"^[A-Za-z0-9_.-]+(/[A-Za-z0-9_.-]+)*$")
_FILENAME_ATTR = re.compile(r'filename="([^"]*)"')


class NoWorkspaceGuardError(ValueError):
    """An arm would be moved with only the implicit, model-derived workspace guard."""


@dataclass(frozen=True)
class HalfSpace:
    """A keep-out region: every point ``p`` with ``normal . p > offset_m``."""

    normal: Vector3
    offset_m: float
    name: str = "keep-out"

    def violated_by(self, points: np.ndarray) -> bool:
        values = np.atleast_2d(np.asarray(points, dtype=np.float64)) @ np.asarray(self.normal)
        return bool((values > self.offset_m).any())


@dataclass(frozen=True)
class WorkspaceGuard:
    """Where an arm may go during calibration.

    The clearances are operator settings: how far above the floor every checked point stays at
    a pose and along the ramps between poses. ``explicit`` is False for the guard a spec gets
    when nobody described the surroundings (floor at the model root, nothing else); nothing
    moves an arm with it. ``clearance_frames`` None means derived from the model.
    """

    floor_z_m: float = 0.0
    pose_clearance_m: float = 0.15
    transition_clearance_m: float = 0.11
    min_x_m: float | None = None
    keep_out: tuple[HalfSpace, ...] = ()
    clearance_frames: tuple[str, ...] | None = None
    point_offset_m: Vector3 = (0.0, 0.0, 0.0)
    explicit: bool = True

    def __post_init__(self) -> None:
        if self.pose_clearance_m <= 0.0 or self.transition_clearance_m <= 0.0:
            raise ValueError("workspace clearances must be positive")

    @classmethod
    def for_bench(cls, floor_z_m: float, min_x_m: float, **fields: object) -> WorkspaceGuard:
        """An explicit guard: the table height and the line the fingertip stays in front of."""
        return cls(floor_z_m=floor_z_m, min_x_m=min_x_m, **fields)  # type: ignore[arg-type]

    @property
    def min_pose_z_m(self) -> float:
        return self.floor_z_m + self.pose_clearance_m

    @property
    def min_transition_z_m(self) -> float:
        return self.floor_z_m + self.transition_clearance_m


@dataclass(frozen=True)
class ArmCalibrationSpec:
    """One arm: its id, mass-bearing model, joints in motor order, and what is fitted and measured."""

    arm_id: str
    model: RobotModel
    joint_names: list[str]
    base_frame: str
    tip_frame: str
    home_joints: dict[str, float]
    gravity_links: list[str]
    measured_joints: list[str]
    joint_ranges: dict[str, tuple[float, float]]
    max_torque_nm: dict[str, float]
    workspace: WorkspaceGuard = field(default_factory=lambda: WorkspaceGuard(explicit=False))
    gains: dict[str, tuple[float, float]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not _ARM_ID.match(self.arm_id) or ".." in self.arm_id.split("/"):
            raise ValueError(f"arm_id must be a relative path of plain names, got {self.arm_id!r}")
        names = self.joint_names
        if not names or len(set(names)) != len(names):
            raise ValueError(f"{self.arm_id}: joint names must be non-empty and distinct")
        for label, keys in (
            ("home_joints", self.home_joints),
            ("joint_ranges", self.joint_ranges),
            ("max_torque_nm", self.max_torque_nm),
        ):
            if set(keys) != set(names):
                raise ValueError(f"{self.arm_id}: {label} must cover exactly {names}")
        if not self.measured_joints or not set(self.measured_joints) <= set(names):
            raise ValueError(
                f"{self.arm_id}: measured joints {self.measured_joints} must be arm joints"
            )
        if any(low > high for low, high in self.joint_ranges.values()):
            raise ValueError(f"{self.arm_id}: empty joint range in {self.joint_ranges}")
        if any(limit <= 0.0 for limit in self.max_torque_nm.values()):
            raise ValueError(f"{self.arm_id}: torque limits must be positive")

    def require_workspace_guard(self) -> WorkspaceGuard:
        if not self.workspace.explicit:
            raise NoWorkspaceGuardError(
                f"no workspace guard defined for {self.arm_id}: pass floor height and front limit"
            )
        return self.workspace

    def on_bench(self, floor_z_m: float, min_x_m: float) -> ArmCalibrationSpec:
        """This spec with an explicit bench guard, keeping its keep-outs, frames and fingertip."""
        guard = replace(self.workspace, floor_z_m=floor_z_m, min_x_m=min_x_m, explicit=True)
        return replace(self, workspace=guard)

    @property
    def home_vector(self) -> np.ndarray:
        return np.asarray([self.home_joints[name] for name in self.joint_names], dtype=np.float64)

    def model_sha256(self) -> str:
        return model_sha256(self.model)

    def model_name(self) -> str:
        return model_name(self.model)


def model_sha256(model: RobotModel) -> str:
    """SHA-256 of the materialized URDF, mesh paths reduced to file names (same on every machine)."""
    return urdf_sha256(model.load().xml)


def urdf_sha256(xml: str) -> str:
    canonical = _FILENAME_ATTR.sub(
        lambda match: f'filename="{PurePosixPath(match.group(1)).name}"', xml
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def model_name(model: RobotModel) -> str:
    loaded = model.load()
    name = ET.fromstring(loaded.xml).get("name")
    if name:
        return name
    return loaded.source_path.stem if loaded.source_path is not None else "robot"


def reduced_model(model: RobotModel, joint_names: Sequence[str]) -> pin.Model:
    """Pinocchio model of ``joint_names`` (one-dimensional, in model order), the rest locked at neutral."""
    full = pin.buildModelFromXML(model.load().xml)
    names = tuple(joint_names)
    missing = set(names) - {str(name) for name in full.names[1:]}
    if missing:
        raise ValueError(f"model has no joints {sorted(missing)}")
    locked = [index for index, name in enumerate(full.names) if index and str(name) not in names]
    reduced = pin.buildReducedModel(full, locked, pin.neutral(full)) if locked else full
    if reduced.nq != len(names) or reduced.nv != len(names):
        raise ValueError("arm joints must each have one position and one velocity")
    if tuple(str(name) for name in reduced.names[1:]) != names:
        raise ValueError(f"model orders the arm joints differently from {list(names)}")
    return reduced


def frame_id(model: pin.Model, name: str) -> int:
    """Id of the body frame ``name``, else of the only frame with that name."""
    if model.existFrame(name, pin.FrameType.BODY):
        return int(model.getFrameId(name, pin.FrameType.BODY))
    if model.existFrame(name):
        return int(model.getFrameId(name))
    raise ValueError(f"model has no frame {name!r}")


def range_grid(ranges: Sequence[tuple[float, float]]) -> np.ndarray:
    """Every combination of each range's ends and middle, one pose per row."""
    return np.asarray(
        list(itertools.product(*((low, 0.5 * (low + high), high) for low, high in ranges)))
    )


def gravity_spans(
    model: pin.Model, joint_names: Sequence[str], ranges: Mapping[str, tuple[float, float]]
) -> np.ndarray:
    """Span (max - min) of each joint's model gravity torque over ``range_grid``."""
    data = model.createData()
    torques = np.vstack(
        [
            pin.computeGeneralizedGravity(model, data, q)
            for q in range_grid([ranges[name] for name in joint_names])
        ]
    )
    spans: np.ndarray = torques.max(axis=0) - torques.min(axis=0)
    return spans


def detect_measured_joints(
    model: pin.Model,
    joint_names: Sequence[str],
    ranges: Mapping[str, tuple[float, float]],
    resolution_nm: Mapping[str, float],
) -> list[str]:
    """Joints whose gravity torque spans more than rounding error and the motor's torque resolution."""
    spans = gravity_spans(model, joint_names, ranges)
    rounding = np.finfo(np.float64).eps * len(joint_names) * float(spans.max())
    return [
        name
        for name, span in zip(joint_names, spans, strict=True)
        if span > max(rounding, resolution_nm.get(name, 0.0))
    ]


def build_arm_spec(
    arm_id: str,
    model: RobotModel,
    joint_names: Sequence[str],
    *,
    base_frame: str | None = None,
    tip_frame: str | None = None,
    home_joints: Mapping[str, float] | Sequence[float] | None = None,
    gravity_links: Sequence[str] | None = None,
    measured_joints: Sequence[str] | None = None,
    torque_resolution_nm: Mapping[str, float] | None = None,
    joint_ranges: Mapping[str, tuple[float, float]] | None = None,
    max_torque_nm: Mapping[str, float] | None = None,
    workspace: WorkspaceGuard | None = None,
    gains: Mapping[str, tuple[float, float]] | None = None,
) -> ArmCalibrationSpec:
    """Spec for one arm of ``model``, deriving every field not given (see the module doc).

    ``joint_ranges`` must stay inside the URDF limits. ``max_torque_nm`` may name only some
    joints; the rest get their URDF effort limit. ``home_joints`` defaults to zeros clipped
    into the limits.
    """
    names = list(joint_names)
    pin_model = reduced_model(model, names)
    joints = list(model.load().joints)
    by_name = {joint.name: joint for joint in joints}
    ranges = {name: urdf_limits(by_name[name]) for name in names}
    for joint, (low, high) in (joint_ranges or {}).items():
        lower, upper = ranges[joint]
        if not lower <= low <= high <= upper:
            raise ValueError(f"{arm_id}: {joint} range {low}..{high} leaves its URDF limits")
        ranges[joint] = (float(low), float(high))
    if home_joints is None:
        home = {name: min(max(0.0, ranges[name][0]), ranges[name][1]) for name in names}
    elif isinstance(home_joints, Mapping):
        home = {name: float(home_joints[name]) for name in names}
    else:
        home = dict(zip(names, map(float, home_joints), strict=True))
    measured = (
        list(measured_joints)
        if measured_joints is not None
        else detect_measured_joints(pin_model, names, ranges, torque_resolution_nm or {})
    )
    torques: dict[str, float] = {}
    for name in names:
        value = (max_torque_nm or {}).get(name, by_name[name].effort)
        if value is None:
            raise ValueError(f"{arm_id}: {name} has no URDF effort limit; pass max_torque_nm")
        torques[name] = float(value)
    base = base_frame or by_name[names[0]].parent_link
    tip = tip_frame or default_tip_frame(joints, names)
    if workspace is None:
        data = pin_model.createData()
        pin.framesForwardKinematics(pin_model, data, np.asarray([home[name] for name in names]))
        floor = -float(data.oMf[frame_id(pin_model, base)].translation[2])
        workspace = WorkspaceGuard(floor_z_m=floor, explicit=False)
    guard = workspace
    for frame in (base, tip, *(guard.clearance_frames or ())):
        frame_id(pin_model, frame)
    if guard.clearance_frames is None:
        frames = default_clearance_frames(pin_model, joints, names, measured, ranges, home)
        guard = replace(guard, clearance_frames=tuple(frames))
    return ArmCalibrationSpec(
        arm_id=arm_id,
        model=model,
        joint_names=names,
        base_frame=base,
        tip_frame=tip,
        home_joints=home,
        gravity_links=(
            list(gravity_links)
            if gravity_links is not None
            else default_gravity_links(joints, names, measured)
        ),
        measured_joints=measured,
        joint_ranges=ranges,
        max_torque_nm=torques,
        workspace=guard,
        gains=dict(gains or {}),
    )


def specs_from_damiao_robot(
    robot: DamiaoRobot,
    *,
    workspace: WorkspaceGuard | Mapping[str, WorkspaceGuard] | None = None,
    arm_id: str | None = None,
    joint_ranges: Mapping[str, tuple[float, float]] | None = None,
    max_torque_nm: Mapping[str, float] | None = None,
) -> list[ArmCalibrationSpec]:
    """One spec per bus, in bus order; arm ids are ``robot.arm_ids()`` (``arm_id`` overrides the name).

    ``workspace`` is one guard for every arm, or guards by arm ("left") or bus ("left_arm")
    name. With several arms each guard also keeps out of the other arms' halves. A pose may
    load a motor up to its datasheet continuous torque; a motor of unknown type (a custom
    profile) gets its joint's URDF effort unless ``max_torque_nm`` names it.
    """
    model = robot.gravity_model if robot.gravity_model is not None else robot.model
    for label, values in (("joint ranges", joint_ranges), ("torque caps", max_torque_nm)):
        unknown = set(values or {}) - set(robot.arm_joint_names)
        if unknown:
            raise ValueError(f"{label} given for joints of no arm: {sorted(unknown)}")
    if arm_id is None and not robot.name:
        raise ValueError("the robot has no name to make arm ids from; pass arm_id")
    named = robot if arm_id is None else replace(robot, name=arm_id)
    specs: list[ArmCalibrationSpec] = []
    offset = 0
    for bus, spec_id in zip(robot.buses, named.arm_ids(), strict=True):
        joints = [motor.name for motor in bus.arm]
        guard = (
            workspace
            if workspace is None or isinstance(workspace, WorkspaceGuard)
            else workspace.get(arm_name(bus.name), workspace.get(bus.name))
        )
        caps = {
            motor.name: DAMIAO_CONTINUOUS_TORQUE_NM[motor.type]
            for motor in bus.arm
            if motor.type in DAMIAO_CONTINUOUS_TORQUE_NM
        }
        caps.update(
            {name: value for name, value in (max_torque_nm or {}).items() if name in joints}
        )
        specs.append(
            build_arm_spec(
                spec_id,
                model,
                joints,
                home_joints=(
                    None
                    if robot.home_joints is None
                    else robot.home_joints[offset : offset + len(joints)]
                ),
                joint_ranges={k: v for k, v in (joint_ranges or {}).items() if k in joints},
                max_torque_nm=caps,
                torque_resolution_nm={
                    motor.name: 2.0 * motor_torque_limit_nm(motor) / (2**MIT_TORQUE_BITS - 1)
                    for motor in bus.arm
                },
                workspace=guard,
                gains={
                    name: (float(kp), float(kd))
                    for name, kp, kd in zip(joints, bus.kp, bus.kd, strict=True)
                },
            )
        )
        offset += len(joints)
    return with_midline_keep_outs(specs) if len(specs) > 1 else specs


def with_midline_keep_outs(specs: Sequence[ArmCalibrationSpec]) -> list[ArmCalibrationSpec]:
    """Each spec with a keep-out half-space toward every other arm, halfway between the bases."""
    model = pin.buildModelFromXML(specs[0].model.load().xml)
    data = model.createData()
    pin.framesForwardKinematics(model, data, pin.neutral(model))
    bases = [np.asarray(data.oMf[frame_id(model, spec.base_frame)].translation) for spec in specs]
    result: list[ArmCalibrationSpec] = []
    for index, spec in enumerate(specs):
        spaces = []
        for other_index, other in enumerate(specs):
            if other_index == index:
                continue
            toward = bases[other_index] - bases[index]
            distance = float(np.linalg.norm(toward))
            if distance == 0.0:
                raise ValueError(f"{spec.arm_id} and {other.arm_id} share a base position")
            nx, ny, nz = (float(v) for v in toward / distance)
            spaces.append(HalfSpace((nx, ny, nz), distance / 2.0, f"midline to {other.arm_id}"))
        guard = replace(spec.workspace, keep_out=(*spec.workspace.keep_out, *spaces))
        result.append(replace(spec, workspace=guard))
    return result


def arm_name(bus_name: str) -> str:
    """``left_arm`` -> ``left``; a bus called ``arm`` gives ``arm``."""
    return bus_name.removesuffix("_arm") or bus_name


def motor_torque_limit_nm(motor: DamiaoMotor) -> float:
    """The torque range a Damiao motor's MIT frames encode: its profile's ``t_max``, else stock."""
    if motor.profile is not None:
        return float(motor.profile.t_max)
    if motor.type not in DAMIAO_STOCK_TORQUE_LIMIT_NM:
        raise ValueError(
            f"no torque limit known for Damiao motor type {motor.type!r} ({motor.name})"
        )
    return DAMIAO_STOCK_TORQUE_LIMIT_NM[motor.type]


def motor_rated_torque_nm(motor: DamiaoMotor) -> float | None:
    """The stock MIT torque range of a motor's type; None for a custom profile, which names no type."""
    return DAMIAO_STOCK_TORQUE_LIMIT_NM.get(motor.type)


def urdf_limits(joint: JointDescription) -> tuple[float, float]:
    if joint.type == "continuous" or joint.lower is None or joint.upper is None:
        return CONTINUOUS_RANGE_RAD
    return (float(joint.lower), float(joint.upper))


def default_gravity_links(
    joints: Sequence[JointDescription], joint_names: Sequence[str], measured: Sequence[str]
) -> list[str]:
    """Child links of the arm joints at or below a measured joint, in joint order."""
    by_child = {joint.child_link: joint for joint in joints}
    by_name = {joint.name: joint for joint in joints}
    links = []
    for name in joint_names:
        joint: JointDescription | None = by_name[name]
        while joint is not None and joint.name not in measured:
            joint = by_child.get(joint.parent_link)
        if joint is not None:
            links.append(by_name[name].child_link)
    return links


def merged_links(
    joints: Sequence[JointDescription], joint_names: Sequence[str], link: str
) -> list[str]:
    """Links below ``link`` reached without crossing an arm joint: pinocchio merges them into it."""
    children = _children(joints)
    found: list[str] = []
    stack = [link]
    while stack:
        for joint in children.get(stack.pop(0), []):
            if joint.name not in joint_names:
                found.append(joint.child_link)
                stack.append(joint.child_link)
    return found


def default_tip_frame(joints: Sequence[JointDescription], joint_names: Sequence[str]) -> str:
    """A grasp or TCP frame rigidly below the last arm joint, else the end of its fixed chain."""
    children = _children(joints)
    tip = next(joint for joint in joints if joint.name == joint_names[-1]).child_link
    rigid = [tip, *merged_links(joints, joint_names, tip)]
    for hint in TIP_FRAME_HINTS:
        named = [link for link in rigid if hint in link.lower()]
        if named:
            return named[0]
    while len(fixed := [j for j in children.get(tip, []) if j.type == "fixed"]) == 1:
        tip = fixed[0].child_link
    return tip


def default_clearance_frames(
    model: pin.Model,
    joints: Sequence[JointDescription],
    joint_names: Sequence[str],
    measured: Sequence[str],
    ranges: Mapping[str, tuple[float, float]],
    home: Mapping[str, float],
) -> list[str]:
    """Links below the first arm joint whose origin moves when the measured joints do (others at home)."""
    children = _children(joints)
    below: list[str] = []
    stack = [next(joint for joint in joints if joint.name == joint_names[0]).child_link]
    while stack:
        below.append(stack.pop(0))
        stack.extend(joint.child_link for joint in children.get(below[-1], []))
    frames = [link for link in below if model.existFrame(link, pin.FrameType.BODY)]
    ids = [frame_id(model, link) for link in frames]
    data = model.createData()
    columns = [list(joint_names).index(name) for name in measured]
    positions = []
    for values in range_grid([ranges[name] for name in measured]):
        q = np.asarray([home[name] for name in joint_names], dtype=np.float64)
        q[columns] = values
        pin.framesForwardKinematics(model, data, q)
        positions.append([data.oMf[index].translation.copy() for index in ids])
    stacked = np.asarray(positions)
    travel = np.linalg.norm(stacked - stacked[0], axis=2).max(axis=0)
    tolerance = np.finfo(np.float64).eps * len(joint_names) * float(np.abs(stacked).max())
    return [frame for frame, moved in zip(frames, travel, strict=True) if moved > tolerance]


def _children(joints: Sequence[JointDescription]) -> dict[str, list[JointDescription]]:
    children: dict[str, list[JointDescription]] = {}
    for joint in joints:
        children.setdefault(joint.parent_link, []).append(joint)
    return children
