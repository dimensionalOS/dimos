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

An :class:`ArmCalibrationSpec` names one arm of a robot (``arm_id``), the
mass-bearing URDF it is calibrated against and, in motor order, the joints that
move it. Everything else is derived from the model unless given:

- ``gravity_links``: the child link of every arm joint at or below the first
  measured joint. Pinocchio merges a link's rigidly attached descendants (fixed
  joints, and the finger joints the calibration locks) into that link's body,
  so a fitted "gripper" carries its fingers too; :func:`merged_links` lists them.
- ``measured_joints``: joints whose model gravity torque varies by at least
  ``MEASURED_MIN_SPAN_NM`` (and ``MEASURED_MIN_SPAN_FRACTION`` of the largest
  span) over random poses inside ``joint_ranges``. A base yaw joint, whose axis
  is vertical, never carries gravity and drops out, as does a wrist roll whose
  load sits on its axis.
- ``joint_ranges``: the URDF limits shrunk on each side by the larger of
  ``range_margin_fraction`` of the span and ``range_margin_rad``.
- ``max_torque_nm``: for Damiao robots, the motor type's rated peak torque
  (``DAMIAO_STOCK_TORQUE_LIMIT_NM``) times ``torque_safety_factor``; no
  identification pose may need more in the model. A custom firmware profile's
  ``t_max`` is the torque range of its MIT frames, not a rating, so a motor of
  unknown type needs an explicit cap, or gets the smallest known rating with a
  warning.
- ``workspace``: a :class:`WorkspaceGuard`. Without one the spec gets an
  implicit guard (``explicit`` False) with the model root's z = 0 plane as the
  floor and nothing else, which knows nothing of the table, the space behind
  the base or a torso: pose generation refuses it, so anything that moves an
  arm needs an explicit guard (:meth:`WorkspaceGuard.for_bench`). Its
  clearance frames default to every link below the arm's first joint that
  moves when the measured joints do. On a robot with several arms, each arm's
  guard also keeps out of the other arms' halves, split halfway between the
  bases in the model; a torso or other bodies need keep-out half-spaces of
  their own (see ``dual_openyam.calibration_workspace``).
- ``grid_step_rad``: the pose grid steps of the measured joints, derived from
  the gravity torque spans unless given (see ``gravity_poses.grid_axes``).

Workspace coordinates are metres with the origin at ``base_frame`` and the axes
of the model root, whose -z is gravity, so z is height and the floor is a plane.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
import hashlib
import math
from pathlib import PurePosixPath
import re
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.robot.manipulators.damiao.robot import DamiaoBus, DamiaoMotor, DamiaoRobot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Vector3 = tuple[float, float, float]

DEFAULT_RANGE_MARGIN_FRACTION = 0.1
DEFAULT_RANGE_MARGIN_RAD = math.radians(5.0)
# Pose generation range of a joint without position limits.
CONTINUOUS_RANGE_RAD = (-math.pi, math.pi)
# An identification pose may load each motor with at most this share of its
# type's peak torque: 10 Nm on a DM4340 (28 Nm peak), about its continuous
# rating and the cap the bench used; 3.6 Nm on a DM4310.
DEFAULT_TORQUE_SAFETY_FACTOR = 10.0 / 28.0
# Peak torque of the stock Damiao MIT scaling, by motor type: the range of the
# torque slot the connection clamps to, and the rating the calibration caps
# scale. A custom profile's t_max is only the first of these.
DAMIAO_STOCK_TORQUE_LIMIT_NM: dict[str, float] = {
    "DM4310": 10.0,
    "DM4340": 28.0,
    "DM8009": 54.0,
}
# A joint is measured when its model gravity torque spans at least this much
# over the sampled poses (about a small joint's Coulomb friction) ...
MEASURED_MIN_SPAN_NM = 0.1
# ... and at least this share of the largest span among the arm's joints.
MEASURED_MIN_SPAN_FRACTION = 0.01
DETECTION_SAMPLES = 256
DETECTION_SEED = 0
# A clearance frame must move at least this far when the measured joints move;
# frames on a joint axis near the base (shoulder, yaw) never approach the floor.
MOBILE_FRAME_MIN_TRAVEL_M = 0.005
# Preferred tip frame names below the last arm joint, in order.
TIP_FRAME_HINTS = ("grasp", "tcp")

NO_WORKSPACE_GUARD_HINT = "pass floor height and front limit"

_ARM_ID = re.compile(r"^[A-Za-z0-9_.-]+(/[A-Za-z0-9_.-]+)*$")
_FILENAME_ATTR = re.compile(r'filename="([^"]*)"')


class NoWorkspaceGuardError(ValueError):
    """An arm would be moved with only the implicit, model-derived workspace guard."""


@dataclass(frozen=True)
class HalfSpace:
    """A keep-out region: every point ``p`` with ``normal . p > offset_m`` (workspace frame)."""

    normal: Vector3
    offset_m: float
    name: str = "keep-out"

    def violated_by(self, points: np.ndarray) -> bool:
        """True when any of ``points`` (one per row, or a single point) lies inside."""
        values = np.atleast_2d(np.asarray(points, dtype=np.float64)) @ np.asarray(self.normal)
        return bool((values > self.offset_m).any())


@dataclass(frozen=True)
class WorkspaceGuard:
    """Where an arm may go during calibration, in the workspace frame (see the module doc).

    Attributes:
        floor_z_m: The surface under the arm (table or floor), as a height.
        pose_clearance_m: Every checked point stays this far above the floor at
            an identification pose.
        transition_clearance_m: The same along the joint-space ramp between
            poses; a ramp may start or end lower only where its end pose is.
        min_x_m: When set, the fingertip stays in front of this x at a pose.
        keep_out: Regions no checked point may enter, e.g. another arm's half.
        clearance_frames: Frames whose origins are checked; ``None`` derives
            them from the model (see :func:`default_clearance_frames`).
        point_offset_m: The fingertip in the spec's tip frame.
        explicit: Whether someone described the arm's surroundings. The guard
            a spec gets when none is given is not (False): its floor is only
            the model root's z = 0 plane, and nothing moves an arm with it.
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
        if self.pose_clearance_m < 0.0 or self.transition_clearance_m < 0.0:
            raise ValueError("workspace clearances must not be negative")

    @classmethod
    def for_bench(
        cls,
        floor_z_m: float,
        min_x_m: float,
        *,
        pose_clearance_m: float = 0.15,
        transition_clearance_m: float = 0.11,
        keep_out: Sequence[HalfSpace] = (),
        clearance_frames: Sequence[str] | None = None,
        point_offset_m: Vector3 = (0.0, 0.0, 0.0),
    ) -> WorkspaceGuard:
        """An explicit guard for an arm on a bench: the table height and the in-front line.

        Args:
            floor_z_m: The table's height in the workspace frame, e.g. -0.03
                for a base 30 mm above it.
            min_x_m: The fingertip stays in front of this x at a pose.
            pose_clearance_m: Height above the table kept at a pose.
            transition_clearance_m: Height above the table kept between poses.
            keep_out: Further keep-out half-spaces.
            clearance_frames: Frames to check; ``None`` derives them.
            point_offset_m: The fingertip in the tip frame.
        """
        return cls(
            floor_z_m=floor_z_m,
            pose_clearance_m=pose_clearance_m,
            transition_clearance_m=transition_clearance_m,
            min_x_m=min_x_m,
            keep_out=tuple(keep_out),
            clearance_frames=None if clearance_frames is None else tuple(clearance_frames),
            point_offset_m=point_offset_m,
        )

    @property
    def min_pose_z_m(self) -> float:
        return self.floor_z_m + self.pose_clearance_m

    @property
    def min_transition_z_m(self) -> float:
        return self.floor_z_m + self.transition_clearance_m


@dataclass(frozen=True)
class ArmCalibrationSpec:
    """Everything the calibration math needs to know about one arm.

    Build one with :func:`build_arm_spec` or :func:`specs_from_damiao_robot`,
    which derive the defaults from the model; the fields here are all explicit.

    Attributes:
        arm_id: Names the arm and its calibration file, e.g. "openyam" or
            "dual_openyam/left".
        model: The mass-bearing URDF the arm is calibrated against.
        joint_names: The arm joints in motor order; every other joint of the
            model is locked at its neutral position.
        base_frame: The frame the workspace is measured from.
        tip_frame: The frame the fingertip offset is applied in.
        home_joints: The posture each pose sequence starts from, rad.
        gravity_links: Links whose mass and centre of mass are fitted.
        measured_joints: Joints whose resting torque is measured.
        joint_ranges: Range per joint for pose generation, rad.
        max_torque_nm: Most model gravity torque a pose may need, per joint.
        workspace: Where the arm may go.
        gains: The robot's configured ``(kp, kd)`` per joint, if known.
        grid_step_rad: Pose grid step per measured joint, rad; joints left out
            (or every joint, when ``None``) get the derived step.
    """

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
    grid_step_rad: dict[str, float] | None = None

    def __post_init__(self) -> None:
        if not _ARM_ID.match(self.arm_id) or ".." in self.arm_id.split("/"):
            raise ValueError(f"arm_id must be a relative path of plain names, got {self.arm_id!r}")
        names = list(self.joint_names)
        if not names or len(set(names)) != len(names):
            raise ValueError(f"{self.arm_id}: joint names must be non-empty and distinct")
        for label, keys in (
            ("home_joints", self.home_joints),
            ("joint_ranges", self.joint_ranges),
            ("max_torque_nm", self.max_torque_nm),
        ):
            if set(keys) != set(names):
                raise ValueError(f"{self.arm_id}: {label} must cover exactly {names}")
        unknown = set(self.measured_joints) - set(names)
        if unknown:
            raise ValueError(f"{self.arm_id}: measured joints {sorted(unknown)} are not arm joints")
        if not self.measured_joints:
            raise ValueError(f"{self.arm_id}: no joint carries a measurable gravity torque")
        for joint, (low, high) in self.joint_ranges.items():
            if not low <= high:
                raise ValueError(f"{self.arm_id}: {joint} range {low}..{high} is empty")
        if any(limit <= 0.0 for limit in self.max_torque_nm.values()):
            raise ValueError(f"{self.arm_id}: torque limits must be positive")
        if self.grid_step_rad is not None:
            unmeasured = set(self.grid_step_rad) - set(self.measured_joints)
            if unmeasured:
                raise ValueError(
                    f"{self.arm_id}: grid steps given for joints that are not measured: "
                    f"{sorted(unmeasured)}"
                )
            if not all(math.isfinite(s) and s > 0.0 for s in self.grid_step_rad.values()):
                raise ValueError(f"{self.arm_id}: grid steps must be positive")

    def require_workspace_guard(self) -> WorkspaceGuard:
        """The guard, when it is explicit; raises ``NoWorkspaceGuardError`` before any motion."""
        if not self.workspace.explicit:
            raise NoWorkspaceGuardError(
                f"no workspace guard defined for {self.arm_id}: {NO_WORKSPACE_GUARD_HINT}"
            )
        return self.workspace

    @property
    def home_vector(self) -> np.ndarray:
        """Home posture in ``joint_names`` order, rad."""
        return np.asarray([self.home_joints[name] for name in self.joint_names], dtype=np.float64)

    def model_sha256(self) -> str:
        """Fingerprint of the model this spec uses; see :func:`model_sha256`."""
        return model_sha256(self.model)

    def model_name(self) -> str:
        """The URDF's robot name, or the source file's stem."""
        return model_name(self.model)


def model_sha256(model: RobotModel) -> str:
    """SHA-256 of the materialized URDF with every mesh path reduced to its file name.

    The materialized XML is what pinocchio is built from, so in-code edits (renamed
    joints, widened limits, xacro arguments) change the hash; mesh paths are made
    absolute per checkout, so they are reduced to file names to keep the hash the
    same on every machine.
    """
    return urdf_sha256(model.load().xml)


def urdf_sha256(xml: str) -> str:
    """:func:`model_sha256` of a materialized URDF text, e.g. ``model.load().xml``."""
    canonical = _FILENAME_ATTR.sub(
        lambda match: f'filename="{PurePosixPath(match.group(1)).name}"', xml
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def model_name(model: RobotModel) -> str:
    name = ET.fromstring(model.load().xml).get("name")
    if name:
        return name
    source = model.load().source_path
    return source.stem if source is not None else "robot"


def reduced_model(model: RobotModel, joint_names: Sequence[str]) -> pin.Model:
    """Pinocchio model of ``joint_names``, every other joint locked at neutral.

    Raises ``ValueError`` when a joint is missing, is not one-dimensional, or the
    model orders the arm joints differently from ``joint_names``.
    """
    full = pin.buildModelFromXML(model.load().xml)
    names = tuple(joint_names)
    present = {str(name) for name in full.names[1:]}
    missing = [name for name in names if name not in present]
    if missing:
        raise ValueError(f"model has no joints {missing}")
    locked = [index for index, name in enumerate(full.names) if index and str(name) not in names]
    reduced = pin.buildReducedModel(full, locked, pin.neutral(full)) if locked else full
    if reduced.nq != len(names) or reduced.nv != len(names):
        raise ValueError("arm joints must each have one position and one velocity")
    if tuple(str(name) for name in reduced.names[1:]) != names:
        raise ValueError(f"model orders the arm joints differently from {list(names)}")
    return reduced


def frame_id(model: pin.Model, name: str) -> int:
    """Id of the link (body) frame ``name``, else of the only frame with that name."""
    if model.existFrame(name, pin.FrameType.BODY):
        return int(model.getFrameId(name, pin.FrameType.BODY))
    if model.existFrame(name):
        return int(model.getFrameId(name))
    raise ValueError(f"model has no frame {name!r}")


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
    joint_ranges: Mapping[str, tuple[float, float]] | None = None,
    range_margin_fraction: float = DEFAULT_RANGE_MARGIN_FRACTION,
    range_margin_rad: float = DEFAULT_RANGE_MARGIN_RAD,
    max_torque_nm: Mapping[str, float] | None = None,
    workspace: WorkspaceGuard | None = None,
    gains: Mapping[str, tuple[float, float]] | None = None,
    grid_step_rad: Mapping[str, float] | None = None,
) -> ArmCalibrationSpec:
    """Spec for one arm of ``model``, deriving every field not given (see the module doc).

    Args:
        arm_id: The arm's name, e.g. "dual_openyam/left".
        model: The mass-bearing URDF.
        joint_names: The arm joints in motor order.
        base_frame: Defaults to the parent link of the first arm joint.
        tip_frame: Defaults to a frame named like "grasp" or "tcp" rigidly below
            the last arm joint, else the end of its chain of fixed joints.
        home_joints: Mapping or motor-order sequence; defaults to zeros clipped
            into the URDF limits.
        gravity_links: Overrides the fitted links.
        measured_joints: Overrides the auto-detected measured joints.
        joint_ranges: Per-joint overrides of the pose-generation range, rad;
            each must stay inside the URDF limits.
        range_margin_fraction: Default range margin as a share of the span.
        range_margin_rad: Smallest default range margin.
        max_torque_nm: Per-joint torque caps; defaults to the URDF effort limit.
        workspace: Defaults to an implicit guard (``explicit`` False) with the
            model root's z = 0 plane as the floor and nothing else; poses are
            not generated for it.
        gains: The robot's configured ``(kp, kd)`` per joint.
        grid_step_rad: Pose grid steps of measured joints, rad.
    """
    names = list(joint_names)
    pin_model = reduced_model(model, names)
    descriptions = {joint.name: joint for joint in model.load().joints}
    ranges = default_joint_ranges(
        [descriptions[name] for name in names],
        margin_fraction=range_margin_fraction,
        margin_rad=range_margin_rad,
    )
    for joint, (low, high) in (joint_ranges or {}).items():
        if joint not in ranges:
            raise ValueError(f"{arm_id}: joint range given for unknown joint {joint!r}")
        lower, upper = urdf_limits(descriptions[joint])
        if not lower - 1e-9 <= low <= high <= upper + 1e-9:
            raise ValueError(
                f"{arm_id}: {joint} range {low:.3f}..{high:.3f} rad leaves the URDF limits "
                f"{lower:.3f}..{upper:.3f} rad"
            )
        ranges[joint] = (float(low), float(high))
    home = _home_joints(names, home_joints, descriptions)
    if measured_joints is None:
        measured = detect_measured_joints(pin_model, names, ranges)
    else:
        measured = list(measured_joints)
    links = (
        list(gravity_links)
        if gravity_links is not None
        else default_gravity_links(list(descriptions.values()), names, measured)
    )
    base = base_frame or descriptions[names[0]].parent_link
    tip = tip_frame or default_tip_frame(list(descriptions.values()), names)
    if max_torque_nm is None:
        torques = {name: _urdf_effort(descriptions[name]) for name in names}
    else:
        torques = {name: float(max_torque_nm[name]) for name in names}
    for frame in (base, tip, *((workspace.clearance_frames or ()) if workspace else ())):
        frame_id(pin_model, frame)  # raises for an unknown frame
    guard = workspace or WorkspaceGuard(
        floor_z_m=root_floor_z(pin_model, base, home, names), explicit=False
    )
    if guard.clearance_frames is None:
        guard = replace(
            guard,
            clearance_frames=tuple(
                default_clearance_frames(
                    pin_model, list(descriptions.values()), names, measured, ranges, home
                )
            ),
        )
    return ArmCalibrationSpec(
        arm_id=arm_id,
        model=model,
        joint_names=names,
        base_frame=base,
        tip_frame=tip,
        home_joints=home,
        gravity_links=links,
        measured_joints=measured,
        joint_ranges=ranges,
        max_torque_nm=torques,
        workspace=guard,
        gains=dict(gains or {}),
        grid_step_rad=None if grid_step_rad is None else dict(grid_step_rad),
    )


def root_floor_z(
    model: pin.Model, base_frame: str, home: Mapping[str, float], joint_names: Sequence[str]
) -> float:
    """Height of the model root's z = 0 plane in the workspace frame (below the base)."""
    data = model.createData()
    pin.framesForwardKinematics(
        model, data, np.asarray([home[name] for name in joint_names], dtype=np.float64)
    )
    return -float(data.oMf[frame_id(model, base_frame)].translation[2])


def specs_from_damiao_robot(
    robot: DamiaoRobot,
    *,
    workspace: WorkspaceGuard | Mapping[str, WorkspaceGuard] | None = None,
    arm_id: str | None = None,
    joint_ranges: Mapping[str, tuple[float, float]] | None = None,
    torque_safety_factor: float = DEFAULT_TORQUE_SAFETY_FACTOR,
    max_torque_nm: Mapping[str, float] | None = None,
    grid_step_rad: Mapping[str, float] | None = None,
) -> list[ArmCalibrationSpec]:
    """One spec per bus of a Damiao robot, in bus order.

    Args:
        robot: The robot; its ``gravity_model`` is the mass-bearing model.
        workspace: One guard for every arm, or guards by arm name ("left") or
            bus name ("left_arm"); arms without one get the implicit guard,
            for which no poses are generated. On several buses each arm's
            guard also gets a keep-out half-space toward every other arm,
            halfway between their bases in the model.
        arm_id: Overrides ``robot.name`` in the arm ids, which are those of
            ``robot.arm_ids()``: a single-bus robot's arm is the name, the
            others ``f"{name}/{arm}"``, ``arm`` being the bus name without its
            "_arm" suffix. Required when the robot has no name.
        joint_ranges: Per-joint range overrides, rad, for any of the arms.
        torque_safety_factor: Share of each motor's rated peak torque a pose
            may use.
        max_torque_nm: Per-joint pose torque caps, used as given; needed for a
            motor of unknown type, such as one on a custom firmware profile.
        grid_step_rad: Per-joint pose grid steps, rad, for any of the arms.
    """
    model = robot.gravity_model if robot.gravity_model is not None else robot.model
    for label, values in (
        ("joint ranges", joint_ranges),
        ("torque caps", max_torque_nm),
        ("grid steps", grid_step_rad),
    ):
        unknown = set(values or {}) - set(robot.arm_joint_names)
        if unknown:
            raise ValueError(f"{label} given for joints of no arm: {sorted(unknown)}")
    if arm_id is None and not robot.name:
        raise ValueError("the robot has no name to make arm ids from; pass arm_id")
    named = robot if arm_id is None or arm_id == robot.name else replace(robot, name=arm_id)
    specs: list[ArmCalibrationSpec] = []
    offset = 0
    for bus, spec_id in zip(robot.buses, named.arm_ids(), strict=True):
        joints = [motor.name for motor in bus.arm]
        arm = arm_name(bus.name)
        guard: WorkspaceGuard | None
        if isinstance(workspace, WorkspaceGuard) or workspace is None:
            guard = workspace
        else:
            guard = workspace.get(arm, workspace.get(bus.name))
        home = (
            robot.home_joints[offset : offset + len(joints)]
            if robot.home_joints is not None
            else None
        )
        specs.append(
            build_arm_spec(
                spec_id,
                model,
                joints,
                home_joints=home,
                joint_ranges={
                    name: value for name, value in (joint_ranges or {}).items() if name in joints
                },
                max_torque_nm=_torque_caps(spec_id, bus, torque_safety_factor, max_torque_nm),
                workspace=guard,
                gains={
                    name: (float(kp), float(kd))
                    for name, kp, kd in zip(joints, bus.kp, bus.kd, strict=True)
                },
                grid_step_rad=(
                    None
                    if grid_step_rad is None
                    else {name: step for name, step in grid_step_rad.items() if name in joints}
                ),
            )
        )
        offset += len(joints)
    return with_midline_keep_outs(specs) if len(specs) > 1 else specs


def with_midline_keep_outs(specs: Sequence[ArmCalibrationSpec]) -> list[ArmCalibrationSpec]:
    """Each spec with a keep-out half-space toward every other arm, halfway between the bases.

    The bases are placed by the specs' (shared) model with every joint at
    neutral; base frames sit above the arm joints, so the arms' postures do not
    move them. A spec's guard keeps its own keep-outs, explicit or not.
    """
    model = pin.buildModelFromXML(specs[0].model.load().xml)
    data = model.createData()
    pin.framesForwardKinematics(model, data, pin.neutral(model))
    bases = [
        np.asarray(data.oMf[frame_id(model, spec.base_frame)].translation, dtype=np.float64)
        for spec in specs
    ]
    result: list[ArmCalibrationSpec] = []
    for index, spec in enumerate(specs):
        spaces: list[HalfSpace] = []
        for other_index, other in enumerate(specs):
            if other_index == index:
                continue
            toward = bases[other_index] - bases[index]
            distance = float(np.linalg.norm(toward))
            if distance < 1e-9:
                raise ValueError(f"{spec.arm_id} and {other.arm_id} share a base position")
            nx, ny, nz = (float(v) for v in toward / distance)
            spaces.append(
                HalfSpace(
                    normal=(nx, ny, nz),
                    offset_m=distance / 2.0,
                    name=f"midline to {other.arm_id}",
                )
            )
        guard = replace(spec.workspace, keep_out=(*spec.workspace.keep_out, *spaces))
        result.append(replace(spec, workspace=guard))
    return result


def arm_name(bus_name: str) -> str:
    """``left_arm`` -> ``left``; a bus called ``arm`` gives ``arm``."""
    return bus_name.removesuffix("_arm") or bus_name


def motor_torque_limit_nm(motor: DamiaoMotor) -> float:
    """The torque range of a Damiao motor's MIT frames: its custom profile's ``t_max``, else stock.

    This is what the torque slot can encode, which the connection clamps to.
    It is not what the motor can deliver: see :func:`motor_rated_torque_nm`.
    """
    if motor.profile is not None:
        return float(motor.profile.t_max)
    try:
        return DAMIAO_STOCK_TORQUE_LIMIT_NM[motor.type]
    except KeyError:
        raise ValueError(
            f"no torque limit known for Damiao motor type {motor.type!r} ({motor.name})"
        ) from None


def motor_rated_torque_nm(motor: DamiaoMotor) -> float | None:
    """Rated peak torque of a Damiao motor by its type, or ``None`` for an unknown type.

    A motor on a custom firmware profile has type "profile", which names no
    motor: its profile's ``t_max`` only scales the MIT frames.
    """
    return DAMIAO_STOCK_TORQUE_LIMIT_NM.get(motor.type)


def _torque_caps(
    arm_id: str,
    bus: DamiaoBus,
    safety_factor: float,
    explicit: Mapping[str, float] | None,
) -> dict[str, float]:
    """Pose torque cap per arm motor: explicit, else the rating times ``safety_factor``."""
    fallback = min(DAMIAO_STOCK_TORQUE_LIMIT_NM.values())
    caps: dict[str, float] = {}
    unrated: list[str] = []
    for motor in bus.arm:
        if explicit is not None and motor.name in explicit:
            caps[motor.name] = float(explicit[motor.name])
            continue
        rating = motor_rated_torque_nm(motor)
        if rating is None:
            unrated.append(motor.name)
            rating = fallback
        caps[motor.name] = rating * safety_factor
    if unrated:
        logger.warning(
            f"{arm_id}: no torque rating is known for {unrated} (a custom profile's t_max is "
            f"its MIT range, not a rating); capping their pose torque at the smallest known "
            f"rating, {fallback:g} Nm x {safety_factor:.3g} = {fallback * safety_factor:.2f} Nm. "
            "Pass max_torque_nm to set them."
        )
    return caps


def urdf_limits(joint: JointDescription) -> tuple[float, float]:
    if joint.type == "continuous" or joint.lower is None or joint.upper is None:
        return CONTINUOUS_RANGE_RAD
    return (float(joint.lower), float(joint.upper))


def default_joint_ranges(
    joints: Sequence[JointDescription],
    *,
    margin_fraction: float = DEFAULT_RANGE_MARGIN_FRACTION,
    margin_rad: float = DEFAULT_RANGE_MARGIN_RAD,
) -> dict[str, tuple[float, float]]:
    """URDF limits shrunk by ``max(margin_fraction * span, margin_rad)`` at each end."""
    ranges: dict[str, tuple[float, float]] = {}
    for joint in joints:
        lower, upper = urdf_limits(joint)
        margin = max(margin_fraction * (upper - lower), margin_rad)
        if upper - lower <= 2.0 * margin:
            middle = 0.5 * (lower + upper)
            ranges[joint.name] = (middle, middle)
        else:
            ranges[joint.name] = (lower + margin, upper - margin)
    return ranges


def detect_measured_joints(
    model: pin.Model,
    joint_names: Sequence[str],
    ranges: Mapping[str, tuple[float, float]],
    *,
    samples: int = DETECTION_SAMPLES,
    seed: int = DETECTION_SEED,
) -> list[str]:
    """Joints whose model gravity torque varies measurably over random poses in ``ranges``.

    The span of ``g_j(q) = Y_j(q) @ phi_urdf`` (the gravity regressor row times
    the URDF parameters) over the samples must reach ``MEASURED_MIN_SPAN_NM``
    and ``MEASURED_MIN_SPAN_FRACTION`` of the largest joint's span.
    """
    torques = gravity_samples(model, joint_names, ranges, samples=samples, seed=seed)
    spans = torques.max(axis=0) - torques.min(axis=0)
    limit = max(MEASURED_MIN_SPAN_NM, MEASURED_MIN_SPAN_FRACTION * float(spans.max()))
    return [name for name, span in zip(joint_names, spans, strict=True) if span >= limit]


def gravity_samples(
    model: pin.Model,
    joint_names: Sequence[str],
    ranges: Mapping[str, tuple[float, float]],
    *,
    samples: int = DETECTION_SAMPLES,
    seed: int = DETECTION_SEED,
) -> np.ndarray:
    """Model gravity torque at ``samples`` uniform random poses, one row per pose."""
    data = model.createData()
    rng = np.random.default_rng(seed)
    low = np.asarray([ranges[name][0] for name in joint_names])
    high = np.asarray([ranges[name][1] for name in joint_names])
    rows = [
        np.asarray(pin.computeGeneralizedGravity(model, data, rng.uniform(low, high)))
        for _ in range(samples)
    ]
    return np.vstack(rows)


def default_gravity_links(
    joints: Sequence[JointDescription], joint_names: Sequence[str], measured: Sequence[str]
) -> list[str]:
    """Child links of the arm joints at or below a measured joint, in joint order.

    A link above every measured joint adds torque to none of them, so its mass
    cannot be identified.
    """
    by_child = {joint.child_link: joint for joint in joints}
    by_name = {joint.name: joint for joint in joints}
    measured_set = set(measured)
    links: list[str] = []
    for name in joint_names:
        joint: JointDescription | None = by_name[name]
        while joint is not None:
            if joint.name in measured_set:
                links.append(by_name[name].child_link)
                break
            joint = by_child.get(joint.parent_link)
    return links


def merged_links(
    joints: Sequence[JointDescription], joint_names: Sequence[str], link: str
) -> list[str]:
    """Links below ``link`` reached without crossing an arm joint, in tree order.

    Pinocchio adds their inertia to ``link``'s body: fixed joints are merged,
    and the calibration locks every joint that is not an arm joint.
    """
    children = _children(joints)
    moving = set(joint_names)
    found: list[str] = []
    stack = [link]
    while stack:
        parent = stack.pop(0)
        for joint in children.get(parent, []):
            if joint.name in moving:
                continue
            found.append(joint.child_link)
            stack.append(joint.child_link)
    return found


def default_tip_frame(joints: Sequence[JointDescription], joint_names: Sequence[str]) -> str:
    """A grasp or TCP frame rigidly below the last arm joint, else its fixed chain's end."""
    children = _children(joints)
    last = next(joint for joint in joints if joint.name == joint_names[-1]).child_link
    rigid = [last, *merged_links(joints, joint_names, last)]
    for hint in TIP_FRAME_HINTS:
        named = [link for link in rigid if hint in link.lower()]
        if named:
            return named[0]
    tip = last
    while True:
        fixed = [joint for joint in children.get(tip, []) if joint.type == "fixed"]
        if len(fixed) != 1:
            return tip
        tip = fixed[0].child_link


def default_clearance_frames(
    model: pin.Model,
    joints: Sequence[JointDescription],
    joint_names: Sequence[str],
    measured: Sequence[str],
    ranges: Mapping[str, tuple[float, float]],
    home: Mapping[str, float],
    *,
    samples: int = 64,
    seed: int = DETECTION_SEED,
) -> list[str]:
    """Links below the first arm joint whose origin moves when the measured joints move.

    Pose generation varies only the measured joints, holding the others at home;
    a frame that stays put then (one on the shoulder axis, say) is left out, as
    it can never come closer to the floor than it already is.
    """
    first = next(joint for joint in joints if joint.name == joint_names[0]).child_link
    children = _children(joints)
    below: list[str] = []
    stack = [first]
    while stack:
        link = stack.pop(0)
        below.append(link)
        stack.extend(joint.child_link for joint in children.get(link, []))
    frames = [link for link in below if model.existFrame(link, pin.FrameType.BODY)]
    ids = [frame_id(model, link) for link in frames]
    data = model.createData()
    home_q = np.asarray([home[name] for name in joint_names], dtype=np.float64)
    rng = np.random.default_rng(seed)
    columns = [joint_names.index(name) for name in measured]
    low = np.asarray([ranges[name][0] for name in measured])
    high = np.asarray([ranges[name][1] for name in measured])

    def positions(q: np.ndarray) -> np.ndarray:
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        return np.stack([data.oMf[index].translation for index in ids])

    reference = positions(home_q)
    travel = np.zeros(len(frames))
    for _ in range(samples):
        q = home_q.copy()
        q[columns] = rng.uniform(low, high)
        travel = np.maximum(travel, np.linalg.norm(positions(q) - reference, axis=1))
    return [
        frame
        for frame, moved in zip(frames, travel, strict=True)
        if moved >= MOBILE_FRAME_MIN_TRAVEL_M
    ]


def _children(joints: Sequence[JointDescription]) -> dict[str, list[JointDescription]]:
    children: dict[str, list[JointDescription]] = {}
    for joint in joints:
        children.setdefault(joint.parent_link, []).append(joint)
    return children


def _home_joints(
    names: Sequence[str],
    home: Mapping[str, float] | Sequence[float] | None,
    descriptions: Mapping[str, JointDescription],
) -> dict[str, float]:
    if home is None:
        result = {}
        for name in names:
            lower, upper = urdf_limits(descriptions[name])
            result[name] = float(min(max(0.0, lower), upper))
        return result
    if isinstance(home, Mapping):
        return {name: float(home[name]) for name in names}
    values = list(home)
    if len(values) != len(names):
        raise ValueError(f"home posture needs {len(names)} values, got {len(values)}")
    return {name: float(value) for name, value in zip(names, values, strict=True)}


def _urdf_effort(joint: JointDescription) -> float:
    if joint.effort is None or joint.effort <= 0.0:
        raise ValueError(f"{joint.name} has no URDF effort limit; pass max_torque_nm")
    return float(joint.effort)
