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

"""Per-arm pose set for gravity-model identification of the Dual OpenYAM.

    python -m dimos.robot.manipulators.calibration.gravity_poses [--side right]

Poses are chosen on a joint grid inside conservative boxes, filtered so the
fingertip, finger frames, gripper, wrist and elbow stay above the table, the
fingertip stays in front of the base and on its own side of the rig midline,
and picked greedily so the stacked gravity regressor (see ``gravity_fit``) is
as well conditioned as possible on its identifiable directions. Each pose is
reachable from the previous one by a joint-space ramp that never dips below
``MIN_TRANSITION_Z_M``. Three extra "gravity-free" poses null the model
gravity torque on joints 2, 3 and 4 for gain sweeps without a gravity error.

Both arms are the identical i2rt arm mounted the same way up, so the same
joint angles serve either side; ``ArmModel(side=...)`` selects which chain
moves (the other stays where the module holds it) and which frames are checked.

The regressor is pinocchio's joint torque regressor at rest: ``tau = Y(q) @
phi`` with ten inertial parameters per body ``(m, m*cx, m*cy, m*cz, I...)``;
only the first four per body carry gravity, and only the moving arm's links
from joint 2 to the gripper are fitted.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
import itertools
import math
from typing import TypeAlias

import numpy as np
import pinocchio as pin
import typer

from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_HOME_JOINTS,
    DUAL_OPENYAM_SIDES as SIDES,
)
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_ARM_JOINTS,
    DUAL_OPENYAM_DOF_PER_ARM,
)
from dimos.robot.manipulators.dual_openyam.model import DUAL_OPENYAM_MODEL

GRAVITY_PARAMS_PER_BODY = 4
PARAMS_PER_BODY = 10


def check_side(side: str) -> str:
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}, got {side!r}")
    return side


def arm_joint_names(side: str) -> tuple[str, ...]:
    return tuple(f"{check_side(side)}_joint{i}" for i in range(1, DUAL_OPENYAM_DOF_PER_ARM + 1))


def fitted_links(side: str) -> tuple[str, ...]:
    """Bodies whose mass and first moment are fitted: joint 2 .. gripper."""
    return (*(f"{check_side(side)}_link{i}" for i in range(2, 6)), f"{side}_gripper")


def measured_joints(side: str) -> tuple[str, ...]:
    """Joints whose resting torque is measured; joints 1 and 6 carry no useful gravity signal."""
    return tuple(f"{check_side(side)}_joint{i}" for i in range(2, 6))


def gravity_free_joints(side: str) -> tuple[str, ...]:
    return tuple(f"{check_side(side)}_joint{i}" for i in (2, 3, 4))


# Left-arm names, the default for the single-arm callers.
FITTED_LINKS = fitted_links("left")
MEASURED_JOINTS = measured_joints("left")
GRAVITY_FREE_JOINTS = gravity_free_joints("left")

# Grasp frame plus the finger length along the gripper's -z gives the fingertip.
FINGERTIP_OFFSET_M = 0.046
# Frames that must also stay clear of the table (per side: f"{side}_{name}").
CLEARANCE_FRAME_SUFFIXES = ("tip_left", "tip_right", "gripper", "link5", "link4")
TABLE_Z_M = -0.03  # table top in the base frame (base sits ~30 mm above it)
MIN_POSE_Z_M = 0.12
MIN_POSE_X_M = 0.15
MIN_TRANSITION_Z_M = 0.08
TRANSITION_SAMPLES = 24
# The moving arm's fingertip may cross the rig midline (y=0, midway between the
# bases) by at most this much, so it never reaches into the other arm's space.
# Was RigGeometry().base_spacing_m (0.430 m default) in the END-310 branch; RigGeometry is not on main.
RIG_BASE_SPACING_M = 0.430
RIG_HALF_SPACING_M = RIG_BASE_SPACING_M / 2.0
MAX_MIDLINE_CROSSING_M = 0.05
# No identification pose may load a joint beyond this in the model (DM4340 continuous ~9 Nm).
MAX_MODEL_TORQUE_NM = 10.0

# Conservative joint boxes, degrees, well inside the URDF limits
# (j2 0..210, j3 0..180, j4 -97..90, j5 -90..90). Joints 1 and 6 stay at 0.
# Keyed by joint number; identical for both arms.
JOINT_BOXES_DEG: dict[int, tuple[float, float]] = {
    1: (0.0, 0.0),
    2: (25.0, 150.0),
    3: (25.0, 160.0),
    4: (-75.0, 75.0),
    5: (-75.0, 75.0),
    6: (0.0, 0.0),
}
GRID_STEP_DEG = {2: 12.5, 3: 13.5, 4: 25.0, 5: 37.5}
DEFAULT_POSE_COUNT = 15
GRAVITY_FREE_LIMIT_NM = 0.05
# Column scaling for the fit: a plausible mass error and first-moment error.
MASS_SCALE_KG = 0.2
MOMENT_SCALE_KGM = 0.02
RANK_TOLERANCE = 1e-6

Joints: TypeAlias = Sequence[float] | np.ndarray


def joint_number(joint: str) -> int:
    """``left_joint3`` -> 3."""
    return int(joint.rsplit("joint", 1)[1])


@dataclass(frozen=True)
class Pose:
    """One arm pose, radians per joint of ``side`` in joint order 1..6."""

    name: str
    joints_rad: tuple[float, ...]
    side: str = "left"

    @property
    def joints_deg(self) -> tuple[float, ...]:
        return tuple(math.degrees(q) for q in self.joints_rad)

    def as_targets(self) -> dict[str, float]:
        return dict(zip(arm_joint_names(self.side), self.joints_rad, strict=True))


@dataclass(frozen=True)
class GravityFreePose:
    """Pose nulling (or, best effort, minimising) the model gravity torque on ``joint``."""

    joint: str
    pose: Pose
    torque_nm: float

    @property
    def exact(self) -> bool:
        return abs(self.torque_nm) < GRAVITY_FREE_LIMIT_NM


@dataclass(frozen=True)
class PoseSet:
    identification: list[Pose]
    gravity_free: dict[str, GravityFreePose]
    rank: int
    singular_values: tuple[float, ...]
    condition_number: float
    candidates: int = 0
    notes: list[str] = field(default_factory=list)
    side: str = "left"


class ArmModel:
    """Pinocchio model of both arms with helpers for one moving arm (``side``)."""

    def __init__(self, model: pin.Model | None = None, *, side: str = "left") -> None:
        self.side = check_side(side)
        self.model = model or load_arm_model()
        self.data = self.model.createData()
        if tuple(str(n) for n in self.model.names[1:]) != tuple(DUAL_OPENYAM_ARM_JOINTS):
            raise ValueError("arm model joints do not match DUAL_OPENYAM_ARM_JOINTS")
        self.home = np.asarray(DUAL_OPENYAM_HOME_JOINTS, dtype=np.float64)
        self.joint_names = arm_joint_names(side)
        self.offset = DUAL_OPENYAM_ARM_JOINTS.index(self.joint_names[0])
        self.fitted_links = fitted_links(side)
        self.measured_joints = measured_joints(side)
        self._grasp = self.model.getFrameId(f"{side}_grasp_frame")
        self._clearance = [
            self.model.getFrameId(f"{side}_{suffix}") for suffix in CLEARANCE_FRAME_SUFFIXES
        ]
        base = self.model.frames[self.model.getFrameId(f"{side}_base")]
        self.base_y = float(base.placement.translation[1])
        self.toward_midline = -1.0 if side == "left" else 1.0  # sign of y pointing at y=0
        self.fitted_joint_ids = tuple(body_joint_id(self.model, name) for name in self.fitted_links)
        self.measured_rows = tuple(
            self.model.joints[self.model.getJointId(name)].idx_v for name in self.measured_joints
        )
        self.param_columns = tuple(
            (joint_id - 1) * PARAMS_PER_BODY + k
            for joint_id in self.fitted_joint_ids
            for k in range(GRAVITY_PARAMS_PER_BODY)
        )
        self.column_scale = np.asarray(
            [MASS_SCALE_KG, *([MOMENT_SCALE_KGM] * 3)] * len(self.fitted_joint_ids)
        )

    def full_q(self, side_rad: Joints) -> np.ndarray:
        """Both arms: the moving arm at ``side_rad``, the other at home."""
        q = self.home.copy()
        q[self.offset : self.offset + DUAL_OPENYAM_DOF_PER_ARM] = side_rad
        return q

    def points(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Fingertip position and the stacked clearance frame positions, base frame."""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        tip = self.data.oMf[self._grasp].act(np.array([0.0, 0.0, -FINGERTIP_OFFSET_M]))
        others = np.stack([self.data.oMf[f].translation for f in self._clearance])
        return tip, others

    def midline_ok(self, tip: np.ndarray) -> bool:
        """True while the fingertip stays on its own side of the rig midline (within the margin)."""
        toward = self.toward_midline * (float(tip[1]) - self.base_y)
        return toward <= RIG_HALF_SPACING_M + MAX_MIDLINE_CROSSING_M

    def gravity(self, q: np.ndarray) -> np.ndarray:
        return np.asarray(pin.computeGeneralizedGravity(self.model, self.data, q))

    def side_gravity(self, q: np.ndarray) -> np.ndarray:
        """Gravity torque of the moving arm's six joints."""
        return self.gravity(q)[self.offset : self.offset + DUAL_OPENYAM_DOF_PER_ARM]

    def regressor(self, q: np.ndarray) -> np.ndarray:
        """Rows: measured joints; columns: ``(m, m*c)`` of the fitted bodies, unscaled."""
        zero = np.zeros(self.model.nv)
        full = np.asarray(pin.computeJointTorqueRegressor(self.model, self.data, q, zero, zero))
        return full[np.ix_(self.measured_rows, self.param_columns)]

    def scaled_regressor(self, q: np.ndarray) -> np.ndarray:
        scaled: np.ndarray = self.regressor(q) * self.column_scale
        return scaled

    def pose_ok(self, side_rad: Joints, *, require_front: bool = True) -> bool:
        """Clear of the table and midline, (optionally) in front, and no joint overloaded."""
        q = self.full_q(side_rad)
        tip, others = self.points(q)
        if tip[2] <= MIN_POSE_Z_M or (others[:, 2] <= MIN_POSE_Z_M).any():
            return False
        if not self.midline_ok(tip):
            return False
        if require_front and tip[0] <= MIN_POSE_X_M:
            return False
        return bool(np.abs(self.gravity(q)).max() <= MAX_MODEL_TORQUE_NM)

    def transition_ok(self, a: Joints, b: Joints) -> bool:
        """True when a joint-space ramp from ``a`` to ``b`` stays above the floor and on its side."""
        start = np.asarray(a, dtype=np.float64)
        end = np.asarray(b, dtype=np.float64)
        for s in np.linspace(0.0, 1.0, TRANSITION_SAMPLES + 1):
            tip, others = self.points(self.full_q(start + s * (end - start)))
            if tip[2] <= MIN_TRANSITION_Z_M or (others[:, 2] <= MIN_TRANSITION_Z_M).any():
                return False
            if not self.midline_ok(tip):
                return False
        return True


def load_arm_model() -> pin.Model:
    """Build the 12-joint arm model the Damiao adapter uses for gravity compensation."""
    return pin.buildModelFromXML(DUAL_OPENYAM_MODEL.load().xml)


def body_joint_id(model: pin.Model, link: str) -> int:
    """Joint whose inertia carries ``link``; the link frame must coincide with the joint."""
    if not model.existFrame(link):
        raise ValueError(f"model has no frame {link!r}")
    frame = model.frames[model.getFrameId(link)]
    if frame.type != pin.FrameType.BODY or not frame.placement.isIdentity(1e-9):
        raise ValueError(f"{link!r} is not a body frame coincident with its joint")
    return int(frame.parentJoint)


def home_pose(side: str = "left") -> Pose:
    offset = DUAL_OPENYAM_ARM_JOINTS.index(arm_joint_names(side)[0])
    return Pose(
        "home", tuple(DUAL_OPENYAM_HOME_JOINTS[offset : offset + DUAL_OPENYAM_DOF_PER_ARM]), side
    )


def _grid_values(joint: str) -> np.ndarray:
    low, high = JOINT_BOXES_DEG[joint_number(joint)]
    if high == low:
        return np.asarray([low])
    count = round((high - low) / GRID_STEP_DEG[joint_number(joint)]) + 1
    return np.linspace(low, high, count)


def candidate_poses(arm: ArmModel, *, require_front: bool = True) -> list[tuple[float, ...]]:
    """Every grid pose inside the boxes that clears the table (and sits in front)."""
    grids = [_grid_values(j) for j in arm.joint_names]
    return [
        tuple(np.radians(combo))
        for combo in itertools.product(*grids)
        if arm.pose_ok(np.radians(combo), require_front=require_front)
    ]


def identifiable_subspace(arm: ArmModel, poses: Iterable[Sequence[float]]) -> np.ndarray:
    """Right singular vectors spanning the directions gravity can identify (scaled columns)."""
    stacked = np.vstack([arm.scaled_regressor(arm.full_q(p)) for p in poses])
    _, s, vt = np.linalg.svd(stacked, full_matrices=False)
    rank = int((s > RANK_TOLERANCE * s[0]).sum())
    basis: np.ndarray = vt[:rank].T
    return basis


def stacked_regressor(arm: ArmModel, poses: Sequence[Sequence[float]]) -> np.ndarray:
    return np.vstack([arm.scaled_regressor(arm.full_q(p)) for p in poses])


def conditioning(arm: ArmModel, poses: Sequence[Sequence[float]], basis: np.ndarray) -> np.ndarray:
    """Singular values of the stacked scaled regressor projected on the identifiable basis."""
    return np.linalg.svd(stacked_regressor(arm, poses) @ basis, compute_uv=False)


def select_poses(
    arm: ArmModel,
    candidates: Sequence[tuple[float, ...]],
    basis: np.ndarray,
    *,
    count: int,
    start: Sequence[float],
) -> list[tuple[float, ...]]:
    """Greedy: add the transition-feasible candidate that most raises the smallest singular value."""
    chosen: list[tuple[float, ...]] = [tuple(start)]
    remaining = list(candidates)
    while len(chosen) < count + 1 and remaining:
        base = stacked_regressor(arm, chosen) @ basis
        best_index = -1
        best_score = -1.0
        for index, candidate in enumerate(remaining):
            trial = np.vstack([base, arm.scaled_regressor(arm.full_q(candidate)) @ basis])
            s = np.linalg.svd(trial, compute_uv=False)
            # Maximise the smallest singular value, break ties on the sum (volume).
            score = s[-1] if trial.shape[0] >= trial.shape[1] else 0.0
            score += 1e-3 * s.sum()
            if score > best_score:
                best_score = score
                best_index = index
        feasible = False
        # Try the ranked candidates until one is reachable from the last pose.
        order = [best_index] + [i for i in range(len(remaining)) if i != best_index]
        for index in order:
            if arm.transition_ok(chosen[-1], remaining[index]):
                chosen.append(remaining.pop(index))
                feasible = True
                break
        if not feasible:
            break
    return chosen[1:]


def _refine_root(
    arm: ArmModel, pose: np.ndarray, joint: int, row: int, lo: float, hi: float
) -> np.ndarray | None:
    """Bisect joint ``joint`` in [lo, hi] for a zero of the gravity torque on ``row``."""

    def torque(value: float) -> float:
        trial = pose.copy()
        trial[joint] = value
        return float(arm.gravity(arm.full_q(trial))[row])

    f_lo, f_hi = torque(lo), torque(hi)
    if f_lo * f_hi > 0.0:
        return None
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        f_mid = torque(mid)
        if f_lo * f_mid <= 0.0:
            hi, f_hi = mid, f_mid
        else:
            lo, f_lo = mid, f_mid
    result = pose.copy()
    result[joint] = 0.5 * (lo + hi)
    return result


def gravity_free_pose(
    arm: ArmModel,
    joint: str,
    candidates: Sequence[tuple[float, ...]],
    *,
    require_front: bool = True,
) -> GravityFreePose:
    """Feasible pose where the model gravity torque on ``joint`` is (numerically) zero.

    Starts from the grid candidates with the smallest torque magnitude and
    bisects along that joint inside its box. When no feasible zero exists the
    lowest-torque feasible candidate is returned instead (``exact`` is False).
    """
    index = arm.joint_names.index(joint)
    row = arm.model.joints[arm.model.getJointId(joint)].idx_v
    ranked = sorted(candidates, key=lambda p: abs(arm.gravity(arm.full_q(p))[row]))
    low, high = (math.radians(v) for v in JOINT_BOXES_DEG[joint_number(joint)])
    name = f"gravity_free_{joint}"
    for candidate in ranked[:40]:
        pose = np.asarray(candidate)
        step = math.radians(GRID_STEP_DEG[joint_number(joint)])
        lo = max(low, pose[index] - step)
        hi = min(high, pose[index] + step)
        refined = _refine_root(arm, pose, index, row, lo, hi)
        if refined is None or not arm.pose_ok(refined, require_front=require_front):
            continue
        torque = float(arm.gravity(arm.full_q(refined))[row])
        if abs(torque) < GRAVITY_FREE_LIMIT_NM:
            return GravityFreePose(
                joint, Pose(name, tuple(float(v) for v in refined), arm.side), torque
            )
    best = ranked[0]
    torque = float(arm.gravity(arm.full_q(best))[row])
    return GravityFreePose(joint, Pose(f"{name}_best_effort", tuple(best), arm.side), torque)


def generate_poses(arm: ArmModel | None = None, *, count: int = DEFAULT_POSE_COUNT) -> PoseSet:
    """Identification poses (reachable in order from home) plus the gravity-free poses."""
    arm = arm or ArmModel()
    home = home_pose(arm.side)
    candidates = candidate_poses(arm)
    if not candidates:
        raise RuntimeError("no feasible candidate poses; check the joint boxes and clearances")
    basis = identifiable_subspace(arm, candidates)
    selected = select_poses(arm, candidates, basis, count=count, start=home.joints_rad)
    poses = [Pose(f"pose_{i:02d}", p, arm.side) for i, p in enumerate(selected, 1)]
    notes: list[str] = []
    if len(poses) < count:
        notes.append(f"only {len(poses)} of {count} poses are reachable in sequence")
    if poses and not arm.transition_ok(poses[-1].joints_rad, home.joints_rad):
        notes.append("the last pose cannot ramp straight back home; return via the previous one")
    sigma = conditioning(arm, [home.joints_rad, *(p.joints_rad for p in poses)], basis)
    gravity_free = {}
    for joint in gravity_free_joints(arm.side):
        found = gravity_free_pose(arm, joint, candidates)
        gravity_free[joint] = found
        if not found.exact:
            # A zero usually exists only with the forearm vertical over the elbow,
            # which reaches back over the base; report it but keep the in-front pose.
            behind = gravity_free_pose(
                arm, joint, candidate_poses(arm, require_front=False), require_front=False
            )
            hint = ""
            if behind.exact:
                tip, _ = arm.points(arm.full_q(behind.pose.joints_rad))
                angles = " ".join(f"{v:.1f}" for v in behind.pose.joints_deg)
                hint = (
                    f"; an exact zero exists only behind the in-front line at [{angles}] deg "
                    f"(fingertip x={tip[0]:+.3f} m, z={tip[2]:+.3f} m)"
                )
            notes.append(
                f"{joint}: no pose inside the boxes nulls its gravity torque; best effort "
                f"{found.torque_nm:+.2f} Nm at {found.pose.name}{hint}"
            )
    return PoseSet(
        identification=poses,
        gravity_free=gravity_free,
        rank=basis.shape[1],
        singular_values=tuple(float(v) for v in sigma),
        condition_number=float(sigma[0] / sigma[-1]) if sigma[-1] > 0 else math.inf,
        candidates=len(candidates),
        notes=notes,
        side=arm.side,
    )


def format_pose(arm: ArmModel, pose: Pose) -> str:
    q = arm.full_q(pose.joints_rad)
    tip, _ = arm.points(q)
    g = arm.side_gravity(q)
    joints = " ".join(f"{v:7.1f}" for v in pose.joints_deg)
    torques = " ".join(f"{v:6.2f}" for v in g[1:5])
    return (
        f"{pose.name:24s} [{joints}]  tip x={tip[0]:+.3f} y={tip[1]:+.3f} z={tip[2]:+.3f} m  "
        f"g2..5=[{torques}] Nm"
    )


def format_pose_set(arm: ArmModel, pose_set: PoseSet) -> str:
    lines = [
        f"{arm.side} arm: {len(pose_set.identification)} identification poses from "
        f"{pose_set.candidates} candidates; joints deg in order {', '.join(arm.joint_names)}",
        format_pose(arm, home_pose(arm.side)),
        *(format_pose(arm, p) for p in pose_set.identification),
        f"identifiable rank {pose_set.rank} of "
        f"{len(arm.fitted_links) * GRAVITY_PARAMS_PER_BODY} parameters; condition number "
        f"{pose_set.condition_number:.1f}; singular values "
        + " ".join(f"{s:.3g}" for s in pose_set.singular_values),
        "gravity-free poses (model torque on the named joint; best effort when not ~0):",
        *(
            f"{format_pose(arm, g.pose)}  -> {g.joint} {g.torque_nm:+.3f} Nm"
            + ("" if g.exact else " (best effort)")
            for g in pose_set.gravity_free.values()
        ),
        *(f"note: {n}" for n in pose_set.notes),
    ]
    return "\n".join(lines)


app = typer.Typer(add_completion=False)


@app.command()
def main(side: str = typer.Option("left", help="Which arm moves: left or right")) -> None:
    """Print the identification and gravity-free poses for one arm, in degrees."""
    arm = ArmModel(side=side)
    typer.echo(format_pose_set(arm, generate_poses(arm)))


if __name__ == "__main__":
    app()
