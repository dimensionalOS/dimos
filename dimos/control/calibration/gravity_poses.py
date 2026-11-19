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

"""Pose set for gravity-model identification of one arm, driven by its spec.

    python -m dimos.control.calibration.gravity_poses --robot dual-openyam --arm left

Poses are chosen on a grid over the spec's measured joints inside their
``joint_ranges`` (every other joint stays at home), filtered by the spec's
``WorkspaceGuard`` (fingertip and clearance frames above the floor, in front
of the base when asked, outside every keep-out half-space) and the per-joint
torque caps, and picked greedily so the stacked gravity regressor (see
``gravity_fit``) is as well conditioned as possible on its identifiable
directions. Each pose is reachable from the previous one by a joint-space
ramp that keeps the transition clearance. Extra "gravity-free" poses null the
model gravity torque on each measured joint for gain sweeps without a gravity
error.

The grid has about ``GRID_CANDIDATES_PER_POSE`` points per requested pose; a
wider range gets more grid values, and a joint carrying more gravity a finer
step (see ``grid_axes``), unless the spec sets the steps (``grid_step_rad``).

Poses are generated only for a spec with an explicit workspace guard: the
implicit one knows nothing of the arm's surroundings.

The regressor is pinocchio's joint torque regressor at rest: ``tau = Y(q) @
phi`` with ten inertial parameters per body ``(m, m*cx, m*cy, m*cz, I...)``;
only the first four per body carry gravity, and only the spec's
``gravity_links`` are fitted.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
import itertools
import math
from typing import TypeAlias

import numpy as np
import pinocchio as pin
import typer

from dimos.control.calibration.robots import arm_spec
from dimos.control.calibration.spec import (
    ArmCalibrationSpec,
    NoWorkspaceGuardError,
    default_clearance_frames,
    frame_id,
    gravity_samples,
    reduced_model,
)

GRAVITY_PARAMS_PER_BODY = 4
PARAMS_PER_BODY = 10
DEFAULT_POSE_COUNT = 15
GRID_CANDIDATES_PER_POSE = 280
MAX_GRID_CANDIDATES = 20_000
TRANSITION_SAMPLES = 24
GRAVITY_FREE_LIMIT_NM = 0.05
GRAVITY_FREE_TRIES = 40
# Column scaling for the fit: a plausible mass error and first-moment error.
MASS_SCALE_KG = 0.2
MOMENT_SCALE_KGM = 0.02
RANK_TOLERANCE = 1e-6

Joints: TypeAlias = Sequence[float] | np.ndarray


@dataclass(frozen=True)
class Pose:
    """One arm pose, radians per arm joint in ``joint_names`` order."""

    name: str
    joints_rad: tuple[float, ...]
    joint_names: tuple[str, ...]

    @property
    def joints_deg(self) -> tuple[float, ...]:
        return tuple(math.degrees(q) for q in self.joints_rad)

    def as_targets(self) -> dict[str, float]:
        return dict(zip(self.joint_names, self.joints_rad, strict=True))


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
    arm_id: str = ""
    grid_step_deg: dict[str, float] = field(default_factory=dict)


class ArmModel:
    """Pinocchio model of one arm (every other joint locked) with the spec's checks."""

    def __init__(self, spec: ArmCalibrationSpec, model: pin.Model | None = None) -> None:
        self.spec = spec
        self.model = model if model is not None else reduced_model(spec.model, spec.joint_names)
        self.data = self.model.createData()
        self.joint_names = tuple(spec.joint_names)
        if tuple(str(n) for n in self.model.names[1:]) != self.joint_names:
            raise ValueError(f"{spec.arm_id}: model joints do not match the spec joints")
        self.dof = len(self.joint_names)
        self.home = spec.home_vector
        self.fitted_links = tuple(spec.gravity_links)
        self.measured_joints = tuple(spec.measured_joints)
        self.measured_rows = tuple(self.joint_names.index(name) for name in self.measured_joints)
        self.workspace = spec.workspace
        self._tip = frame_id(self.model, spec.tip_frame)
        self._tip_offset = np.asarray(spec.workspace.point_offset_m, dtype=np.float64)
        clearance = spec.workspace.clearance_frames
        if clearance is None:
            descriptions = list(spec.model.load().joints)
            clearance = tuple(
                default_clearance_frames(
                    self.model,
                    descriptions,
                    self.joint_names,
                    self.measured_joints,
                    spec.joint_ranges,
                    spec.home_joints,
                )
            )
        self.clearance_frames = tuple(clearance)
        self._clearance = [frame_id(self.model, name) for name in self.clearance_frames]
        pin.framesForwardKinematics(self.model, self.data, self.home)
        self.origin = np.array(
            self.data.oMf[frame_id(self.model, spec.base_frame)].translation, dtype=np.float64
        )
        self.fitted_joint_ids = tuple(body_joint_id(self.model, name) for name in self.fitted_links)
        self.param_columns = tuple(
            (joint_id - 1) * PARAMS_PER_BODY + k
            for joint_id in self.fitted_joint_ids
            for k in range(GRAVITY_PARAMS_PER_BODY)
        )
        self.column_scale = np.asarray(
            [MASS_SCALE_KG, *([MOMENT_SCALE_KGM] * 3)] * len(self.fitted_joint_ids)
        )
        self.max_torque = np.asarray([spec.max_torque_nm[name] for name in self.joint_names])

    @property
    def arm_id(self) -> str:
        return self.spec.arm_id

    def q(self, joints: Joints | Mapping[str, float]) -> np.ndarray:
        """Model configuration from arm joint values, in ``joint_names`` order or by name."""
        if isinstance(joints, Mapping):
            return np.asarray([joints[name] for name in self.joint_names], dtype=np.float64)
        q = np.asarray(joints, dtype=np.float64)
        if q.shape != (self.dof,):
            raise ValueError(f"{self.arm_id}: expected {self.dof} joint values, got {q.shape}")
        return q

    def points(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Fingertip and the stacked clearance frame origins, workspace frame."""
        pin.framesForwardKinematics(self.model, self.data, q)
        tip = self.data.oMf[self._tip].act(self._tip_offset) - self.origin
        frames = [self.data.oMf[f].translation for f in self._clearance]
        others = (np.stack(frames) if frames else np.zeros((0, 3))) - self.origin
        return np.asarray(tip, dtype=np.float64), others

    def fingertip(self, joints: Joints) -> np.ndarray:
        tip, _ = self.points(self.q(joints))
        return tip

    def gravity(self, q: np.ndarray) -> np.ndarray:
        """Model gravity torque of every arm joint, ``joint_names`` order."""
        return np.asarray(pin.computeGeneralizedGravity(self.model, self.data, q))

    def regressor(self, q: np.ndarray) -> np.ndarray:
        """Rows: measured joints; columns: ``(m, m*c)`` of the fitted bodies, unscaled."""
        zero = np.zeros(self.model.nv)
        full = np.asarray(pin.computeJointTorqueRegressor(self.model, self.data, q, zero, zero))
        return full[np.ix_(self.measured_rows, self.param_columns)]

    def scaled_regressor(self, q: np.ndarray) -> np.ndarray:
        scaled: np.ndarray = self.regressor(q) * self.column_scale
        return scaled

    def keep_out_ok(self, points: np.ndarray) -> bool:
        return not any(space.violated_by(points) for space in self.workspace.keep_out)

    def pose_ok(self, joints: Joints, *, require_front: bool = True) -> bool:
        """Clear of the floor and keep-outs, (optionally) in front, and no joint overloaded."""
        q = self.q(joints)
        tip, others = self.points(q)
        limit = self.workspace.min_pose_z_m
        if tip[2] <= limit or (others[:, 2] <= limit).any():
            return False
        if not self.keep_out_ok(np.vstack([tip, others])):
            return False
        min_x = self.workspace.min_x_m
        if require_front and min_x is not None and tip[0] <= min_x:
            return False
        return bool((np.abs(self.gravity(q)) <= self.max_torque).all())

    def transition_ok(self, a: Joints, b: Joints) -> bool:
        """True when a joint-space ramp from ``a`` to ``b`` keeps the transition clearance.

        A point may run lower only where an end of the ramp already has it (an arm
        resting low at home has to start there), never below both ends.
        """
        start = self.q(a)
        end = self.q(b)
        heights = [
            np.concatenate([[tip[2]], others[:, 2]])
            for tip, others in map(self.points, (start, end))
        ]
        floor = np.minimum(self.workspace.min_transition_z_m, np.minimum(*heights) - 1e-9)
        for s in np.linspace(0.0, 1.0, TRANSITION_SAMPLES + 1):
            tip, others = self.points(start + s * (end - start))
            if tip[2] <= floor[0] or (others[:, 2] <= floor[1:]).any():
                return False
            if not self.keep_out_ok(np.vstack([tip, others])):
                return False
        return True


def body_joint_id(model: pin.Model, link: str) -> int:
    """Joint whose inertia carries ``link``; the link frame must coincide with the joint."""
    if not model.existFrame(link):
        raise ValueError(f"model has no frame {link!r}")
    frame = model.frames[frame_id(model, link)]
    if frame.type != pin.FrameType.BODY or not frame.placement.isIdentity(1e-9):
        raise ValueError(f"{link!r} is not a body frame coincident with its joint")
    return int(frame.parentJoint)


def home_pose(arm: ArmModel) -> Pose:
    return Pose("home", tuple(float(v) for v in arm.home), arm.joint_names)


def grid_axes(
    arm: ArmModel, count: int = DEFAULT_POSE_COUNT
) -> tuple[list[np.ndarray], dict[str, float]]:
    """Grid values per arm joint (rad) and the grid step per measured joint (deg).

    Measured joints span their ``joint_ranges``; every other joint stays at home.
    A joint in the spec's ``grid_step_rad`` gets that step (rounded so the grid
    ends on both ends of its range). Any other joint's step is a shared base
    step times ``sqrt(largest span / its span)`` of the model gravity torque over
    random poses, so joints that carry more gravity get a finer grid, and the
    base step is chosen so the grid has about ``GRID_CANDIDATES_PER_POSE *
    count`` points.
    """
    budget = min(MAX_GRID_CANDIDATES, GRID_CANDIDATES_PER_POSE * max(count, 1))
    ranges = arm.spec.joint_ranges
    given = arm.spec.grid_step_rad or {}
    torques = gravity_samples(arm.model, arm.joint_names, ranges)
    torque_spans = dict(
        zip(arm.joint_names, torques.max(axis=0) - torques.min(axis=0), strict=True)
    )
    gridded = [name for name in arm.measured_joints if ranges[name][1] > ranges[name][0]]
    widest = max((torque_spans[name] for name in gridded), default=1.0)
    weight = {
        name: math.sqrt(widest / max(float(torque_spans[name]), 1e-9 * widest)) for name in gridded
    }
    span_deg = {name: math.degrees(ranges[name][1] - ranges[name][0]) for name in gridded}
    fixed = {
        name: max(2, round(span_deg[name] / math.degrees(given[name])) + 1)
        for name in gridded
        if name in given
    }

    def points(step: float) -> dict[str, int]:
        return {
            name: fixed.get(name, max(2, round(span_deg[name] / (step * weight[name])) + 1))
            for name in gridded
        }

    low_step, high_step = 1e-3, 360.0
    for _ in range(60):
        step = math.sqrt(low_step * high_step)
        if math.prod(points(step).values()) > budget:
            low_step = step
        else:
            high_step = step
    counts = points(high_step)
    axes = []
    for name, value in zip(arm.joint_names, arm.home, strict=True):
        if name in counts:
            low, high = ranges[name]
            axes.append(np.linspace(low, high, counts[name]))
        else:
            axes.append(np.asarray([value]))
    steps = {name: span_deg[name] / (counts[name] - 1) for name in gridded}
    return axes, steps


def candidate_poses(
    arm: ArmModel, axes: Sequence[np.ndarray], *, require_front: bool = True
) -> list[tuple[float, ...]]:
    """Every grid pose that passes ``pose_ok``."""
    return [
        tuple(float(v) for v in combo)
        for combo in itertools.product(*axes)
        if arm.pose_ok(np.asarray(combo), require_front=require_front)
    ]


def identifiable_subspace(arm: ArmModel, poses: Iterable[Sequence[float]]) -> np.ndarray:
    """Right singular vectors spanning the directions gravity can identify (scaled columns)."""
    stacked = np.vstack([arm.scaled_regressor(arm.q(p)) for p in poses])
    _, s, vt = np.linalg.svd(stacked, full_matrices=False)
    rank = int((s > RANK_TOLERANCE * s[0]).sum())
    basis: np.ndarray = vt[:rank].T
    return basis


def stacked_regressor(arm: ArmModel, poses: Sequence[Sequence[float]]) -> np.ndarray:
    return np.vstack([arm.scaled_regressor(arm.q(p)) for p in poses])


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
    projected = [arm.scaled_regressor(arm.q(c)) @ basis for c in remaining]
    while len(chosen) < count + 1 and remaining:
        base = stacked_regressor(arm, chosen) @ basis
        best_index = -1
        best_score = -1.0
        for index, rows in enumerate(projected):
            trial = np.vstack([base, rows])
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
                projected.pop(index)
                feasible = True
                break
        if not feasible:
            break
    return chosen[1:]


def _refine_root(
    arm: ArmModel, pose: np.ndarray, joint: int, lo: float, hi: float
) -> np.ndarray | None:
    """Bisect joint ``joint`` in [lo, hi] for a zero of its own gravity torque."""

    def torque(value: float) -> float:
        trial = pose.copy()
        trial[joint] = value
        return float(arm.gravity(trial)[joint])

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
    step_rad: float,
    require_front: bool = True,
) -> GravityFreePose:
    """Feasible pose where the model gravity torque on ``joint`` is (numerically) zero.

    Starts from the grid candidates with the smallest torque magnitude and
    bisects along that joint, one grid step either way, inside its range. When
    no feasible zero exists the lowest-torque feasible candidate is returned
    instead (``exact`` is False).
    """
    index = arm.joint_names.index(joint)
    ranked = sorted(candidates, key=lambda p: abs(arm.gravity(arm.q(p))[index]))
    low, high = arm.spec.joint_ranges[joint]
    name = f"gravity_free_{joint}"
    for candidate in ranked[:GRAVITY_FREE_TRIES]:
        pose = np.asarray(candidate)
        lo = max(low, pose[index] - step_rad)
        hi = min(high, pose[index] + step_rad)
        refined = _refine_root(arm, pose, index, lo, hi)
        if refined is None or not arm.pose_ok(refined, require_front=require_front):
            continue
        torque = float(arm.gravity(refined)[index])
        if abs(torque) < GRAVITY_FREE_LIMIT_NM:
            return GravityFreePose(
                joint, Pose(name, tuple(float(v) for v in refined), arm.joint_names), torque
            )
    best = ranked[0]
    torque = float(arm.gravity(arm.q(best))[index])
    return GravityFreePose(joint, Pose(f"{name}_best_effort", tuple(best), arm.joint_names), torque)


def generate_poses(
    spec: ArmCalibrationSpec | ArmModel,
    *,
    count: int = DEFAULT_POSE_COUNT,
    gravity_free_joints: Sequence[str] | None = None,
) -> PoseSet:
    """Identification poses (reachable in order from home) plus the gravity-free poses.

    Args:
        spec: The arm, or an ``ArmModel`` already built for it.
        count: How many identification poses to pick.
        gravity_free_joints: Joints to find a gravity-free pose for; defaults
            to every measured joint.

    Raises:
        NoWorkspaceGuardError: The spec has only the implicit workspace guard.
    """
    arm = spec if isinstance(spec, ArmModel) else ArmModel(spec)
    arm.spec.require_workspace_guard()
    home = home_pose(arm)
    axes, steps_deg = grid_axes(arm, count)
    candidates = candidate_poses(arm, axes)
    if not candidates:
        raise RuntimeError(
            f"{arm.arm_id}: no feasible candidate poses; check the joint ranges and the workspace"
        )
    basis = identifiable_subspace(arm, candidates)
    selected = select_poses(arm, candidates, basis, count=count, start=home.joints_rad)
    poses = [Pose(f"pose_{i:02d}", p, arm.joint_names) for i, p in enumerate(selected, 1)]
    notes: list[str] = []
    if len(poses) < count:
        notes.append(f"only {len(poses)} of {count} poses are reachable in sequence")
    if poses and not arm.transition_ok(poses[-1].joints_rad, home.joints_rad):
        notes.append("the last pose cannot ramp straight back home; return via the previous one")
    sigma = conditioning(arm, [home.joints_rad, *(p.joints_rad for p in poses)], basis)
    gravity_free: dict[str, GravityFreePose] = {}
    behind_candidates: list[tuple[float, ...]] | None = None
    for joint in arm.measured_joints if gravity_free_joints is None else gravity_free_joints:
        step_rad = math.radians(steps_deg.get(joint, 0.0))
        found = gravity_free_pose(arm, joint, candidates, step_rad=step_rad)
        gravity_free[joint] = found
        if found.exact:
            continue
        hint = ""
        if arm.workspace.min_x_m is not None:
            # A zero often exists only with the forearm vertical over the elbow,
            # which reaches back over the base; report it but keep the in-front pose.
            if behind_candidates is None:
                behind_candidates = candidate_poses(arm, axes, require_front=False)
            behind = gravity_free_pose(
                arm, joint, behind_candidates, step_rad=step_rad, require_front=False
            )
            if behind.exact:
                tip = arm.fingertip(behind.pose.joints_rad)
                angles = " ".join(f"{v:.1f}" for v in behind.pose.joints_deg)
                hint = (
                    f"; an exact zero exists only behind the in-front line at [{angles}] deg "
                    f"(fingertip x={tip[0]:+.3f} m, z={tip[2]:+.3f} m)"
                )
        notes.append(
            f"{joint}: no pose inside the ranges nulls its gravity torque; best effort "
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
        arm_id=arm.arm_id,
        grid_step_deg=steps_deg,
    )


def format_pose(arm: ArmModel, pose: Pose) -> str:
    q = arm.q(pose.joints_rad)
    tip, _ = arm.points(q)
    g = arm.gravity(q)
    joints = " ".join(f"{v:7.1f}" for v in pose.joints_deg)
    torques = " ".join(f"{g[row]:6.2f}" for row in arm.measured_rows)
    return (
        f"{pose.name:24s} [{joints}]  tip x={tip[0]:+.3f} y={tip[1]:+.3f} z={tip[2]:+.3f} m  "
        f"g=[{torques}] Nm"
    )


def format_pose_set(arm: ArmModel, pose_set: PoseSet) -> str:
    steps = ", ".join(f"{name} {step:.1f}" for name, step in pose_set.grid_step_deg.items())
    lines = [
        f"{arm.arm_id}: {len(pose_set.identification)} identification poses from "
        f"{pose_set.candidates} candidates (grid steps {steps} deg); "
        f"joints deg in order {', '.join(arm.joint_names)}; "
        f"g = model gravity torque on {', '.join(arm.measured_joints)}",
        format_pose(arm, home_pose(arm)),
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
def main(
    robot: str = typer.Option(..., help="A known robot, e.g. dual-openyam or b601-dm"),
    arm: str | None = typer.Option(None, help="Which arm, e.g. left; omit for a single arm"),
    count: int = typer.Option(DEFAULT_POSE_COUNT, help="Identification poses to pick"),
    floor_z_m: float | None = typer.Option(
        None, help="Table height from the arm's base, m, for a robot without a bench guard"
    ),
    min_x_m: float | None = typer.Option(
        None, help="The fingertip stays in front of this x from the base, m, with --floor-z-m"
    ),
) -> None:
    """Print the identification and gravity-free poses for one arm, in degrees."""
    try:
        spec = arm_spec(robot, arm)
    except ValueError as error:
        raise typer.BadParameter(str(error)) from None
    if (floor_z_m is None) != (min_x_m is None):
        raise typer.BadParameter("give both --floor-z-m and --min-x-m, or neither")
    if floor_z_m is not None and min_x_m is not None:
        spec = spec.on_bench(floor_z_m, min_x_m)
    model = ArmModel(spec)
    try:
        pose_set = generate_poses(model, count=count)
    except NoWorkspaceGuardError as error:
        raise typer.BadParameter(f"{error} (--floor-z-m and --min-x-m)") from None
    typer.echo(format_pose_set(model, pose_set))


if __name__ == "__main__":
    app()
