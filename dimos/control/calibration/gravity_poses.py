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

"""Poses for gravity-model identification of one arm, and the arm model the fit and the guards share.

Poses lie on a grid over the measured joints (the others at home) inside their ranges less a
margin, pass the workspace guard and torque caps, and are picked greedily to best condition
the stacked gravity regressor; each is reachable from the previous one by a joint-space ramp
that keeps the transition clearance. The regressor is pinocchio's joint torque regressor at
rest, of which the first four parameters per body ``(m, m*cx, m*cy, m*cz)`` carry gravity.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
import itertools
import math

import numpy as np
import pinocchio as pin

from dimos.control.calibration._inertial import body_joint_id
from dimos.control.calibration.spec import (
    ArmCalibrationSpec,
    default_clearance_frames,
    frame_id,
    reduced_model,
)

GRAVITY_PARAMS_PER_BODY = 4
PARAMS_PER_BODY = 10

Joints = Sequence[float] | np.ndarray


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
class PoseSet:
    identification: list[Pose]
    rank: int
    singular_values: tuple[float, ...]
    condition_number: float
    candidates: int
    notes: list[str] = field(default_factory=list)


def matrix_rank_tolerance(singular_values: np.ndarray, shape: tuple[int, ...]) -> float:
    """numpy.linalg.matrix_rank's tolerance: largest singular value * max(shape) * eps."""
    return float(singular_values.max(initial=0.0)) * max(shape) * float(np.finfo(np.float64).eps)


class ArmModel:
    """Pinocchio model of one arm (every other joint locked) with the spec's workspace checks.

    ``reach_m`` is the arm's stretched length (joint to joint to fingertip), which bounds how
    far any point of it moves per radian. The fit's columns are scaled by the fitted bodies'
    URDF mass ``M`` and ``M * reach_m``, so a correction is weighed relative to the arm itself.
    """

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
        self.clearance_frames = spec.workspace.clearance_frames
        if self.clearance_frames is None:
            self.clearance_frames = tuple(
                default_clearance_frames(
                    self.model,
                    list(spec.model.load().joints),
                    self.joint_names,
                    self.measured_joints,
                    spec.joint_ranges,
                    spec.home_joints,
                )
            )
        self._clearance = [frame_id(self.model, name) for name in self.clearance_frames]
        pin.framesForwardKinematics(self.model, self.data, self.home)
        self.origin = np.array(self.data.oMf[frame_id(self.model, spec.base_frame)].translation)
        self.reach_m = float(
            sum(np.linalg.norm(p.translation) for p in list(self.model.jointPlacements)[2:])
            + np.linalg.norm(self.model.frames[self._tip].placement.translation)
            + np.linalg.norm(self._tip_offset)
        )
        # How far above the floor every point stays at a pose and along ramps; the caller sets it.
        self.clearance_m = 0.0
        self.fitted_joint_ids = tuple(body_joint_id(self.model, name) for name in self.fitted_links)
        self.param_columns = tuple(
            (joint_id - 1) * PARAMS_PER_BODY + k
            for joint_id in self.fitted_joint_ids
            for k in range(GRAVITY_PARAMS_PER_BODY)
        )
        mass = sum(float(self.model.inertias[j].mass) for j in self.fitted_joint_ids)
        if mass <= 0.0 or self.reach_m <= 0.0:
            raise ValueError(f"{spec.arm_id}: the fitted links have no URDF mass or no length")
        self.column_scale = np.tile([mass, *[mass * self.reach_m] * 3], len(self.fitted_joint_ids))
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
        return self.points(self.q(joints))[0]

    def gravity(self, q: np.ndarray) -> np.ndarray:
        return np.asarray(pin.computeGeneralizedGravity(self.model, self.data, q))

    def scaled_regressor(self, q: np.ndarray, rows: Sequence[int] | None = None) -> np.ndarray:
        """Rows: the arm joints ``rows`` (default the measured joints); columns: ``(m, m*c)`` of
        the fitted bodies over ``column_scale``."""
        zero = np.zeros(self.model.nv)
        full = np.asarray(pin.computeJointTorqueRegressor(self.model, self.data, q, zero, zero))
        index = self.measured_rows if rows is None else tuple(rows)
        scaled: np.ndarray = full[np.ix_(index, self.param_columns)] * self.column_scale
        return scaled

    def _clear(self, tip: np.ndarray, others: np.ndarray) -> bool:
        """Clear of the keep-outs, and the fingertip in front when asked."""
        min_x = self.workspace.min_x_m
        if min_x is not None and tip[0] <= min_x:
            return False
        points = np.vstack([tip, others])
        return not any(space.violated_by(points) for space in self.workspace.keep_out)

    def _bound_distance(self, tip: np.ndarray, others: np.ndarray) -> float:
        """Smallest distance of a point to a keep-out, or of the fingertip to the front limit."""
        points = np.vstack([tip, others])
        distances = [float(space.distance_m(points).min()) for space in self.workspace.keep_out]
        if self.workspace.min_x_m is not None:
            distances.append(float(tip[0]) - self.workspace.min_x_m)
        return min(distances, default=math.inf)

    def pose_ok(self, joints: Joints) -> bool:
        """Clear of the floor and keep-outs, in front when asked, and no joint over its cap."""
        q = self.q(joints)
        tip, others = self.points(q)
        limit = self.workspace.floor_z_m + self.clearance_m
        if tip[2] <= limit or (others[:, 2] <= limit).any() or not self._clear(tip, others):
            return False
        return bool((np.abs(self.gravity(q)) <= self.max_torque).all())

    def transition_ok(self, a: Joints, b: Joints) -> bool:
        """Whether a joint-space ramp from ``a`` to ``b`` keeps the clearance.

        A point may run lower only where an end of the ramp has it, never below both ends. The
        margin is the smallest distance, at either end, of a point to the floor or a keep-out,
        or of the fingertip to the front limit; the ramp is checked often enough that no point
        moves more than twice the margin between checks (a point moves at most ``reach_m`` per
        radian).
        """
        start, end = self.q(a), self.q(b)
        ends = [self.points(q) for q in (start, end)]
        heights = [np.concatenate([[tip[2]], others[:, 2]]) for tip, others in ends]
        floor = np.minimum(self.workspace.floor_z_m + self.clearance_m, np.minimum(*heights))
        margin = min(
            float((floor - self.workspace.floor_z_m).min()),
            *(self._bound_distance(tip, others) for tip, others in ends),
        )
        if margin <= 0.0:
            return False
        checks = max(1, math.ceil(np.abs(end - start).sum() * self.reach_m / (2.0 * margin)))
        for s in np.linspace(0.0, 1.0, checks + 1):
            tip, others = self.points(start + s * (end - start))
            if (
                tip[2] < floor[0]
                or (others[:, 2] < floor[1:]).any()
                or not self._clear(tip, others)
            ):
                return False
        return True


def generate_poses(arm: ArmModel, *, count: int, margin_rad: float, grid_points: int) -> PoseSet:
    """``count`` identification poses, reachable in order from home.

    Args:
        count: How many poses to pick (an operator setting).
        margin_rad: How far every pose stays inside the joint ranges: the largest excursion
            the procedure makes around a pose (its dither and steps).
        grid_points: About how many grid poses to search, the caller's compute budget.

    Raises:
        NoWorkspaceGuardError: The spec has only the implicit workspace guard.
    """
    arm.spec.require_workspace_guard()
    values: list[np.ndarray] = []
    widths: dict[int, float] = {}
    for index, name in enumerate(arm.joint_names):
        low, high = arm.spec.joint_ranges[name]
        if name in arm.measured_joints and high - low > 2.0 * margin_rad:
            widths[index] = high - low - 2.0 * margin_rad
        values.append(
            np.asarray([0.5 * (low + high) if name in arm.measured_joints else arm.home[index]])
        )
    if widths:
        step = (math.prod(widths.values()) / grid_points) ** (1.0 / len(widths))
        for index, width in widths.items():
            low = arm.spec.joint_ranges[arm.joint_names[index]][0] + margin_rad
            values[index] = np.linspace(low, low + width, max(2, round(width / step) + 1))
    candidates = [c for c in itertools.product(*values) if arm.pose_ok(np.asarray(c))]
    if not candidates:
        raise ValueError(
            f"{arm.arm_id}: no feasible pose; check the joint ranges and the workspace"
        )
    stacked = np.vstack([arm.scaled_regressor(np.asarray(c)) for c in candidates])
    _, s, vt = np.linalg.svd(stacked, full_matrices=False)
    basis = vt[s > matrix_rank_tolerance(s, stacked.shape)].T
    projected = [arm.scaled_regressor(np.asarray(c)) @ basis for c in candidates]
    chosen: list[tuple[float, ...]] = [tuple(arm.home)]
    rows = arm.scaled_regressor(arm.home) @ basis
    while len(chosen) <= count and candidates:
        scores = []
        for extra in projected:
            sigma = np.linalg.svd(np.vstack([rows, extra]), compute_uv=False)
            full = rows.shape[0] + extra.shape[0] >= basis.shape[1]
            scores.append((float(sigma[-1]) if full else 0.0, float(sigma.sum())))
        order = sorted(range(len(candidates)), key=scores.__getitem__, reverse=True)
        pick = next((i for i in order if arm.transition_ok(chosen[-1], candidates[i])), None)
        if pick is None:
            break
        chosen.append(candidates.pop(pick))
        rows = np.vstack([rows, projected.pop(pick)])
    poses = [
        Pose(f"pose_{i:02d}", tuple(map(float, p)), arm.joint_names)
        for i, p in enumerate(chosen[1:], 1)
    ]
    notes = []
    if len(poses) < count:
        notes.append(f"only {len(poses)} of {count} poses are reachable in sequence")
    if poses and not arm.transition_ok(poses[-1].joints_rad, arm.home):
        notes.append("the last pose cannot ramp straight back home; return via the previous one")
    sigma = np.linalg.svd(rows, compute_uv=False)
    return PoseSet(
        identification=poses,
        rank=basis.shape[1],
        singular_values=tuple(float(v) for v in sigma),
        condition_number=float(sigma[0] / sigma[-1]) if sigma[-1] > 0 else math.inf,
        candidates=len(stacked) // len(arm.measured_rows),
        notes=notes,
    )
