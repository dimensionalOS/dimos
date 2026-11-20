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

"""Gravity-model identification of one arm from dithers at rest.

At each pose every measured joint is dithered to +d and -d. At rest the motor torque is the
compensated gravity plus ``kp (target - q)`` plus friction, so the PD torque averaged over the
pair is the error of the compensated model (Coulomb friction flips sign with the approach and
cancels) and half the difference is the friction. A pair the joint followed less than a share of
the way is stuck: friction held it, so the pair cancels less than its friction; it stays in the
fit, flagged. Adding what was compensated beyond the URDF
gives the URDF's error, fitted with pinocchio's torque regressor as ridge least squares for
corrections of ``(m, m*c)`` of the fitted links, inside the directions the poses identify and
inside physical bounds (mass >= 0, COM within the arm's reach of its joint). The ridge is
chosen by leave-one-pose-out cross-validation, and the fit is scored on every pose with that
pose left out (the selection of the ridge nested inside), so a fit is offered only when its
held-out error is no worse than the URDF's and the running model's on every measured joint.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np
import pinocchio as pin
from scipy.optimize import minimize

from dimos.control.calibration._inertial import LinkInertial, apply_inertial_overrides
from dimos.control.calibration.gravity_poses import (
    GRAVITY_PARAMS_PER_BODY,
    ArmModel,
    matrix_rank_tolerance,
)
from dimos.control.calibration.step_analysis import RestNoise, StepEvent, rest_window


@dataclass(frozen=True)
class Compensation:
    """The gravity the connection added while the poses were held: none, or the arm's model
    with ``links`` in place of the URDF's (none given: the URDF itself)."""

    gravity: bool
    links: dict[str, LinkInertial] = field(default_factory=dict)


@dataclass(frozen=True)
class DitherHold:
    """One dither: its step event and every arm joint's samples (rows, radians) during the hold."""

    event: StepEvent
    t: np.ndarray
    q_rad: np.ndarray


@dataclass(frozen=True)
class Measurement:
    """Resting PD torque of one joint at one pose, from a +d/-d dither pair (``stuck``: see
    ``measure_dither``)."""

    pose: str
    joint: str
    kp: float
    dither_deg: float
    q_rad: dict[str, float]
    tau_pd_nm: float
    friction_nm: float
    stuck: bool = False


@dataclass(frozen=True)
class GravityFit:
    """A fit and its errors, one value per measurement (``rms`` sums them up by joint)."""

    measurements: list[Measurement]
    joints: tuple[str, ...]
    urdf_error: np.ndarray
    running_error: np.ndarray
    held_out_error: np.ndarray
    fit_error: np.ndarray
    links: dict[str, LinkInertial]
    urdf_links: dict[str, LinkInertial]
    ridge: float
    rank: int
    singular_values: tuple[float, ...]

    def rms(self, values: np.ndarray) -> dict[str, float]:
        names = np.asarray([m.joint for m in self.measurements])
        return {
            joint: float(np.sqrt(np.mean(np.square(values[names == joint]))))
            for joint in self.joints
            if (names == joint).any()
        }


def measure_dither(
    pose: str,
    joint_names: Sequence[str],
    plus: DitherHold,
    minus: DitherHold,
    noise: RestNoise,
    *,
    stuck_fraction: float,
) -> Measurement:
    """The measurement of one dither pair; ``noise`` is the dithered joint's at rest.

    Each hold's rest is its last ``noise.window_s``. The pair is stuck when the joint moved
    between the two rests less than ``stuck_fraction`` of the way between the two targets
    (stiction held it, so the pair does not cancel its friction). Raises ``ValueError`` for a
    mismatched pair or a hold whose samples stop short.
    """
    a, b = plus.event, minus.event
    if (
        a.joint != b.joint
        or a.kp != b.kp
        or not a.delta_deg > 0.0 > b.delta_deg
        or a.delta_deg != -b.delta_deg
        or not math.isclose(a.target_after_deg - a.delta_deg, b.target_after_deg - b.delta_deg)
    ):
        raise ValueError(f"{pose}: {a} and {b} are not a +d/-d dither pair")
    joint = list(joint_names).index(a.joint)
    rests = []
    for hold in (plus, minus):
        end = hold.event.t0 + hold.event.hold_s
        if noise.window_s >= hold.event.hold_s:
            raise ValueError(f"{pose}: the dither hold must outlast the {noise.window_s:g} s rest")
        inside = (hold.t >= hold.event.t0) & (hold.t <= end)
        t, q = hold.t[inside], np.asarray(hold.q_rad, dtype=np.float64)[inside]
        if t.size < 2 or not np.isfinite(q).all():
            raise ValueError(f"{pose} {a.joint}: the dither hold has too few or non-finite samples")
        rests.append(q[rest_window(t, end, noise.window_s)].mean(axis=0))
    moved = rests[0][joint] - rests[1][joint]
    tau = [
        hold.event.kp * (math.radians(hold.event.target_after_deg) - rest[joint])
        for hold, rest in zip((plus, minus), rests, strict=True)
    ]
    return Measurement(
        pose=pose,
        joint=a.joint,
        kp=a.kp,
        dither_deg=a.delta_deg,
        q_rad={name: float(0.5 * (x + y)) for name, x, y in zip(joint_names, *rests, strict=True)},
        tau_pd_nm=0.5 * (tau[0] + tau[1]),
        friction_nm=0.5 * (tau[0] - tau[1]),
        stuck=bool(moved < stuck_fraction * math.radians(a.delta_deg - b.delta_deg)),
    )


def dither_friction_nm(measurements: Sequence[Measurement]) -> dict[str, float]:
    """Per joint, the median friction of its pairs that are not stuck, never below zero."""
    by_joint: dict[str, list[float]] = {}
    for m in measurements:
        if not m.stuck:
            by_joint.setdefault(m.joint, []).append(m.friction_nm)
    return {joint: max(0.0, float(np.median(values))) for joint, values in by_joint.items()}


def stuck_pairs(measurements: Sequence[Measurement]) -> tuple[list[str], list[str]]:
    """The stuck pairs ("<pose> <joint>"), and the joints whose every pair is stuck."""
    pairs = [f"{m.pose} {m.joint}" for m in measurements if m.stuck]
    joints = dict.fromkeys(m.joint for m in measurements)
    unfollowed = [j for j in joints if all(m.stuck for m in measurements if m.joint == j)]
    return pairs, unfollowed


def _solve(a: np.ndarray, b: np.ndarray, ridge: float, g: np.ndarray, h: np.ndarray) -> np.ndarray:
    """Ridge least squares for ``a x = b`` inside the row space of ``a``, keeping ``g x <= h``."""
    u, s, vt = np.linalg.svd(a, full_matrices=False)
    keep = s > matrix_rank_tolerance(s, a.shape)
    u, s, v = u[:, keep], s[keep], vt[keep].T
    x: np.ndarray = v @ (s / (s**2 + ridge**2) * (u.T @ b))
    if (g @ x <= h).all():
        return x
    m = np.vstack([a @ v, ridge * np.eye(v.shape[1])])
    c = np.concatenate([b, np.zeros(v.shape[1])])
    gv = g @ v
    result = minimize(
        lambda z: 0.5 * float(np.sum(np.square(m @ z - c))),
        v.T @ x,
        jac=lambda z: m.T @ (m @ z - c),
        method="SLSQP",
        constraints=[{"type": "ineq", "fun": lambda z: h - gv @ z, "jac": lambda z: -gv}],
    )
    if not result.success:
        raise RuntimeError(f"the bounded gravity fit did not converge: {result.message}")
    bounded: np.ndarray = v @ result.x
    return bounded


def _choose_ridge(
    a: np.ndarray, b: np.ndarray, poses: np.ndarray, g: np.ndarray, h: np.ndarray
) -> float:
    """The ridge, among zero and the singular values of ``a``, with the least leave-one-pose-out error."""
    s = np.linalg.svd(a, compute_uv=False)

    def held_out_mse(ridge: float) -> float:
        errors = [
            a[poses == pose] @ _solve(a[poses != pose], b[poses != pose], ridge, g, h)
            - b[poses == pose]
            for pose in np.unique(poses)
        ]
        return float(np.mean(np.square(np.concatenate(errors))))

    return min([0.0, *map(float, s[s > matrix_rank_tolerance(s, a.shape)])], key=held_out_mse)


def fit_gravity(
    arm: ArmModel, measurements: Sequence[Measurement], compensation: Compensation
) -> GravityFit:
    """Fit the gravity links to ``measurements``, which were held with ``compensation``.

    Needs at least three poses (each held-out pose's ridge is chosen by cross-validation over
    the others). Raises ``ValueError`` when the masses compensated do not fit the model or
    the fit drives a mass to zero.
    """
    poses = np.asarray([m.pose for m in measurements])
    if np.unique(poses).size < 3:
        raise ValueError(f"need measurements at three or more poses, got {np.unique(poses).size}")
    unknown = {m.joint for m in measurements} - set(arm.measured_joints)
    if unknown:
        raise ValueError(f"{sorted(unknown)} are not {arm.arm_id} measured joints")
    compensated = arm.model.copy()
    apply_inertial_overrides(compensated, compensation.links)
    data = compensated.createData()
    a_rows, urdf_error = [], []
    for m in measurements:
        q = arm.q(m.q_rad)
        row = arm.joint_names.index(m.joint)
        given = (
            pin.computeGeneralizedGravity(compensated, data, q)[row]
            if compensation.gravity
            else 0.0
        )
        urdf_error.append(m.tau_pd_nm + float(given) - float(arm.gravity(q)[row]))
        a_rows.append(arm.scaled_regressor(q)[arm.measured_joints.index(m.joint)])
    a, b = np.vstack(a_rows), np.asarray(urdf_error)
    masses = np.asarray([arm.model.inertias[j].mass for j in arm.fitted_joint_ids])
    coms = np.asarray([arm.model.inertias[j].lever for j in arm.fitted_joint_ids])
    moments = masses[:, None] * coms
    n = GRAVITY_PARAMS_PER_BODY
    reach = arm.reach_m
    g_rows, h = [], []
    for k, (mass, moment) in enumerate(zip(masses, moments, strict=True)):
        mass_row = np.zeros(a.shape[1])
        mass_row[n * k] = 1.0
        g_rows.append(-mass_row)
        h.append(mass)
        for axis in range(3):
            moment_row = np.zeros(a.shape[1])
            moment_row[n * k + 1 + axis] = 1.0
            g_rows += [moment_row - reach * mass_row, -moment_row - reach * mass_row]
            h += [reach * mass - moment[axis], reach * mass + moment[axis]]
    g, h_ = np.vstack(g_rows) * arm.column_scale, np.asarray(h)
    held_out = np.empty_like(b)
    for pose in np.unique(poses):
        train = poses != pose
        ridge = _choose_ridge(a[train], b[train], poses[train], g, h_)
        held_out[~train] = b[~train] - a[~train] @ _solve(a[train], b[train], ridge, g, h_)
    ridge = _choose_ridge(a, b, poses, g, h_)
    x = _solve(a, b, ridge, g, h_)
    delta = (x * arm.column_scale).reshape(-1, n)
    links, urdf_links = {}, {}
    for k, link in enumerate(arm.fitted_links):
        mass = masses[k] + delta[k, 0]
        if mass <= 0.0:
            raise ValueError(f"{arm.arm_id}: the fit drives {link}'s mass to zero")
        com = (moments[k] + delta[k, 1:]) / mass
        links[link] = LinkInertial(
            mass_kg=float(mass), com_m=(float(com[0]), float(com[1]), float(com[2]))
        )
        urdf_links[link] = LinkInertial(
            mass_kg=float(masses[k]),
            com_m=(float(coms[k, 0]), float(coms[k, 1]), float(coms[k, 2])),
        )
    s = np.linalg.svd(a, compute_uv=False)
    s = s[s > matrix_rank_tolerance(s, a.shape)]
    return GravityFit(
        measurements=list(measurements),
        joints=arm.measured_joints,
        urdf_error=b,
        running_error=np.asarray([m.tau_pd_nm for m in measurements]),
        held_out_error=held_out,
        fit_error=b - a @ x,
        links=links,
        urdf_links=urdf_links,
        ridge=ridge,
        rank=int(s.size),
        singular_values=tuple(map(float, s)),
    )


def refusals(fit: GravityFit) -> list[str]:
    """Why the fit should not be offered: on any measured joint, held-out RMS above the URDF's or
    the running model's, or no measurement. Empty when it may be offered."""
    held, urdf, running = (
        fit.rms(v) for v in (fit.held_out_error, fit.urdf_error, fit.running_error)
    )
    reasons = []
    for joint in fit.joints:
        if joint not in held:
            reasons.append(f"{joint} has no measurement")
            continue
        for name, other in (("URDF", urdf[joint]), ("running model", running[joint])):
            if held[joint] > other:
                reasons.append(
                    f"{joint} held-out RMS {held[joint]:.3f} Nm is worse than the {name}'s {other:.3f} Nm"
                )
    return reasons


def fit_stats(fit: GravityFit) -> dict[str, Any]:
    """The fit's statistics, as a calibration file stores them."""
    by_kind: Mapping[str, dict[str, float]] = {
        kind: fit.rms(values)
        for kind, values in (
            ("urdf", fit.urdf_error),
            ("running", fit.running_error),
            ("held_out", fit.held_out_error),
            ("fit", fit.fit_error),
        )
    }
    return {
        "ridge": fit.ridge,
        "rank": fit.rank,
        "singular_values": list(fit.singular_values),
        "poses": list(dict.fromkeys(m.pose for m in fit.measurements)),
        "measurements": len(fit.measurements),
        "stuck_pairs": stuck_pairs(fit.measurements)[0],
        "rms_nm": {
            joint: {kind: rms[joint] for kind, rms in by_kind.items()} for joint in by_kind["urdf"]
        },
        "urdf_links": {
            name: {"mass_kg": link.mass_kg, "com_m": list(link.com_m)}
            for name, link in fit.urdf_links.items()
        },
    }
