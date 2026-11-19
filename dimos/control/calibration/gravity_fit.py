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

"""Gravity-model identification of one arm from a step-test recording.

    python -m dimos.control.calibration.gravity_fit recordings/<run-id> \
        --robot dual-openyam --arm left [--save]

At every pose the driver dithers each of the spec's measured joints by +d and
-d degrees. At rest the motor torque is ``g_model(q) + kp (q_target - q)``
plus friction, so the PD torque averaged over the two dithers is the error of
the gravity model the connection compensated (friction, which flips sign with
the approach direction, cancels) and half their difference is the Coulomb
friction. The pose markers record that compensation (``Compensation``: none,
the URDF or the masses of a calibration file), which turns the PD torque into
the error of the URDF; a recording that does not say is refused. The error is
fitted with pinocchio's torque regressor to corrections of the mass and first
moment ``(m, m*c)`` of the spec's ``gravity_links``, ridge-regularised toward
the URDF values, so directions the poses cannot determine stay at the URDF.

The report leads with the held-out error: by default every fifth pose is kept
out of the fit and the corrected model is scored on it. Masses must stay
positive and each COM within ``COM_BOX_M`` of the URDF COM, or the fit is
reported as failed and no calibration file is written. A passing fit writes
``gravity_report/gravity_<arm>_<date>.json``, an ``ArmCalibration`` (see
``schema``) holding the fitted links, the fit statistics and the median
friction per joint over the dither pairs that measured it; ``--save`` also
stores it as the arm's calibration file, keeping the gains already there (and
the checks, while the fitted masses stay the same). The hard bounds keep a fit
physical, not right, so ``--save`` also needs at least one held-out pose and a
held-out error no worse after the fit on every measured joint; ``--force``
saves without that.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Mapping, Sequence
import csv
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
from typing import Any

import matplotlib
import numpy as np
import pinocchio as pin
from scipy.optimize import minimize
import typer

from dimos.control.calibration._inertial import (
    LinkInertial,
    apply_inertial_overrides,
    load_inertial_overrides,
)
from dimos.control.calibration.gravity_poses import (
    GRAVITY_PARAMS_PER_BODY,
    ArmModel,
)
from dimos.control.calibration.robots import arm_spec
from dimos.control.calibration.schema import (
    ArmCalibration,
    ArmCalibrationError,
    GravityCalibration,
    calibration_path,
    gravity_links_from_inertial,
    load_calibration,
    load_for_spec,
    merge_patch,
    save_calibration,
    to_inertial_overrides,
)
from dimos.control.calibration.spec import ArmCalibrationSpec
from dimos.control.calibration.step_analysis import (
    REST_WINDOW_S,
    rest_window,
    segment_steps,
)
from dimos.control.calibration.step_report import (
    Recording,
    load_recording,
)
from dimos.control.connection.arm_config import ConfigPatch
from dimos.memory.store.sqlite import SqliteStore

matplotlib.use("Agg")
import matplotlib.pyplot as plt

GRAVITY_EVENTS_STREAM = "gravity_events"
COMPENSATION_KEY = "compensation"
REPORT_DIR_NAME = "gravity_report"
DEFAULT_RIDGE_NM = 0.05  # torque-equivalent weight of one column scale of correction
# Ridge weights tried by leave-one-pose-out cross-validation when none is given.
RIDGE_GRID_NM = (0.02, 0.05, 0.1, 0.2, 0.4, 0.8, 1.6)
MASS_RATIO_BOUNDS = (0.5, 2.0)  # a fitted mass stays within this ratio of the URDF mass
DEFAULT_HOLDOUT_EVERY = 5
MIN_POSES_FOR_HOLDOUT = 6
COM_BOX_M = 0.05  # a fitted COM may move this far per axis from the URDF COM
# Singular directions weaker than this share of the strongest carry no usable
# information from the recorded poses (1 % -> a 0.1 Nm error would move the
# parameter by ten column scales).
FIT_RANK_TOLERANCE = 1e-2
DITHER_SATURATION_FRACTION = 0.95  # friction this close to kp*d/2 means the joint never moved
DETERMINED_FRACTION = 0.5  # share of a parameter's direction inside the identifiable subspace
FRICTION_FLAG_RATIO = 2.0
FRICTION_FLAG_MIN_NM = 0.1
PARAM_NAMES = ("m", "m*cx", "m*cy", "m*cz")

_BEFORE_COLOR = "#2a78d6"
_AFTER_COLOR = "#eb6834"
_INK = "#52514e"
_GRID = "#e5e4df"
_SURFACE = "#fcfcfb"


@dataclass(frozen=True)
class PoseMarker:
    """Marker published by ``JointStepTestModule.mark`` at the start of a pose."""

    label: str
    t0: float
    targets_deg: dict[str, float]
    data: dict[str, Any]

    @classmethod
    def from_json(cls, text: str) -> PoseMarker:
        raw = json.loads(text)
        return cls(
            label=str(raw["label"]),
            t0=float(raw["t0"]),
            targets_deg={str(k): float(v) for k, v in raw.get("targets_deg", {}).items()},
            data=dict(raw.get("data", {})),
        )


@dataclass(frozen=True)
class GravityRecording:
    recording: Recording
    markers: list[PoseMarker]


@dataclass(frozen=True)
class Compensation:
    """The gravity torque the connection added while the poses were held.

    None when ``gravity`` is False (the PD alone held the arm), else the arm's
    model with ``links`` in place of the URDF's (none: the URDF itself). A
    pose marker carries it under ``COMPENSATION_KEY`` (``as_data``).
    """

    gravity: bool
    links: dict[str, LinkInertial] = field(default_factory=dict)

    def as_data(self) -> dict[str, Any]:
        return {
            "gravity": self.gravity,
            "links": {
                name: {"mass_kg": link.mass_kg, "com_m": list(link.com_m)}
                for name, link in self.links.items()
            },
        }

    @classmethod
    def from_data(cls, raw: Mapping[str, Any]) -> Compensation:
        return cls(
            gravity=bool(raw["gravity"]), links=load_inertial_overrides(raw.get("links", {}))
        )


@dataclass(frozen=True)
class Measurement:
    """Resting torque of one joint at one pose, averaged over a +d/-d dither pair."""

    pose: str
    joint: str
    kp: float
    kd: float
    dither_deg: float
    q_rad: dict[str, float]  # every arm joint by name, mean of the two rest windows
    tau_pd_nm: float  # kp (q_target - q) averaged over the pair: what the compensation left
    friction_nm: float  # half the difference: Coulomb friction (>= 0 when the joint settles)

    def as_row(self) -> dict[str, Any]:
        row = asdict(self)
        row["q_deg"] = " ".join(f"{name}={math.degrees(v):.2f}" for name, v in self.q_rad.items())
        del row["q_rad"]
        return row


@dataclass(frozen=True)
class ParameterEstimate:
    link: str
    name: str
    urdf: float
    delta: float
    std_err: float
    observable: float  # share of this parameter's direction inside the identifiable subspace
    determined: bool

    @property
    def value(self) -> float:
        return self.urdf + self.delta


@dataclass(frozen=True)
class FitResult:
    train_poses: list[str]
    holdout_poses: list[str]
    measurements: list[Measurement]
    residual_before: np.ndarray  # tau_pd per measurement (model error before correction)
    residual_after: np.ndarray  # after applying the fitted correction
    is_holdout: np.ndarray
    parameters: list[ParameterEstimate]
    links: dict[str, LinkInertial]
    urdf_links: dict[str, LinkInertial]
    rank: int
    singular_values: tuple[float, ...]
    condition_number: float
    ridge_nm: float
    bound_violations: list[str] = field(default_factory=list)
    at_bounds: list[str] = field(default_factory=list)
    ridge_cv_nm: dict[float, float] = field(default_factory=dict)
    arm_id: str = ""
    measured_joints: tuple[str, ...] = ()

    @property
    def bounds_ok(self) -> bool:
        return not self.bound_violations

    def rms_by_joint(self, *, holdout: bool, after: bool) -> dict[str, float]:
        return self.rms_of(self.residual_after if after else self.residual_before, holdout=holdout)

    def rms_of(self, values: np.ndarray, *, holdout: bool) -> dict[str, float]:
        """RMS by measured joint of ``values``, one per measurement, over held-out or fit poses.

        E.g. the raw resting torques, the error of the masses the arm ran.
        """
        values = np.asarray(values, dtype=np.float64)
        if values.shape != (len(self.measurements),):
            raise ValueError(f"need one value per measurement, got {values.shape}")
        out: dict[str, float] = {}
        for joint in self.measured_joints:
            mask = np.asarray(
                [
                    m.joint == joint and bool(h) == holdout
                    for m, h in zip(self.measurements, self.is_holdout, strict=True)
                ]
            )
            if mask.any():
                out[joint] = float(np.sqrt(np.mean(np.square(values[mask]))))
        return out


# Loading


def load_gravity_recording(
    recording_dir: Path, window: tuple[float, float] | None = None
) -> GravityRecording:
    """Joint traces and step events (as the step report) plus the pose markers, if any.

    With ``window``, only what was recorded between those Unix times.
    """
    recording = load_recording(recording_dir, window)
    store = SqliteStore(path=str(recording_dir / "memory.db"), must_exist=True)
    store.start()
    try:
        markers: list[PoseMarker] = []
        if GRAVITY_EVENTS_STREAM in set(store.list_streams()):
            stream: Any = store.stream(GRAVITY_EVENTS_STREAM)
            markers = [PoseMarker.from_json(obs.data.data) for obs in stream]
    finally:
        store.stop()
    if window is not None:
        markers = [marker for marker in markers if window[0] <= marker.t0 <= window[1]]
    markers.sort(key=lambda m: m.t0)
    return GravityRecording(recording=recording, markers=markers)


def pose_windows(markers: list[PoseMarker], t_end: float) -> list[tuple[str, float, float]]:
    """``(label, start, stop)`` per marker; without markers the whole recording is one pose."""
    if not markers:
        return [("all", -math.inf, math.inf)]
    windows = []
    for index, marker in enumerate(markers):
        stop = markers[index + 1].t0 if index + 1 < len(markers) else max(t_end, marker.t0)
        windows.append((marker.label, marker.t0, stop))
    return windows


def trace_names(recording: Recording, joint_names: Sequence[str]) -> dict[str, str]:
    """Recorded trace per arm joint: the same name, or one ``robot/`` prefix on it."""
    found: dict[str, str] = {}
    for name in joint_names:
        if name in recording.traces:
            found[name] = name
            continue
        prefixed = [trace for trace in recording.traces if trace.rsplit("/", 1)[-1] == name]
        if len(prefixed) != 1:
            raise ValueError(
                f"the recording has no single joint trace for {name!r}; "
                f"it has {sorted(recording.traces)}"
            )
        found[name] = prefixed[0]
    return found


def _rest_q_rad(
    recording: Recording, traces: Mapping[str, str], start: float, stop: float
) -> dict[str, float] | None:
    """Mean of every arm joint over the last ``REST_WINDOW_S`` of ``[start, stop]``, radians."""
    first = recording.traces[next(iter(traces.values()))]
    mask = (first.t >= start) & (first.t <= stop)
    if mask.sum() < 2:
        return None
    t = first.t[mask]
    rest = rest_window(t, REST_WINDOW_S)
    return {
        name: math.radians(float(recording.traces[trace].q_deg[mask][rest].mean()))
        for name, trace in traces.items()
    }


def extract_measurements(grec: GravityRecording, spec: ArmCalibrationSpec) -> list[Measurement]:
    """Pair the +d and -d dithers of each of ``spec``'s measured joints at each pose."""
    recording = grec.recording
    traces = trace_names(recording, spec.joint_names)
    measured = set(spec.measured_joints)
    windows = pose_windows(grec.markers, recording.t_end)
    # (pose, joint, kp, kd, |delta|) -> sign -> [(tau_pd, q)]
    groups: dict[
        tuple[str, str, float, float, float], dict[int, list[tuple[float, dict[str, float]]]]
    ] = defaultdict(lambda: defaultdict(list))
    for event, start, stop in segment_steps(recording.events, recording.t_end):
        joint = event.joint.rsplit("/", 1)[-1] if event.joint not in measured else event.joint
        if joint not in measured:
            continue  # another arm, or a joint without a usable gravity signal
        pose = next((label for label, a, b in windows if a <= event.t0 < b), None)
        if pose is None:
            continue
        q = _rest_q_rad(recording, traces, start, stop)
        if q is None:
            continue
        tau_pd = event.kp * (math.radians(event.target_after_deg) - q[joint])
        key = (pose, joint, event.kp, event.kd, abs(event.delta_deg))
        groups[key][int(math.copysign(1, event.delta_deg))].append((tau_pd, q))
    measurements = []
    for (pose, joint, kp, kd, dither), signs in groups.items():
        for (tau_plus, q_plus), (tau_minus, q_minus) in zip(signs[1], signs[-1], strict=False):
            measurements.append(
                Measurement(
                    pose=pose,
                    joint=joint,
                    kp=kp,
                    kd=kd,
                    dither_deg=dither,
                    q_rad={name: 0.5 * (q_plus[name] + q_minus[name]) for name in q_plus},
                    tau_pd_nm=0.5 * (tau_plus + tau_minus),
                    friction_nm=0.5 * (tau_plus - tau_minus),
                )
            )
    return measurements


def recorded_compensation(grec: GravityRecording) -> Compensation | None:
    """The compensation the first pose marker that records one holds; ``None`` without."""
    for marker in grec.markers:
        if COMPENSATION_KEY in marker.data:
            return Compensation.from_data(marker.data[COMPENSATION_KEY])
    return None


def to_urdf_error(
    arm: ArmModel, measurements: list[Measurement], compensation: Compensation
) -> list[Measurement]:
    """``measurements`` as errors of the URDF gravity, which the fit corrects.

    Each resting torque gains what was compensated beyond the URDF: minus the
    whole URDF gravity when nothing was. Raises ``ValueError`` when the
    compensated links do not fit the arm's model.
    """
    model = arm.model.copy()
    apply_inertial_overrides(model, compensation.links)
    data = model.createData()
    rows = {name: index for index, name in enumerate(arm.joint_names)}
    corrected = []
    for m in measurements:
        q = arm.q(m.q_rad)
        compensated = (
            np.asarray(pin.computeGeneralizedGravity(model, data, q))
            if compensation.gravity
            else np.zeros(arm.dof)
        )
        offset = float((compensated - arm.gravity(q))[rows[m.joint]])
        corrected.append(replace(m, tau_pd_nm=m.tau_pd_nm + offset))
    return corrected


def default_holdout(poses: list[str], every: int = DEFAULT_HOLDOUT_EVERY) -> list[str]:
    """Every ``every``-th pose in recording order, none when there are too few poses."""
    if len(poses) < MIN_POSES_FOR_HOLDOUT or every <= 0:
        return []
    return [pose for index, pose in enumerate(poses, 1) if index % every == 0]


def ordered_poses(measurements: list[Measurement]) -> list[str]:
    seen: dict[str, None] = {}
    for m in measurements:
        seen.setdefault(m.pose, None)
    return list(seen)


# Fitting


def fit_gravity(
    arm: ArmModel,
    measurements: list[Measurement],
    *,
    ridge_nm: float | None = None,
    holdout: list[str] | None = None,
) -> FitResult:
    """Ridge least squares for ``(m, m*c)`` corrections of the fitted links.

    ``tau_pd = Y(q) @ delta_phi`` row by row for the stepped joint of each
    measurement. Columns are scaled by a plausible error (0.2 kg, 0.02 kg m).
    The correction lives in the identifiable subspace of the training regressor
    (singular values above ``FIT_RANK_TOLERANCE`` of the largest); directions
    outside it get exactly zero correction, i.e. stay at the URDF, and within
    it a ridge of ``ridge_nm`` shrinks weak directions toward the URDF too.
    The mass ratio and COM box bounds are linear in the correction once each
    COM bound is multiplied through by the fitted mass, which they keep
    positive, so the solve is a convex quadratic program, physical by
    construction.
    Standard errors come from the ridge covariance with the residual variance
    of the training rows; a parameter is "determined" when at least half of
    its direction lies in the identifiable subspace.
    """
    if not measurements:
        raise ValueError("no paired +/- dithers found; nothing to fit")
    poses = ordered_poses(measurements)
    holdout_set = set(default_holdout(poses) if holdout is None else holdout)
    unknown = holdout_set - set(poses)
    if unknown:
        raise ValueError(f"hold-out poses not in the recording: {sorted(unknown)}")
    is_holdout = np.asarray([m.pose in holdout_set for m in measurements])
    if is_holdout.all():
        raise ValueError("every pose is held out; nothing to fit")
    rows = []
    for m in measurements:
        if m.joint not in arm.measured_joints:
            raise ValueError(
                f"{m.joint} is not one of the {arm.arm_id} measured joints {arm.measured_joints}"
            )
        full = arm.scaled_regressor(arm.q(m.q_rad))
        rows.append(full[arm.measured_joints.index(m.joint)])
    a_scaled = np.vstack(rows)
    b = np.asarray([m.tau_pd_nm for m in measurements])
    a_train = a_scaled[~is_holdout]
    b_train = b[~is_holdout]

    sigma, basis = _identifiable_directions(a_train)
    rank = sigma.size
    if rank == 0:
        raise ValueError("the training poses carry no gravity information (zero regressor)")
    condition = float(sigma[0] / sigma[-1])

    masses0 = np.array([float(arm.model.inertias[j].mass) for j in arm.fitted_joint_ids])
    coms0 = np.array(
        [np.asarray(arm.model.inertias[j].lever, dtype=np.float64) for j in arm.fitted_joint_ids]
    )

    def to_delta_phi(x: np.ndarray) -> np.ndarray:
        d_mass = x[0::4]
        shift = x.reshape(-1, 4)[:, 1:]
        moment = d_mass[:, None] * coms0 + (masses0 + d_mass)[:, None] * shift
        return np.concatenate([np.column_stack([d_mass, moment])[k] for k in range(len(masses0))])

    def to_x(delta_phi: np.ndarray) -> np.ndarray:
        d_mass = delta_phi[0::4]
        moment = delta_phi.reshape(-1, 4)[:, 1:]
        shift = (moment - d_mass[:, None] * coms0) / (masses0 + d_mass)[:, None]
        return np.column_stack([d_mass, shift]).ravel()

    lower = np.concatenate(
        [[(MASS_RATIO_BOUNDS[0] - 1.0) * m, -COM_BOX_M, -COM_BOX_M, -COM_BOX_M] for m in masses0]
    )
    upper = np.concatenate(
        [[(MASS_RATIO_BOUNDS[1] - 1.0) * m, COM_BOX_M, COM_BOX_M, COM_BOX_M] for m in masses0]
    )
    constraints: list[np.ndarray] = []
    limits: list[float] = []
    unit = np.eye(lower.size)
    for k, (mass0, com0) in enumerate(zip(masses0, coms0, strict=True)):
        mass_row = unit[4 * k]
        constraints += [mass_row, -mass_row]
        limits += [upper[4 * k], -lower[4 * k]]
        for axis in range(3):
            j = 4 * k + 1 + axis
            constraints += [unit[j] - (com0[axis] + upper[j]) * mass_row]
            constraints += [(com0[axis] + lower[j]) * mass_row - unit[j]]
            limits += [upper[j] * mass0, -lower[j] * mass0]
    bound_rows = np.vstack(constraints) * arm.column_scale
    bound_limits = np.asarray(limits)
    train_poses_arr = np.asarray([m.pose for m in measurements])[~is_holdout]

    def solve(a: np.ndarray, b_: np.ndarray, ridge: float, directions: np.ndarray) -> np.ndarray:
        """Scaled correction within ``directions`` fitting ``a`` to ``b_``, ridge and bounds kept."""
        size = directions.shape[1]
        m = np.vstack([a @ directions, ridge * np.eye(size)])
        c = np.concatenate([b_, np.zeros(size)])
        g = bound_rows @ directions
        z = minimize(
            lambda z: 0.5 * float(np.sum(np.square(m @ z - c))),
            np.zeros(size),
            jac=lambda z: m.T @ (m @ z - c),
            method="SLSQP",
            constraints=[
                {"type": "ineq", "fun": lambda z: bound_limits - g @ z, "jac": lambda z: -g}
            ],
            options={"ftol": 1e-15, "maxiter": 1000},
        ).x
        correction: np.ndarray = directions @ z
        return correction

    ridge_cv: dict[float, float] = {}
    if ridge_nm is None:
        # Leave one training pose out at a time; score each ridge on the pose left out.
        for candidate in RIDGE_GRID_NM:
            errors = []
            for pose in sorted(set(train_poses_arr)):
                keep = train_poses_arr != pose
                _, directions = _identifiable_directions(a_train[keep])
                y = solve(a_train[keep], b_train[keep], candidate, directions)
                errors.append(a_train[~keep] @ y - b_train[~keep])
            ridge_cv[candidate] = float(np.sqrt(np.mean(np.square(np.concatenate(errors)))))
        ridge_nm = min(ridge_cv, key=lambda r: ridge_cv[r])

    correction = solve(a_train, b_train, ridge_nm, basis)
    x_phys = np.clip(to_x(correction * arm.column_scale), lower, upper)
    delta = to_delta_phi(x_phys)
    x_scaled = delta / arm.column_scale
    residual_after = b - a_scaled @ x_scaled
    train_residual = residual_after[~is_holdout]
    gain = sigma / (sigma**2 + ridge_nm**2)
    dof = max(len(b_train) - float((sigma * gain).sum()), 1.0)
    sigma2 = float(train_residual @ train_residual) / dof
    covariance = sigma2 * (basis * gain**2) @ basis.T
    std_scaled = np.sqrt(np.clip(np.diag(covariance), 0.0, None))
    std_err = std_scaled * arm.column_scale
    observable = np.diag(basis @ basis.T)
    determined = observable >= DETERMINED_FRACTION
    at_bounds: list[str] = []
    labels = ("mass", "COM x", "COM y", "COM z")
    for k, (lo, hi, v) in enumerate(zip(lower, upper, x_phys, strict=True)):
        if min(v - lo, hi - v) < 1e-6 * (hi - lo):
            link = arm.fitted_links[k // 4]
            at_bounds.append(
                f"{link} {labels[k % 4]} sits at its bound ({'lower' if v - lo < hi - v else 'upper'})"
            )

    parameters: list[ParameterEstimate] = []
    links: dict[str, LinkInertial] = {}
    urdf_links: dict[str, LinkInertial] = {}
    violations: list[str] = []
    for body_index, (link, joint_id) in enumerate(
        zip(arm.fitted_links, arm.fitted_joint_ids, strict=True)
    ):
        inertia = arm.model.inertias[joint_id]
        mass0 = float(inertia.mass)
        com0 = np.asarray(inertia.lever, dtype=np.float64)
        phi0 = np.concatenate([[mass0], mass0 * com0])
        sl = slice(body_index * GRAVITY_PARAMS_PER_BODY, (body_index + 1) * GRAVITY_PARAMS_PER_BODY)
        for k, name in enumerate(PARAM_NAMES):
            parameters.append(
                ParameterEstimate(
                    link=link,
                    name=name,
                    urdf=float(phi0[k]),
                    delta=float(delta[sl][k]),
                    std_err=float(std_err[sl][k]),
                    observable=float(observable[sl][k]),
                    determined=bool(determined[sl][k]),
                )
            )
        mass = mass0 + float(x_phys[sl][0])
        urdf_links[link] = LinkInertial(mass_kg=mass0, com_m=_vec3(com0))
        if mass <= 0.0:
            violations.append(f"{link}: fitted mass {mass:.3f} kg is not positive")
            continue
        shift = x_phys[sl][1:]
        com = com0 + shift
        if np.abs(shift).max() > COM_BOX_M:
            violations.append(
                f"{link}: COM moved {np.abs(shift).max() * 1000:.0f} mm from the URDF COM "
                f"(limit {COM_BOX_M * 1000:.0f} mm)"
            )
        links[link] = LinkInertial(mass_kg=mass, com_m=_vec3(com))

    return FitResult(
        train_poses=[p for p in poses if p not in holdout_set],
        holdout_poses=[p for p in poses if p in holdout_set],
        measurements=measurements,
        residual_before=b,
        residual_after=residual_after,
        is_holdout=is_holdout,
        parameters=parameters,
        links=links,
        urdf_links=urdf_links,
        rank=rank,
        singular_values=tuple(float(v) for v in sigma),
        condition_number=condition,
        ridge_nm=ridge_nm,
        bound_violations=violations,
        at_bounds=at_bounds,
        ridge_cv_nm=ridge_cv,
        arm_id=arm.arm_id,
        measured_joints=arm.measured_joints,
    )


def _identifiable_directions(a: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Singular values of ``a`` above ``FIT_RANK_TOLERANCE`` of the largest, and their directions.

    The directions are orthonormal columns, spanning part of ``a``'s row space;
    none when ``a`` is zero.
    """
    _, s, vt = np.linalg.svd(a, full_matrices=False)
    rank = int((s > FIT_RANK_TOLERANCE * s[0]).sum()) if s.size and s[0] > 0 else 0
    return s[:rank], vt[:rank].T


def _vec3(values: np.ndarray) -> tuple[float, float, float]:
    x, y, z = (float(v) for v in values)
    return (x, y, z)


def friction_flags(measurements: list[Measurement]) -> dict[int, str]:
    """Measurement index -> reason, for dither pairs whose friction stands out for the joint."""
    by_joint: dict[str, list[float]] = defaultdict(list)
    for m in measurements:
        by_joint[m.joint].append(m.friction_nm)
    flags: dict[int, str] = {}
    for index, m in enumerate(measurements):
        median = float(np.median(by_joint[m.joint]))
        # A joint that never moved for +d/-d reports exactly kp*d/2 as "friction":
        # stiction exceeded the dither torque, and the pair cancels nothing.
        saturated = m.kp * math.radians(m.dither_deg) / 2.0
        if m.friction_nm >= DITHER_SATURATION_FRACTION * saturated:
            flags[index] = (
                f"joint did not move for the {m.dither_deg:g} deg dither "
                f"(friction {m.friction_nm:.2f} = kp*d/2); raise --dither-deg or kp"
            )
        elif m.friction_nm < 0.0:
            flags[index] = f"negative friction {m.friction_nm:.2f} Nm: overshoot or not settled"
        elif abs(m.friction_nm - median) > FRICTION_FLAG_MIN_NM and (
            m.friction_nm > FRICTION_FLAG_RATIO * median
            or m.friction_nm < median / FRICTION_FLAG_RATIO
        ):
            flags[index] = f"friction {m.friction_nm:.2f} Nm vs joint median {median:.2f} Nm"
    return flags


# Output


def fit_stats(result: FitResult, recording_dir: Path | None = None) -> dict[str, Any]:
    """The fit's statistics as stored in ``ArmCalibration.gravity.fit``."""
    holdout_before = result.rms_by_joint(holdout=True, after=False)
    holdout_after = result.rms_by_joint(holdout=True, after=True)
    train_before = result.rms_by_joint(holdout=False, after=False)
    train_after = result.rms_by_joint(holdout=False, after=True)
    return {
        "source": "dimos.control.calibration.gravity_fit",
        "recording": recording_dir.name if recording_dir is not None else None,
        "ridge_nm": result.ridge_nm,
        "rank": result.rank,
        "condition_number": result.condition_number,
        "singular_values": list(result.singular_values),
        "train_poses": result.train_poses,
        "holdout_poses": result.holdout_poses,
        "measurements": len(result.measurements),
        "measured_joints": list(result.measured_joints),
        "holdout_rms_nm": {
            joint: {"before": holdout_before[joint], "after": holdout_after[joint]}
            for joint in holdout_before
        },
        "train_rms_nm": {
            joint: {"before": train_before[joint], "after": train_after[joint]}
            for joint in train_before
        },
        "parameters": [asdict(p) for p in result.parameters],
        "urdf_links": {
            name: {"mass_kg": link.mass_kg, "com_m": list(link.com_m)}
            for name, link in result.urdf_links.items()
        },
        "bounds_ok": result.bounds_ok,
        "bound_violations": result.bound_violations,
        "at_bounds": result.at_bounds,
        "ridge_cv_nm": {str(k): v for k, v in result.ridge_cv_nm.items()},
    }


def median_friction(measurements: Sequence[Measurement]) -> dict[str, float]:
    """Median Coulomb friction per joint over the dither pairs ``friction_flags`` leaves alone.

    A flagged pair (the joint never moved, overshot, or stands out) measures
    no friction, so it is left out; a joint whose pairs are all flagged gets
    no value. The median is clamped at zero.
    """
    flags = friction_flags(list(measurements))
    by_joint: dict[str, list[float]] = defaultdict(list)
    for index, m in enumerate(measurements):
        if index not in flags:
            by_joint[m.joint].append(m.friction_nm)
    return {joint: max(0.0, float(np.median(values))) for joint, values in by_joint.items()}


def calibration_from_fit(
    result: FitResult,
    spec: ArmCalibrationSpec,
    *,
    recording_dir: Path | None = None,
    created: datetime | None = None,
) -> ArmCalibration:
    """The arm calibration a passing fit produces; raises ``ValueError`` for a failed fit."""
    if not result.bounds_ok:
        raise ValueError(f"{spec.arm_id}: the fit failed its physical bounds; nothing to store")
    return ArmCalibration.for_spec(
        spec,
        created=created,
        gravity=GravityCalibration(
            links=gravity_links_from_inertial(spec, result.links),
            fit=fit_stats(result, recording_dir),
        ),
        friction_nm=median_friction(result.measurements),
    )


def save_refusals(result: FitResult, running_rms: Mapping[str, float] | None = None) -> list[str]:
    """Why a fit should not become the arm's calibration file; empty when it may.

    The solver's hard bounds keep every fit physical, so passing them says
    nothing about whether the fit is right: it must improve, or at least not
    worsen, the resting error at poses it never saw, on every measured joint,
    compared with the URDF and, given ``running_rms`` (the held-out RMS of the
    masses the arm ran while recording, by joint), with those too. A fit
    pressed against a bound (``at_bounds``) is not refused for that alone.
    """
    if not result.holdout_poses:
        return ["no pose was held out, so the fit was never scored on poses it did not see"]
    before = result.rms_by_joint(holdout=True, after=False)
    after = result.rms_by_joint(holdout=True, after=True)
    reasons: list[str] = []
    for joint in result.measured_joints:
        if joint not in before:
            reasons.append(f"{joint} has no held-out measurement")
            continue
        if after[joint] > before[joint]:
            reasons.append(
                f"{joint} held-out RMS rose from {before[joint]:.3f} to {after[joint]:.3f} Nm"
            )
        running = None if running_rms is None else running_rms.get(joint)
        if running is not None and after[joint] > running:
            reasons.append(
                f"{joint} held-out RMS {after[joint]:.3f} Nm is worse than the running "
                f"calibration's {running:.3f} Nm"
            )
    return reasons


def report_json_name(arm_id: str, created: datetime) -> str:
    """``gravity_<arm>_<date>.json``, the arm id's slashes as underscores."""
    return f"gravity_{arm_id.replace('/', '_')}_{created.date().isoformat()}.json"


def find_previous_json(
    search_dirs: list[Path], arm_id: str, exclude: Path | None = None
) -> Path | None:
    """Newest ``gravity_<arm>_*.json`` under the given directories (and ``*/gravity_report``)."""
    pattern = f"gravity_{arm_id.replace('/', '_')}_*.json"
    found: list[Path] = []
    for base in search_dirs:
        if not base.is_dir():
            continue
        found.extend(base.glob(pattern))
        found.extend(base.glob(f"*/gravity_report/{pattern}"))
    found = [p for p in found if exclude is None or p.resolve() != exclude.resolve()]
    if not found:
        return None
    return max(found, key=lambda p: p.stat().st_mtime)


def previous_links(path: Path) -> dict[str, LinkInertial]:
    """Fitted links of an arm calibration file, or of a legacy ``{"links": ...}`` file."""
    try:
        return to_inertial_overrides(load_calibration(path))
    except ArmCalibrationError:
        return load_inertial_overrides(path)


def compare_with_previous(result: FitResult, previous: Path) -> str:
    links_before = previous_links(previous)
    lines = [
        f"## Compared with {previous}",
        "",
        "| link | mass prev (kg) | mass new (kg) | COM shift new - prev (mm) |",
        "|---|---|---|---|",
    ]
    for name, link in result.links.items():
        old = links_before.get(name)
        if old is None:
            lines.append(f"| {name} | - | {link.mass_kg:.3f} | - |")
            continue
        shift = 1000.0 * (np.asarray(link.com_m) - np.asarray(old.com_m))
        lines.append(
            f"| {name} | {old.mass_kg:.3f} | {link.mass_kg:.3f} | "
            + " ".join(f"{v:+.0f}" for v in shift)
            + " |"
        )
    lines.append("")
    return "\n".join(lines)


def _rms_table(
    result: FitResult, *, holdout: bool, running: Mapping[str, float] | None = None
) -> str:
    before = result.rms_by_joint(holdout=holdout, after=False)
    after = result.rms_by_joint(holdout=holdout, after=True)
    if not before:
        return "(no poses)\n"
    if running is None:
        lines = ["| joint | RMS before (Nm) | RMS after (Nm) |", "|---|---|---|"]
        for joint in before:
            lines.append(f"| {joint} | {before[joint]:.3f} | {after[joint]:.3f} |")
        return "\n".join(lines) + "\n"
    lines = [
        "| joint | URDF RMS (Nm) | running file RMS (Nm) | new RMS (Nm) |",
        "|---|---|---|---|",
    ]
    for joint in before:
        ran = running.get(joint)
        lines.append(
            f"| {joint} | {before[joint]:.3f} | {'-' if ran is None else f'{ran:.3f}'} "
            f"| {after[joint]:.3f} |"
        )
    return "\n".join(lines) + "\n"


def format_report(
    result: FitResult,
    previous: Path | None,
    json_path: Path | None,
    *,
    not_saved: Sequence[str] = (),
    running_rms: Mapping[str, float] | None = None,
) -> str:
    out = [f"# Gravity-model fit: {result.arm_id}", ""]
    if result.bounds_ok:
        out.append(
            f"Result: OK, parameters written to `{json_path}`." if json_path else "Result: OK."
        )
    else:
        out.append("Result: FAILED physical bounds; no parameters written.")
        out.extend(f"- {v}" for v in result.bound_violations)
    if not_saved:
        out.append("")
        out.append("Not saved as the arm's calibration (save anyway with --force):")
        out.extend(f"- {reason}" for reason in not_saved)
    if result.at_bounds:
        out.append("")
        out.append(
            "Warning: the fit pressed against its physical bounds; the data want more than "
            "the model allows there, so treat these links with care:"
        )
        out.extend(f"- {v}" for v in result.at_bounds)
    out.append("")
    out.append(
        f"## Held-out error ({len(result.holdout_poses)} pose(s): "
        f"{', '.join(result.holdout_poses) or 'none'})"
    )
    out.append("")
    out.append(
        "Resting gravity torque error at poses the fit never saw, before and after the correction"
        + (", and with the masses the arm ran while recording." if running_rms is not None else ".")
    )
    out.append("")
    out.append(_rms_table(result, holdout=True, running=running_rms))
    out.append(
        f"## Fit residual ({len(result.train_poses)} pose(s): {', '.join(result.train_poses)})"
    )
    out.append("")
    out.append(_rms_table(result, holdout=False))
    out.append("## Regressor")
    out.append("")
    out.append(
        f"{len(result.measurements)} dither pairs; identifiable rank {result.rank} of "
        f"{len(result.parameters)} parameters; condition number {result.condition_number:.1f}; "
        f"ridge {result.ridge_nm:g} Nm toward the URDF values"
        + (
            " (chosen by leave-one-pose-out CV: "
            + ", ".join(f"{r:g} Nm -> {e:.3f}" for r, e in sorted(result.ridge_cv_nm.items()))
            + ")"
            if result.ridge_cv_nm
            else ""
        )
        + ". Singular values: "
        + " ".join(f"{s:.3g}" for s in result.singular_values)
    )
    out.append("")
    out.append("## Parameters")
    out.append("")
    out.append(
        "Corrections are confined to the parameter combinations the poses can observe; "
        "everything else stays at the URDF value. 'observable' is the share of a parameter's "
        "own direction inside that subspace: below "
        f"{DETERMINED_FRACTION:.0%} the parameter only moves as part of a combination and its "
        "individual value is not determined, whatever its standard error."
    )
    out.append("")
    out.append("| link | parameter | URDF | fitted | std err | observable | determined |")
    out.append("|---|---|---|---|---|---|---|")
    for p in result.parameters:
        unit = "kg" if p.name == "m" else "kg m"
        out.append(
            f"| {p.link} | {p.name} | {p.urdf:.4f} {unit} | {p.value:.4f} | {p.std_err:.4f} "
            f"| {p.observable:.2f} | {'yes' if p.determined else 'no'} |"
        )
    out.append("")
    out.append("| link | URDF mass (kg) | fitted mass (kg) | URDF COM (mm) | fitted COM (mm) |")
    out.append("|---|---|---|---|---|")
    for name, urdf in result.urdf_links.items():
        fitted = result.links.get(name)
        com0 = " ".join(f"{1000 * v:+.0f}" for v in urdf.com_m)
        com1 = " ".join(f"{1000 * v:+.0f}" for v in fitted.com_m) if fitted else "invalid"
        mass1 = f"{fitted.mass_kg:.3f}" if fitted else "invalid"
        out.append(f"| {name} | {urdf.mass_kg:.3f} | {mass1} | {com0} | {com1} |")
    out.append("")
    out.append("## Friction per pose and joint (half the +/- dither difference, Nm)")
    out.append("")
    flags = friction_flags(result.measurements)
    out.append(
        "| pose | joint | kp | dither (deg) | gravity error before | after | friction | note |"
    )
    out.append("|---|---|---|---|---|---|---|---|")
    for index, (m, before, after, held) in enumerate(
        zip(
            result.measurements,
            result.residual_before,
            result.residual_after,
            result.is_holdout,
            strict=True,
        )
    ):
        note = flags.get(index, "")
        if held:
            note = ("held out; " + note) if note else "held out"
        out.append(
            f"| {m.pose} | {m.joint} | {m.kp:g} | {m.dither_deg:g} | {before:+.3f} | {after:+.3f} "
            f"| {m.friction_nm:.3f} | {note} |"
        )
    out.append("")
    if previous is not None and result.links:
        out.append(compare_with_previous(result, previous))
    return "\n".join(out) + "\n"


def _style(ax: Any) -> None:
    ax.set_facecolor(_SURFACE)
    ax.grid(True, axis="y", color=_GRID, linewidth=0.6)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(_GRID)
    ax.tick_params(colors=_INK, labelsize=8)


def plot_resting_torque(result: FitResult, path: Path) -> None:
    """One panel per joint: resting torque error per pose before and after the fit."""
    poses = ordered_poses(result.measurements)
    joints = [j for j in result.measured_joints if any(m.joint == j for m in result.measurements)]
    fig, axes = plt.subplots(
        len(joints),
        1,
        figsize=(max(7.0, 0.5 * len(poses) + 3), 2.2 * len(joints) + 0.8),
        squeeze=False,
        sharex=True,
    )
    fig.patch.set_facecolor(_SURFACE)
    x = np.arange(len(poses))
    width = 0.38
    for ax, joint in zip(axes[:, 0], joints, strict=True):
        _style(ax)
        before = np.full(len(poses), np.nan)
        after = np.full(len(poses), np.nan)
        held = np.zeros(len(poses), dtype=bool)
        for m, b, a, h in zip(
            result.measurements,
            result.residual_before,
            result.residual_after,
            result.is_holdout,
            strict=True,
        ):
            if m.joint == joint:
                i = poses.index(m.pose)
                before[i], after[i], held[i] = b, a, h
        ax.axhline(0.0, color=_INK, linewidth=0.8)
        ax.bar(
            x - width / 2,
            before,
            width,
            color=_BEFORE_COLOR,
            label="before",
            edgecolor=_SURFACE,
            linewidth=1.0,
        )
        bars = ax.bar(
            x + width / 2,
            after,
            width,
            color=_AFTER_COLOR,
            label="after",
            edgecolor=_SURFACE,
            linewidth=1.0,
        )
        for bar, h in zip(bars, held, strict=True):
            if h:
                bar.set_hatch("//")
        ax.set_ylabel(f"{joint}\nerror (Nm)", fontsize=8, color=_INK)
    axes[-1, 0].set_xticks(x)
    axes[-1, 0].set_xticklabels(poses, rotation=45, ha="right", fontsize=7)
    axes[0, 0].legend(loc="upper right", fontsize=8, frameon=False)
    fig.suptitle(
        f"{result.arm_id}: resting gravity torque error per pose (hatched: held out)",
        fontsize=10,
        color=_INK,
        x=0.02,
        ha="left",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(path, dpi=120)
    plt.close(fig)


def write_measurements_csv(result: FitResult, path: Path) -> None:
    rows = []
    for m, before, after, held in zip(
        result.measurements,
        result.residual_before,
        result.residual_after,
        result.is_holdout,
        strict=True,
    ):
        row = m.as_row()
        row["error_before_nm"] = float(before)
        row["error_after_nm"] = float(after)
        row["holdout"] = bool(held)
        rows.append(row)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["pose"])
        writer.writeheader()
        writer.writerows(rows)


def generate_gravity_report(
    recording_dir: Path,
    spec: ArmCalibrationSpec,
    *,
    ridge_nm: float | None = None,
    holdout: list[str] | None = None,
    previous: Path | None = None,
    arm: ArmModel | None = None,
    now: datetime | None = None,
    save_to: Path | None = None,
    force: bool = False,
) -> tuple[FitResult, Path, ArmCalibration | None]:
    """Fit one arm's recording and write ``gravity_report/`` inside it.

    The resting torques are taken against what the pose markers record the
    connection compensated (``to_urdf_error``); a recording that does not
    record it raises ``ValueError``. A passing fit also writes
    ``gravity_<arm>_<date>.json``, an ``ArmCalibration``; with ``save_to`` it
    is saved there too, keeping the gains and notes of a calibration already
    there for the same model (see ``merge_with_stored``), unless
    ``save_refusals`` objects and ``force`` is not given.
    """
    arm = arm or ArmModel(spec)
    if arm.arm_id != spec.arm_id:
        raise ValueError(f"arm model is for {arm.arm_id}, not {spec.arm_id}")
    now = now or datetime.now(timezone.utc)
    grec = load_gravity_recording(recording_dir)
    compensation = recorded_compensation(grec)
    if compensation is None:
        raise ValueError(
            f"{recording_dir} does not record the gravity the connection compensated at its "
            "poses, so the fit cannot tell the model's error from the gravity torque; record "
            "the poses again with the calibration procedure"
        )
    measurements = to_urdf_error(arm, extract_measurements(grec, spec), compensation)
    result = fit_gravity(arm, measurements, ridge_nm=ridge_nm, holdout=holdout)
    out_dir = recording_dir / REPORT_DIR_NAME
    out_dir.mkdir(exist_ok=True)
    json_path: Path | None = out_dir / report_json_name(spec.arm_id, now)
    if previous is None:
        stored = calibration_path(spec.arm_id)
        previous = (
            stored
            if stored.exists()
            else find_previous_json([recording_dir.parent], spec.arm_id, exclude=json_path)
        )
    calibration: ArmCalibration | None = None
    not_saved: list[str] = []
    if result.bounds_ok and json_path is not None:
        calibration = calibration_from_fit(result, spec, recording_dir=recording_dir, created=now)
        json_path.write_text(calibration.model_dump_json(indent=2) + "\n")
        if save_to is not None:
            not_saved = [] if force else save_refusals(result)
            if not not_saved:
                save_calibration(save_to, merge_with_stored(save_to, calibration, spec))
    else:
        json_path = None
    (out_dir / "report.md").write_text(
        format_report(result, previous, json_path, not_saved=not_saved)
    )
    write_measurements_csv(result, out_dir / "measurements.csv")
    plot_resting_torque(result, out_dir / "resting_torque.png")
    return result, out_dir, calibration


def merge_with_stored(path: Path, new: ArmCalibration, spec: ArmCalibrationSpec) -> ArmCalibration:
    """``new``'s fit and friction merged into a stored calibration for the same arm and model.

    The stored gains and notes stay. Its checks (float drift and the like)
    ran with the old masses, so they are kept only when the fitted links are
    unchanged; the file they were in stays in ``history/``. See
    ``schema.merge_patch``.
    """
    try:
        existing = load_for_spec(spec, path)
    except ArmCalibrationError:
        return new
    gravity = new.gravity
    patch = ConfigPatch(
        arm_id=new.arm_id,
        links=None if gravity is None else gravity.links,
        fit=None if gravity is None else gravity.fit,
        friction_nm=new.friction_nm,
    )
    return merge_patch(existing, patch, created=new.created)


app = typer.Typer(add_completion=False)


@app.command()
def main(
    recording_dir: Path = typer.Argument(..., help="recordings/<run-id> with memory.db"),
    robot: str = typer.Option(..., help="A known robot, e.g. dual-openyam or b601-dm"),
    arm: str | None = typer.Option(None, help="Which arm the recording moved, e.g. left"),
    ridge_nm: float | None = typer.Option(
        None, help="Ridge weight toward the URDF, Nm; default: chosen by leave-one-pose-out CV"
    ),
    holdout: str = typer.Option(
        "",
        help="Comma-separated pose labels to hold out; default every 5th pose",
    ),
    previous: Path | None = typer.Option(
        None, help="Earlier calibration or gravity json to compare with"
    ),
    save: bool = typer.Option(
        False,
        "--save",
        help="Also store the fit as the arm's calibration file, if it improves the held-out error",
    ),
    force: bool = typer.Option(
        False, "--force", help="With --save: store the fit even if its held-out check fails"
    ),
) -> None:
    """Fit the gravity model from a recording and write gravity_report/ next to it."""
    labels = [p.strip() for p in holdout.split(",") if p.strip()] or None
    try:
        spec = arm_spec(robot, arm)
    except ValueError as error:
        raise typer.BadParameter(str(error)) from None
    target = calibration_path(spec.arm_id) if save else None
    result, out_dir, calibration = generate_gravity_report(
        recording_dir,
        spec,
        ridge_nm=ridge_nm,
        holdout=labels,
        previous=previous,
        save_to=target,
        force=force,
    )
    typer.echo((out_dir / "report.md").read_text())
    typer.echo(f"Wrote {out_dir}")
    if not result.bounds_ok:
        raise typer.Exit(1)
    if target is None or calibration is None:
        return
    refusals = save_refusals(result)
    if refusals and not force:
        typer.echo(
            f"Did not save the calibration of {spec.arm_id}: {'; '.join(refusals)}. "
            "Re-run with --force to save it anyway."
        )
        raise typer.Exit(1)
    typer.echo(f"Saved the calibration of {spec.arm_id} to {target}")
    if refusals:
        typer.echo(f"  despite (--force): {'; '.join(refusals)}")
    if result.at_bounds:
        typer.echo(f"  at bounds: {'; '.join(result.at_bounds)}")


if __name__ == "__main__":
    sys.exit(app())
