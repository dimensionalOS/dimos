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

"""Gravity-model identification from a ``dual_openyam_step_test`` recording.

    python -m dimos.robot.manipulators.calibration.gravity_fit recordings/<run-id>
    python -m ...gravity_fit recordings/<right-run> --side right --merge <left gravity json>

At every pose the driver (``demo_gravity_id``) dithers each measured joint by
+d and -d degrees. At rest the motor torque is ``g_model(q) + kp (q_target -
q)`` plus friction, so the PD torque averaged over the two dithers is the
gravity-model error (friction, which flips sign with the approach direction,
cancels) and half their difference is the Coulomb friction. The error is fitted
with pinocchio's torque regressor to corrections of the mass and first moment
``(m, m*c)`` of the moving arm's links (``--side``), ridge-regularised toward
the URDF values, so directions the poses cannot determine stay at the URDF.

The report leads with the held-out error: by default every fifth pose is kept
out of the fit and the corrected model is scored on it. Masses must stay
positive and each COM within ``COM_BOX_M`` of the URDF COM, or the fit is
reported as failed and no JSON is written. The result
``gravity_<side>_<date>.json`` is what ``DamiaoRuntimeConfig.inertial_overrides``
consumes; ``--merge`` joins it with the other arm's file into
``gravity_both_<date>.json`` so one override file covers both arms.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Sequence
import csv
from dataclasses import asdict, dataclass, field
from datetime import datetime
import json
import math
from pathlib import Path
import sys
from typing import Any

import matplotlib
import numpy as np
from scipy.optimize import least_squares
import typer

from dimos.memory.store.sqlite import SqliteStore
from dimos.robot.manipulators.calibration._inertial import LinkInertial, load_inertial_overrides
from dimos.robot.manipulators.calibration.gravity_poses import (
    GRAVITY_PARAMS_PER_BODY,
    ArmModel,
    check_side,
    measured_joints,
)
from dimos.robot.manipulators.calibration.step_analysis import (
    REST_WINDOW_S,
    rest_window,
    segment_steps,
)
from dimos.robot.manipulators.calibration.step_report import (
    Recording,
    load_recording,
)
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_ARM_JOINTS

matplotlib.use("Agg")
import matplotlib.pyplot as plt

GRAVITY_EVENTS_STREAM = "gravity_events"
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
class Measurement:
    """Resting torque of one joint at one pose, averaged over a +d/-d dither pair."""

    pose: str
    joint: str
    kp: float
    kd: float
    dither_deg: float
    q_rad: tuple[float, ...]  # all arm joints, mean of the two rest windows
    tau_pd_nm: float  # kp (q_target - q) averaged over the pair: gravity-model error
    friction_nm: float  # half the difference: Coulomb friction (>= 0 when the joint settles)

    def as_row(self) -> dict[str, Any]:
        row = asdict(self)
        row["q_deg"] = " ".join(f"{math.degrees(v):.2f}" for v in self.q_rad)
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
    side: str = "left"

    @property
    def bounds_ok(self) -> bool:
        return not self.bound_violations

    def rms_by_joint(self, *, holdout: bool, after: bool) -> dict[str, float]:
        values = self.residual_after if after else self.residual_before
        out: dict[str, float] = {}
        for joint in measured_joints(self.side):
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


def load_gravity_recording(recording_dir: Path) -> GravityRecording:
    """Joint traces and step events (as the step report) plus the pose markers, if any."""
    recording = load_recording(recording_dir)
    store = SqliteStore(path=str(recording_dir / "memory.db"), must_exist=True)
    store.start()
    try:
        markers: list[PoseMarker] = []
        if GRAVITY_EVENTS_STREAM in set(store.list_streams()):
            stream: Any = store.stream(GRAVITY_EVENTS_STREAM)
            markers = [PoseMarker.from_json(obs.data.data) for obs in stream]
    finally:
        store.stop()
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


def _rest_q_rad(recording: Recording, start: float, stop: float) -> np.ndarray | None:
    """Mean of every arm joint over the last ``REST_WINDOW_S`` of ``[start, stop]``, radians."""
    first = recording.traces[DUAL_OPENYAM_ARM_JOINTS[0]]
    mask = (first.t >= start) & (first.t <= stop)
    if mask.sum() < 2:
        return None
    t = first.t[mask]
    rest = rest_window(t, REST_WINDOW_S)
    return np.asarray(
        [
            math.radians(float(recording.traces[name].q_deg[mask][rest].mean()))
            for name in DUAL_OPENYAM_ARM_JOINTS
        ]
    )


def extract_measurements(grec: GravityRecording, side: str = "left") -> list[Measurement]:
    """Pair the +d and -d dithers of each measured joint of ``side`` at each pose."""
    recording = grec.recording
    joints = measured_joints(side)
    windows = pose_windows(grec.markers, recording.t_end)
    # (pose, joint, kp, kd, |delta|) -> sign -> [(tau_pd, q)]
    groups: dict[
        tuple[str, str, float, float, float], dict[int, list[tuple[float, np.ndarray]]]
    ] = defaultdict(lambda: defaultdict(list))
    for event, start, stop in segment_steps(recording.events, recording.t_end):
        if event.joint not in joints:
            continue  # the other arm, or joints 1 and 6 (no usable gravity signal)
        pose = next((label for label, a, b in windows if a <= event.t0 < b), None)
        if pose is None:
            continue
        q = _rest_q_rad(recording, start, stop)
        if q is None:
            continue
        index = DUAL_OPENYAM_ARM_JOINTS.index(event.joint)
        tau_pd = event.kp * (math.radians(event.target_after_deg) - q[index])
        key = (pose, event.joint, event.kp, event.kd, abs(event.delta_deg))
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
                    q_rad=tuple(float(v) for v in 0.5 * (q_plus + q_minus)),
                    tau_pd_nm=0.5 * (tau_plus + tau_minus),
                    friction_nm=0.5 * (tau_plus - tau_minus),
                )
            )
    return measurements


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
                f"{m.joint} is not one of the {arm.side} measured joints {arm.measured_joints}"
            )
        full = arm.scaled_regressor(np.asarray(m.q_rad))
        rows.append(full[arm.measured_joints.index(m.joint)])
    a_scaled = np.vstack(rows)
    b = np.asarray([m.tau_pd_nm for m in measurements])
    a_train = a_scaled[~is_holdout]
    b_train = b[~is_holdout]

    u, s, vt = np.linalg.svd(a_train, full_matrices=False)
    rank = int((s > FIT_RANK_TOLERANCE * s[0]).sum()) if s.size and s[0] > 0 else 0
    if rank == 0:
        raise ValueError("the training poses carry no gravity information (zero regressor)")
    basis = vt[:rank].T  # identifiable directions, scaled parameter space
    condition = float(s[0] / s[rank - 1])
    sigma = s[:rank]

    # Physical parameters: per link (d_mass, com_shift) with hard bounds, so the
    # solution is physical by construction instead of being checked afterwards.
    mass0 = np.array([float(arm.model.inertias[j].mass) for j in arm.fitted_joint_ids])
    com0 = np.array(
        [np.asarray(arm.model.inertias[j].lever, dtype=np.float64) for j in arm.fitted_joint_ids]
    )
    a_phys = a_scaled / arm.column_scale  # torque per unit (m, m*c) correction

    def to_delta_phi(x: np.ndarray) -> np.ndarray:
        d_mass = x[0::4]
        shift = x.reshape(-1, 4)[:, 1:]
        moment = d_mass[:, None] * com0 + (mass0 + d_mass)[:, None] * shift
        return np.concatenate([np.column_stack([d_mass, moment])[k] for k in range(len(mass0))])

    lower = np.concatenate(
        [[(MASS_RATIO_BOUNDS[0] - 1.0) * m, -COM_BOX_M, -COM_BOX_M, -COM_BOX_M] for m in mass0]
    )
    upper = np.concatenate(
        [[(MASS_RATIO_BOUNDS[1] - 1.0) * m, COM_BOX_M, COM_BOX_M, COM_BOX_M] for m in mass0]
    )
    train_poses_arr = np.asarray([m.pose for m in measurements])[~is_holdout]
    a_phys_train = a_phys[~is_holdout]

    def solve(a: np.ndarray, b_: np.ndarray, ridge: float) -> np.ndarray:
        def residual(x: np.ndarray) -> np.ndarray:
            dphi = to_delta_phi(x)
            return np.concatenate([a @ dphi - b_, ridge * dphi / arm.column_scale])

        return least_squares(
            residual, np.zeros(lower.size), bounds=(lower, upper), xtol=1e-12, ftol=1e-12
        ).x

    ridge_cv: dict[float, float] = {}
    if ridge_nm is None:
        # Leave one training pose out at a time; score each ridge on the pose left out.
        for candidate in RIDGE_GRID_NM:
            errors = []
            for pose in sorted(set(train_poses_arr)):
                keep = train_poses_arr != pose
                x = solve(a_phys_train[keep], b_train[keep], candidate)
                errors.append(a_phys_train[~keep] @ to_delta_phi(x) - b_train[~keep])
            ridge_cv[candidate] = float(np.sqrt(np.mean(np.square(np.concatenate(errors)))))
        ridge_nm = min(ridge_cv, key=lambda r: ridge_cv[r])

    x_phys = solve(a_phys_train, b_train, ridge_nm)
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
        mass = mass0 + float(delta[sl][0])
        urdf_links[link] = LinkInertial(mass_kg=mass0, com_m=_vec3(com0))
        if mass <= 0.0:
            violations.append(f"{link}: fitted mass {mass:.3f} kg is not positive")
            continue
        com = (mass0 * com0 + delta[sl][1:]) / mass
        shift = com - com0
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
        singular_values=tuple(float(v) for v in s[:rank]),
        condition_number=condition,
        ridge_nm=ridge_nm,
        bound_violations=violations,
        at_bounds=at_bounds,
        ridge_cv_nm=ridge_cv,
        side=arm.side,
    )


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


def result_json(result: FitResult, recording_dir: Path, created: datetime) -> dict[str, Any]:
    return {
        "created": created.isoformat(timespec="seconds"),
        "recording": recording_dir.name,
        "side": result.side,
        "source": "dimos.robot.manipulators.calibration.gravity_fit",
        "links": {
            name: {"mass_kg": link.mass_kg, "com_m": list(link.com_m)}
            for name, link in result.links.items()
        },
        "urdf_links": {
            name: {"mass_kg": link.mass_kg, "com_m": list(link.com_m)}
            for name, link in result.urdf_links.items()
        },
        "fit": {
            "ridge_nm": result.ridge_nm,
            "rank": result.rank,
            "condition_number": result.condition_number,
            "singular_values": list(result.singular_values),
            "train_poses": result.train_poses,
            "holdout_poses": result.holdout_poses,
            "measurements": len(result.measurements),
            "holdout_rms_nm": {
                joint: {
                    "before": result.rms_by_joint(holdout=True, after=False)[joint],
                    "after": result.rms_by_joint(holdout=True, after=True)[joint],
                }
                for joint in result.rms_by_joint(holdout=True, after=False)
            },
            "train_rms_nm": {
                joint: {
                    "before": result.rms_by_joint(holdout=False, after=False)[joint],
                    "after": result.rms_by_joint(holdout=False, after=True)[joint],
                }
                for joint in result.rms_by_joint(holdout=False, after=False)
            },
            "parameters": [asdict(p) for p in result.parameters],
            "bounds_ok": result.bounds_ok,
            "bound_violations": result.bound_violations,
            "at_bounds": result.at_bounds,
            "ridge_cv_nm": {str(k): v for k, v in result.ridge_cv_nm.items()},
        },
    }


def find_previous_json(
    search_dirs: list[Path], exclude: Path | None = None, side: str = "left"
) -> Path | None:
    """Newest ``gravity_<side>_*.json`` under the given directories (and ``*/gravity_report``).

    Files named ``gravity_<date>.json`` predate the side suffix and are left-arm fits.
    """
    patterns = [f"gravity_{check_side(side)}_*.json"]
    if side == "left":
        patterns.append("gravity_[0-9]*.json")
    found: list[Path] = []
    for base in search_dirs:
        if not base.is_dir():
            continue
        for pattern in patterns:
            found.extend(base.glob(pattern))
            found.extend(base.glob(f"*/gravity_report/{pattern}"))
    found = [p for p in found if exclude is None or p.resolve() != exclude.resolve()]
    if not found:
        return None
    return max(found, key=lambda p: p.stat().st_mtime)


def merge_inertial_json(paths: Sequence[Path], created: datetime) -> dict[str, Any]:
    """One override file from several fits; later files win on a link named twice.

    Each input is a ``gravity_fit`` JSON (or any ``{"links": ...}`` / flat file);
    the result carries every link plus where each input came from.
    """
    links: dict[str, dict[str, Any]] = {}
    sources: list[dict[str, Any]] = []
    for path in paths:
        for name, link in load_inertial_overrides(path).items():
            links[name] = {"mass_kg": link.mass_kg, "com_m": list(link.com_m)}
        raw = json.loads(path.read_text())
        sources.append(
            {
                "path": str(path),
                "side": raw.get("side") if isinstance(raw, dict) else None,
                "created": raw.get("created") if isinstance(raw, dict) else None,
            }
        )
    return {
        "created": created.isoformat(timespec="seconds"),
        "side": "both",
        "source": "dimos.robot.manipulators.calibration.gravity_fit (merge)",
        "merged_from": sources,
        "links": links,
    }


def compare_with_previous(result: FitResult, previous: Path) -> str:
    previous_links = load_inertial_overrides(previous)
    lines = [
        f"## Compared with {previous}",
        "",
        "| link | mass prev (kg) | mass new (kg) | COM shift new - prev (mm) |",
        "|---|---|---|---|",
    ]
    for name, link in result.links.items():
        old = previous_links.get(name)
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


def _rms_table(result: FitResult, *, holdout: bool) -> str:
    before = result.rms_by_joint(holdout=holdout, after=False)
    after = result.rms_by_joint(holdout=holdout, after=True)
    if not before:
        return "(no poses)\n"
    lines = ["| joint | RMS before (Nm) | RMS after (Nm) |", "|---|---|---|"]
    for joint in before:
        lines.append(f"| {joint} | {before[joint]:.3f} | {after[joint]:.3f} |")
    return "\n".join(lines) + "\n"


def format_report(result: FitResult, previous: Path | None, json_path: Path | None) -> str:
    out = [f"# Dual OpenYAM gravity-model fit ({result.side} arm)", ""]
    if result.bounds_ok:
        out.append(
            f"Result: OK, parameters written to `{json_path}`." if json_path else "Result: OK."
        )
    else:
        out.append("Result: FAILED physical bounds; no parameters written.")
        out.extend(f"- {v}" for v in result.bound_violations)
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
        "Resting gravity torque error at poses the fit never saw, before and after the correction."
    )
    out.append("")
    out.append(_rms_table(result, holdout=True))
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
    joints = [
        j for j in measured_joints(result.side) if any(m.joint == j for m in result.measurements)
    ]
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
        f"{result.side} arm: resting gravity torque error per pose (hatched: held out)",
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
    *,
    ridge_nm: float | None = None,
    holdout: list[str] | None = None,
    previous: Path | None = None,
    arm: ArmModel | None = None,
    now: datetime | None = None,
    side: str = "left",
    merge: Path | None = None,
) -> tuple[FitResult, Path]:
    """Fit one arm's recording and write ``gravity_report/`` inside it.

    ``merge`` is the other arm's ``gravity_<side>_<date>.json``; when given and
    the fit passes its bounds, ``gravity_both_<date>.json`` is written too, with
    this fit's links overriding any same-named links in ``merge``.
    """
    arm = arm or ArmModel(side=side)
    if arm.side != side:
        raise ValueError(f"arm model is for the {arm.side} side, not {side}")
    now = now or datetime.now().astimezone()
    grec = load_gravity_recording(recording_dir)
    measurements = extract_measurements(grec, side)
    result = fit_gravity(arm, measurements, ridge_nm=ridge_nm, holdout=holdout)
    out_dir = recording_dir / REPORT_DIR_NAME
    out_dir.mkdir(exist_ok=True)
    json_path = out_dir / f"gravity_{side}_{now.date().isoformat()}.json"
    if previous is None:
        previous = find_previous_json(
            [recording_dir.parent, Path(__file__).resolve().parent], exclude=json_path, side=side
        )
    if result.bounds_ok:
        json_path.write_text(json.dumps(result_json(result, recording_dir, now), indent=2) + "\n")
        if merge is not None:
            merged = merge_inertial_json([merge, json_path], now)
            (out_dir / f"gravity_both_{now.date().isoformat()}.json").write_text(
                json.dumps(merged, indent=2) + "\n"
            )
    else:
        json_path = None  # type: ignore[assignment]
    (out_dir / "report.md").write_text(format_report(result, previous, json_path))
    write_measurements_csv(result, out_dir / "measurements.csv")
    plot_resting_torque(result, out_dir / "resting_torque.png")
    return result, out_dir


app = typer.Typer(add_completion=False)


@app.command()
def main(
    recording_dir: Path = typer.Argument(..., help="recordings/<run-id> with memory.db"),
    ridge_nm: float | None = typer.Option(
        None, help="Ridge weight toward the URDF, Nm; default: chosen by leave-one-pose-out CV"
    ),
    holdout: str = typer.Option(
        "",
        help="Comma-separated pose labels to hold out; default every 5th pose",
    ),
    previous: Path | None = typer.Option(
        None, help="Earlier gravity_<side>_<date>.json to compare with"
    ),
    side: str = typer.Option("left", help="Which arm the recording moved: left or right"),
    merge: Path | None = typer.Option(
        None,
        help="The other arm's gravity_<side>_<date>.json; also writes gravity_both_<date>.json",
    ),
) -> None:
    """Fit the gravity model from a recording and write gravity_report/ next to it."""
    labels = [p.strip() for p in holdout.split(",") if p.strip()] or None
    try:
        check_side(side)
    except ValueError as error:
        raise typer.BadParameter(str(error)) from None
    result, out_dir = generate_gravity_report(
        recording_dir, ridge_nm=ridge_nm, holdout=labels, previous=previous, side=side, merge=merge
    )
    typer.echo((out_dir / "report.md").read_text())
    typer.echo(f"Wrote {out_dir}")
    if merge is not None and result.bounds_ok:
        typer.echo(f"Merged with {merge} into gravity_both_<date>.json for --inertial-overrides")
    if not result.bounds_ok:
        raise typer.Exit(1)


if __name__ == "__main__":
    sys.exit(app())
