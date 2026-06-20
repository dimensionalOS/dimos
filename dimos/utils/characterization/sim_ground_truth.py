# Copyright 2025-2026 Dimensional Inc.
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

"""Sim ground-truth harness for plant-ID validation.

Inject KNOWN FOPDT (K, tau, L) per axis into the vendored ``TwistBasePlantSim``,
simulate a multi-step excitation, and write a recording in the exact format the
live ``CharacterizationRecorder`` produces -- through the memory2 ``SqliteStore``
interface, no raw SQL -- plus a sidecar JSON that reports the injected truth.

Both the existing velocity-domain fitter and the new pose-domain fitter consume
this recording, so recovery error is measurable against a known answer. Pure
offline generation: no hardware, no LCM, no blueprint.

The default measurement-noise constants were measured from the 2026-06-18 local
Go2 session (``data/characterization/go2/go2_recording_*.db``, git-ignored): the
odom x/y residual std over a 92 s zero-command window is ~0.8 mm and the yaw
residual std ~6 mrad. Drift defaults to a 7 cm net stress value -- the worst-case
under-command odom slip seen in that session's floor probes, larger than the
~0.85 cm measured while truly stationary. :func:`calibrate_measurement_from_db`
re-derives all three from any recording.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any, Literal

import numpy as np

from dimos.control.components import make_twist_base_joints
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.utils.benchmarking.plant import (
    GO2_PLANT_FITTED,
    FopdtChannelParams,
    TwistBasePlantParams,
    TwistBasePlantSim,
)

Axis = Literal["vx", "vy", "wz"]
_AXES: tuple[Axis, ...] = ("vx", "vy", "wz")
_AXIS_INDEX: dict[str, int] = {"vx": 0, "vy": 1, "wz": 2}

# Deterministic epoch base for synthetic timestamps (fixed so runs reproduce).
_BASE_TS = 1_700_000_000.0

# Measurement-model defaults measured from the 2026-06-18 local Go2 session.
_GO2_POS_NOISE_STD_M = 0.0008  # 0.8 mm: odom x/y residual std, 92 s stationary window
_GO2_YAW_NOISE_STD_RAD = 0.006  # 6 mrad: odom yaw residual std
_DEFAULT_DRIFT_TOTAL_M = 0.07  # 7 cm net XY drift over the run (under-command slip)

# Gate value the live recorder logs for "operator advances to next step".
_GATE_ADVANCE = 1

# Default per-axis excitation amplitudes (m/s for vx/vy, rad/s for wz).
_DEFAULT_VX_AMPS = (0.3, 0.6)
_DEFAULT_VY_AMPS = (0.3,)
_DEFAULT_WZ_AMPS = (0.5, 1.0)
_DEFAULT_HOLD_S = 4.0
_DEFAULT_SETTLE_S = 3.0

_GO2_JOINT_PREFIX = "go2"
_COORD_FRAME = "coordinator"
_ODOM_FRAME = "odom"

# Phase-1 grid: brackets the brief's fast Go2 regime (tau~0.2, L~0.15) and the
# vendored GO2_PLANT_FITTED regime (tau~0.4-0.6, L~0.05-0.065).
_GRID_TAUS = (0.15, 0.2, 0.3, 0.4, 0.6)
_GRID_LS = (0.05, 0.10, 0.15, 0.20)


@dataclass(frozen=True)
class StepSegment:
    """One commanded excitation step: hold ``amplitude`` on ``axis``, then rest."""

    axis: Axis
    amplitude: float
    hold_s: float = _DEFAULT_HOLD_S
    settle_s: float = _DEFAULT_SETTLE_S


@dataclass(frozen=True)
class MeasurementModel:
    """Odom corruption applied on top of the noiseless simulated pose.

    Defaults are the 2026-06-18 Go2 measured values (see module docstring)."""

    pos_noise_std_m: float = _GO2_POS_NOISE_STD_M
    yaw_noise_std_rad: float = _GO2_YAW_NOISE_STD_RAD
    drift_total_m: float = _DEFAULT_DRIFT_TOTAL_M
    drift_yaw_total_rad: float = 0.0


@dataclass
class GroundTruth:
    """The injected truth a fitter must recover, plus the generation settings."""

    plant: TwistBasePlantParams
    effective_l_s: dict[str, float]  # discretized L actually simulated, per axis
    measurement: MeasurementModel
    seed: int
    robot_id: str
    sim_dt_s: float
    command_rate_hz: float
    odom_rate_hz: float
    duration_s: float
    segments: list[StepSegment]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class GroundTruthRecording:
    """Result of :func:`synthesize_recording` -- where the data and truth landed."""

    db_path: Path
    sidecar_path: Path
    ground_truth: GroundTruth


def multistep_excitation(
    *,
    vx_amps: tuple[float, ...] = _DEFAULT_VX_AMPS,
    vy_amps: tuple[float, ...] = _DEFAULT_VY_AMPS,
    wz_amps: tuple[float, ...] = _DEFAULT_WZ_AMPS,
    hold_s: float = _DEFAULT_HOLD_S,
    settle_s: float = _DEFAULT_SETTLE_S,
) -> list[StepSegment]:
    """A multi-step sweep: each axis held at each amplitude, with rests between.

    Multi-step (rather than a single step) gives the rich excitation Phase 3's
    dead-time estimation needs while staying faithful to how the live sweep
    commands one axis at a time.
    """
    segments: list[StepSegment] = []
    for axis, amps in (("vx", vx_amps), ("vy", vy_amps), ("wz", wz_amps)):
        for amp in amps:
            segments.append(StepSegment(axis, amp, hold_s, settle_s))  # type: ignore[arg-type]
    return segments


def go2_validation_grid(
    *,
    taus: tuple[float, ...] = _GRID_TAUS,
    ls: tuple[float, ...] = _GRID_LS,
    k_vx: float = GO2_PLANT_FITTED.vx.K,
    k_wz: float = GO2_PLANT_FITTED.wz.K,
) -> list[tuple[str, TwistBasePlantParams]]:
    """(label, plant) pairs bracketing both candidate Go2 regimes.

    Used by Phase 2/3 to drive recovery across the whole plausible (tau, L)
    space at many seeds; the default grid is 5x4 = 20 plants.
    """
    grid: list[tuple[str, TwistBasePlantParams]] = []
    for tau in taus:
        for dead_time in ls:
            label = f"tau{tau:g}_L{dead_time:g}"
            plant = TwistBasePlantParams(
                vx=FopdtChannelParams(K=k_vx, tau=tau, L=dead_time),
                vy=FopdtChannelParams(K=k_vx, tau=tau, L=dead_time),
                wz=FopdtChannelParams(K=k_wz, tau=tau, L=dead_time),
            )
            grid.append((label, plant))
    return grid


def effective_deadtime_s(params: FopdtChannelParams, sim_dt_s: float) -> float:
    """Dead time the sim actually realizes for ``params`` at ``sim_dt_s``.

    ``FOPDTChannel`` quantizes L to a whole number of ticks (a delay buffer of
    ``max(1, int(L/dt)+1)`` slots delays by one fewer tick than its length), so
    the simulated L is a truncation of the nominal L. The fitter's recovery
    must be judged against this realized value, not the nominal request.
    """
    n_delay = max(1, int(params.L / sim_dt_s) + 1) - 1
    return n_delay * sim_dt_s


def _command_timeline(
    segments: list[StepSegment], command_rate_hz: float
) -> tuple[np.ndarray, np.ndarray, list[float]]:
    """Build the per-tick command stream and each segment's onset time."""
    dt = 1.0 / command_rate_hz
    times: list[float] = []
    cmds: list[list[float]] = []
    onsets: list[float] = []
    t = 0.0
    for seg in segments:
        onsets.append(t)
        idx = _AXIS_INDEX[seg.axis]
        for _ in range(round(seg.hold_s / dt)):
            row = [0.0, 0.0, 0.0]
            row[idx] = seg.amplitude
            cmds.append(row)
            times.append(t)
            t += dt
        for _ in range(round(seg.settle_s / dt)):
            cmds.append([0.0, 0.0, 0.0])
            times.append(t)
            t += dt
    return np.asarray(times), np.asarray(cmds), onsets


def _simulate_true_pose(
    plant: TwistBasePlantParams,
    t_cmd: np.ndarray,
    cmds: np.ndarray,
    sim_dt_s: float,
    duration_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Run the plant at ``sim_dt_s`` under a zero-order-hold of the command.

    Returns ``(t_sim, pose[N,3])`` of the noiseless world pose (x, y, yaw)
    sampled after each fine step.
    """
    sim = TwistBasePlantSim(plant)
    sim.reset(0.0, 0.0, 0.0, sim_dt_s)
    n = round(duration_s / sim_dt_s)
    t_sim = (np.arange(n) + 1) * sim_dt_s
    cmd_idx = np.clip(np.searchsorted(t_cmd, t_sim, side="right") - 1, 0, len(t_cmd) - 1)
    pose = np.empty((n, 3))
    for i in range(n):
        ci = cmd_idx[i]
        sim.step(float(cmds[ci, 0]), float(cmds[ci, 1]), float(cmds[ci, 2]), sim_dt_s)
        pose[i, 0] = sim.x
        pose[i, 1] = sim.y
        pose[i, 2] = sim.yaw
    return t_sim, pose


def _measure(
    t_sim: np.ndarray,
    pose: np.ndarray,
    sim_dt_s: float,
    odom_rate_hz: float,
    duration_s: float,
    measurement: MeasurementModel,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Decimate the true pose to ``odom_rate_hz`` and corrupt it with the model.

    A constant-velocity drift ramp (fixed random heading, magnitude set so net
    displacement equals ``drift_total_m``) plus per-sample Gaussian noise. The
    drift is the nuisance the pose-domain fitter must separate from K/tau.
    """
    n_odom = int(duration_s * odom_rate_hz)
    t_odom = np.arange(n_odom) / odom_rate_hz
    sim_idx = np.clip(np.round(t_odom / sim_dt_s).astype(int) - 1, 0, len(t_sim) - 1)
    x = pose[sim_idx, 0].copy()
    y = pose[sim_idx, 1].copy()
    yaw = pose[sim_idx, 2].copy()

    drift_heading = rng.uniform(0.0, 2.0 * math.pi)
    drift_speed = measurement.drift_total_m / duration_s
    x += drift_speed * math.cos(drift_heading) * t_odom
    y += drift_speed * math.sin(drift_heading) * t_odom
    if measurement.drift_yaw_total_rad:
        yaw += (measurement.drift_yaw_total_rad / duration_s) * t_odom

    x += rng.normal(0.0, measurement.pos_noise_std_m, n_odom)
    y += rng.normal(0.0, measurement.pos_noise_std_m, n_odom)
    yaw += rng.normal(0.0, measurement.yaw_noise_std_rad, n_odom)
    return t_odom, x, y, yaw


def _write_recording(
    db_path: Path,
    t_cmd: np.ndarray,
    cmds: np.ndarray,
    onsets: list[float],
    t_odom: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    yaw: np.ndarray,
    joint_prefix: str,
) -> None:
    """Write the four streams via the memory2 store -- format-identical to the recorder.

    ``odom`` and ``joint_state`` carry the same decimated pose: on real Go2 the
    298 Hz joint_state is just the ~16 Hz pose held at high rate (no extra
    information), so emitting both at ``odom_rate`` is faithful, not lossy. The
    old velocity-domain fitter reads pose from ``joint_state`` (names
    ``go2/{vx,vy,wz}``, position ``[x, y, yaw]``); the new fitter reads ``odom``.
    """
    joints = make_twist_base_joints(joint_prefix)
    store = SqliteStore(path=str(db_path))
    store.start()
    try:
        cmd_stream = store.stream("cmd_vel", Twist)
        odom_stream = store.stream("odom", PoseStamped)
        joint_stream = store.stream("joint_state", JointState)
        gate_stream = store.stream("gate", Int8)

        for ti, row in zip(t_cmd, cmds, strict=True):
            twist = Twist([float(row[0]), float(row[1]), 0.0], [0.0, 0.0, float(row[2])])
            cmd_stream.append(twist, ts=_BASE_TS + float(ti), pose=None)

        for ti, xx, yy, yw in zip(t_odom, x, y, yaw, strict=True):
            ts = _BASE_TS + float(ti)
            orientation = Quaternion.from_euler(Vector3(0.0, 0.0, float(yw)))
            odom_stream.append(
                PoseStamped(
                    ts=ts,
                    frame_id=_ODOM_FRAME,
                    position=[float(xx), float(yy), 0.0],
                    orientation=orientation,
                ),
                ts=ts,
                pose=None,
            )
            joint_stream.append(
                JointState(
                    ts=ts,
                    frame_id=_COORD_FRAME,
                    name=joints,
                    position=[float(xx), float(yy), float(yw)],
                    velocity=[0.0, 0.0, 0.0],
                    effort=[0.0, 0.0, 0.0],
                ),
                ts=ts,
                pose=None,
            )

        for onset in onsets:
            gate_stream.append(Int8(_GATE_ADVANCE), ts=_BASE_TS + onset, pose=None)
    finally:
        store.stop()


def synthesize_recording(
    plant: TwistBasePlantParams,
    *,
    db_path: str | Path,
    segments: list[StepSegment] | None = None,
    robot_id: str = "go2_u01",
    command_rate_hz: float = 10.0,
    odom_rate_hz: float = 18.0,
    sim_dt_s: float = 0.001,
    measurement: MeasurementModel | None = None,
    seed: int = 0,
    joint_prefix: str = _GO2_JOINT_PREFIX,
) -> GroundTruthRecording:
    """Generate one sim recording with KNOWN dynamics + a ground-truth sidecar.

    Args:
        plant: injected FOPDT (K, tau, L) per axis -- the answer a fitter recovers.
        db_path: where to write the ``.db`` (parent dirs created).
        segments: excitation; defaults to :func:`multistep_excitation`.
        robot_id: per-unit id encoded into the sidecar (e.g. ``go2_u01``).
        command_rate_hz: rate the cmd_vel stream is written at.
        odom_rate_hz: rate the odom/joint_state streams are sampled at (Go2 ~18 Hz).
        sim_dt_s: fine integration step; sets the realized dead-time resolution.
        measurement: noise/drift model; defaults to the measured Go2 values.
        seed: RNG seed -- same seed reproduces the recording bit-for-bit.
        joint_prefix: hardware id for joint names (``go2`` -> ``go2/vx`` ...).

    Returns:
        Paths to the recording and sidecar, plus the :class:`GroundTruth`.
    """
    segments = segments if segments is not None else multistep_excitation()
    measurement = measurement if measurement is not None else MeasurementModel()
    db_path = Path(db_path)
    db_path.parent.mkdir(parents=True, exist_ok=True)
    # Write a FRESH recording each call -- appending to an existing db collides on
    # the per-stream UNIQUE ts. Clear the db and its WAL/SHM sidecars first.
    for stale in (
        db_path,
        db_path.with_suffix(db_path.suffix + "-wal"),
        db_path.with_suffix(db_path.suffix + "-shm"),
    ):
        stale.unlink(missing_ok=True)
    rng = np.random.default_rng(seed)

    t_cmd, cmds, onsets = _command_timeline(segments, command_rate_hz)
    duration_s = float(t_cmd[-1]) + 1.0 / command_rate_hz
    t_sim, pose = _simulate_true_pose(plant, t_cmd, cmds, sim_dt_s, duration_s)
    t_odom, x, y, yaw = _measure(t_sim, pose, sim_dt_s, odom_rate_hz, duration_s, measurement, rng)

    _write_recording(db_path, t_cmd, cmds, onsets, t_odom, x, y, yaw, joint_prefix)

    ground_truth = GroundTruth(
        plant=plant,
        effective_l_s={a: effective_deadtime_s(getattr(plant, a), sim_dt_s) for a in _AXES},
        measurement=measurement,
        seed=seed,
        robot_id=robot_id,
        sim_dt_s=sim_dt_s,
        command_rate_hz=command_rate_hz,
        odom_rate_hz=odom_rate_hz,
        duration_s=duration_s,
        segments=list(segments),
    )
    sidecar_path = db_path.with_name(db_path.stem + "_ground_truth.json")
    sidecar_path.write_text(json.dumps(ground_truth.to_dict(), indent=2))
    return GroundTruthRecording(db_path, sidecar_path, ground_truth)


def calibrate_measurement_from_db(
    db_path: str | Path,
    *,
    warmup_s: float = 2.0,
    min_window_s: float = 10.0,
) -> MeasurementModel:
    """Re-derive a :class:`MeasurementModel` from a recording's stationary window.

    Reads the leading zero-command segment of ``db_path`` through the memory2
    store, linearly detrends odom x/y/yaw, and returns the residual stds as
    noise and the net XY displacement over the window as ``drift_total_m``. This
    is how the baked-in Go2 defaults were obtained; rerun it on fresh data to
    refresh them.
    """
    store = SqliteStore(path=str(db_path))
    store.start()
    try:
        cmd = [
            (obs.ts, obs.data.linear.x, obs.data.linear.y, obs.data.angular.z)
            for obs in store.stream("cmd_vel", Twist)
        ]
        odom = [
            (obs.ts, obs.data.x, obs.data.y, obs.data.yaw)
            for obs in store.stream("odom", PoseStamped)
        ]
    finally:
        store.stop()
    if not cmd or not odom:
        raise ValueError(f"{db_path}: needs both cmd_vel and odom streams")

    nonzero = [c[0] for c in cmd if abs(c[1]) + abs(c[2]) + abs(c[3]) > 1e-9]
    first_motion = nonzero[0] if nonzero else cmd[-1][0]
    odom_arr = np.asarray(odom)
    start = odom_arr[0, 0]
    window = odom_arr[(odom_arr[:, 0] >= start + warmup_s) & (odom_arr[:, 0] < first_motion)]
    if len(window) < 2 or (window[-1, 0] - window[0, 0]) < min_window_s:
        raise ValueError(
            f"{db_path}: stationary window too short to calibrate "
            f"(need >= {min_window_s}s of zero-command odom)"
        )

    rel_t = window[:, 0] - window[0, 0]
    design = np.vstack([rel_t, np.ones_like(rel_t)]).T
    pos_residual_stds: list[float] = []
    for col in (1, 2):  # x, y
        slope_intercept, *_ = np.linalg.lstsq(design, window[:, col], rcond=None)
        residual = window[:, col] - design @ slope_intercept
        pos_residual_stds.append(float(np.std(residual)))
    yaw_unwrapped = np.unwrap(window[:, 3])
    yaw_fit, *_ = np.linalg.lstsq(design, yaw_unwrapped, rcond=None)
    yaw_residual = yaw_unwrapped - design @ yaw_fit
    net_drift = float(np.hypot(window[-1, 1] - window[0, 1], window[-1, 2] - window[0, 2]))

    return MeasurementModel(
        pos_noise_std_m=float(np.mean(pos_residual_stds)),
        yaw_noise_std_rad=float(np.std(yaw_residual)),
        drift_total_m=net_drift,
    )


__all__ = [
    "GroundTruth",
    "GroundTruthRecording",
    "MeasurementModel",
    "StepSegment",
    "calibrate_measurement_from_db",
    "effective_deadtime_s",
    "go2_validation_grid",
    "multistep_excitation",
    "synthesize_recording",
]
