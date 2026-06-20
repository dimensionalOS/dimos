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

from __future__ import annotations

import argparse
from collections.abc import Callable, Iterator
from dataclasses import asdict
from datetime import date
import math
from pathlib import Path
import queue
import threading
import time
from typing import Any, Literal

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from reactivex.disposable import Disposable

from dimos.control.components import make_twist_base_joints
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.robot.unitree.keyboard_teleop import GATE_ADVANCE, GATE_QUIT, GATE_SKIP
from dimos.utils.path_utils import get_project_root

_CMD_VEL_TOPIC = "/cmd_vel"
_JOINT_STATE_TOPIC = "/coordinator/joint_state"
from dimos.utils.benchmarking.plant import (
    ROBOT_PLANT_PROFILES,
    ChannelEnvelope,
    FopdtChannelParams,
    RobotPlantProfile,
    TwistBasePlantParams,
    TwistBasePlantSim,
    VelocityEnvelope,
)
from dimos.utils.benchmarking.tuning import (
    AmplitudeFitDC,
    ChannelEnvelopeDC,
    DynamicsByAmplitude,
    FloorProbeResultDC,
    FloorProbeResults,
    Provenance,
    TuningConfig,
    VelocityEnvelopeDC,
    _floor_from_probe,
    _output_ceiling,
    _saturating_at_amp,
    derive_config,
    git_sha,
    re_derive_config,
)
from dimos.utils.characterization.modeling.fopdt import fit_fopdt, fopdt_step_response

_CHANNELS = ("vx", "vy", "wz")
_SIM_DT = 0.02  # in-process self-test integration step (not robot-specific)
_FLOOR_CAP_EPS = 1e-9  # float-rounding guard so a cap on a step boundary is probed

REPORTS_DIR = Path(__file__).parent / "reports"
DEFAULT_OUT_DIR = get_project_root() / "data" / "characterization"


def reconstruct_body_velocities(
    ts: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    yaw: np.ndarray,
    window: int = 5,
    order: int = 2,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    from scipy.signal import savgol_filter

    ts = np.asarray(ts, dtype=float)
    if len(ts) >= 2:
        keep = np.ones(len(ts), dtype=bool)
        last = ts[0]
        for i in range(1, len(ts)):
            if ts[i] - last < 0.005:
                keep[i] = False
            else:
                last = ts[i]
        ts = ts[keep]
        x = np.asarray(x, dtype=float)[keep]
        y = np.asarray(y, dtype=float)[keep]
        yaw = np.asarray(yaw, dtype=float)[keep]

    yaw_u = np.unwrap(yaw)
    if len(ts) >= window and window % 2 == 1 and order < window:
        xf = savgol_filter(x, window, order)
        yf = savgol_filter(y, window, order)
        yawf = savgol_filter(yaw_u, window, order)
    else:
        xf, yf, yawf = x, y, yaw_u
    dx = np.gradient(xf, ts)
    dy = np.gradient(yf, ts)
    dyaw = np.gradient(yawf, ts)
    c, s = np.cos(yawf), np.sin(yawf)
    vx = c * dx + s * dy
    vy = -s * dx + c * dy
    return ts, vx, vy, dyaw


def _hampel(ys: np.ndarray, window: int = 11, n_sigma: float = 3.0) -> tuple[np.ndarray, int]:
    if window <= 0 or len(ys) < window:
        return ys.copy(), 0
    half = window // 2
    n = len(ys)
    out = ys.copy()
    replaced = 0
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        w = ys[lo:hi]
        med = float(np.median(w))
        mad = float(np.median(np.abs(w - med)))
        if mad == 0.0:
            continue
        if abs(ys[i] - med) > n_sigma * 1.4826 * mad:
            out[i] = med
            replaced += 1
    return out, replaced


def _physical_clip(ys: np.ndarray, max_abs: float) -> tuple[np.ndarray, int]:
    out = ys.copy()
    bad = np.abs(out) > max_abs
    n = int(bad.sum())
    if n == 0:
        return out, 0
    good_idx = np.where(~bad)[0]
    if len(good_idx) == 0:
        out[:] = 0.0
        return out, n
    for i in np.where(bad)[0]:
        lo = max(0, i - 10)
        hi = min(len(out), i + 11)
        local_good = out[lo:hi][~bad[lo:hi]]
        if len(local_good) > 0:
            out[i] = float(np.median(local_good))
        else:
            out[i] = float(np.median(out[good_idx]))
    return out, n


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _d3_sustained_pass(
    ts: np.ndarray,
    ys: np.ndarray,
    amp: float,
    motion_threshold: float,
    fractional_threshold: float,
    sustained: int,
    displacement_threshold: float,
) -> tuple[bool, int, float]:
    """D3 floor gate: a probe "moved" iff it genuinely translated.

    AND of two tests on the signed body-frame velocity ``ys`` (sampled at
    ``ts``), commanded at amplitude ``amp``:

    1. NET signed displacement in the commanded direction exceeds
       ``displacement_threshold`` (kills net-zero posture wobble whose
       ``|v|`` spikes but whose integral cancels).
    2. Signed velocity (in the commanded sign) sustains the motion+fractional
       thresholds for ``sustained`` consecutive samples.

    Returns ``(passed, longest_run, net_displacement_in_cmd_dir)``.
    """
    if len(ys) == 0 or amp == 0.0:
        return False, 0, 0.0
    cmd_sign = math.copysign(1.0, amp)
    net = float(np.trapezoid(ys, ts)) if len(ts) == len(ys) and len(ts) > 1 else 0.0
    net_in_cmd_dir = cmd_sign * net
    pass_displacement = net_in_cmd_dir >= displacement_threshold

    signed = cmd_sign * np.asarray(ys, dtype=float)
    pass_mask = (signed > motion_threshold) & (signed > fractional_threshold * abs(amp))
    longest = 0
    cur = 0
    pass_sustained = False
    for v in pass_mask:
        if v:
            cur += 1
            if cur > longest:
                longest = cur
            if cur >= sustained:
                pass_sustained = True
        else:
            cur = 0
    return (pass_displacement and pass_sustained), longest, net_in_cmd_dir


def _pick_linear_regime_fit(fits: list[dict], r2_floor: float = 0.9) -> dict | None:
    if not fits:
        return None
    candidates = [f for f in fits if f.get("r2", 0.0) >= r2_floor]
    if candidates:
        return min(candidates, key=lambda f: abs(f["amp"]))
    return max(fits, key=lambda f: f.get("r2", 0.0))


def _channel_cap(profile: RobotPlantProfile, channel: str) -> float:
    return profile.wz_max if channel == "wz" else profile.vx_max


def _floor_candidate_amplitudes(
    profile: RobotPlantProfile, channel: str
) -> Iterator[tuple[float, bool]]:
    """Yield ascending ``(amp, is_extension)``: first the predefined ladder,
    then ``last + step`` until the per-channel cap (inclusive). ``is_extension``
    flags amplitudes generated past the predefined list (for logging)."""
    ladder = sorted(profile.floor_probe_amplitudes.get(channel, []))
    for amp in ladder:
        yield amp, False
    step = profile.floor_probe_step.get(channel, 0.0)
    cap = profile.floor_probe_max.get(channel, 0.0)
    if step <= 0.0:
        return
    amp = (ladder[-1] if ladder else 0.0) + step
    while amp <= cap + _FLOOR_CAP_EPS:
        yield amp, True
        amp += step


# Spacing used only when the discovered floor lands at/above the predefined
# sweep top, so the floor-anchored ladder still spans a usable range.
_FLOOR_ANCHORED_MIN_STEP = 0.1


def _floor_anchored_sweep(profile: RobotPlantProfile, channel: str, floor: float) -> list[float]:
    """Dynamics-sweep amplitudes anchored at the discovered ``floor``.

    The floor probe finds the smallest commanded amplitude that produces real
    motion; the sweep then climbs from there to the predefined top, keeping the
    same number of points. Starting at the floor (rather than a fixed ladder that
    can sit *below* it) guarantees every sweep step actually moves the robot --
    the failure that left vy under-excited when its floor (~0.25) exceeded the
    first static sweep amplitude (0.2).
    """
    static = sorted(profile.si_amplitudes.get(channel, []))
    if not static:
        return []
    n = len(static)
    top = max(static[-1], floor)
    if top <= floor + _FLOOR_CAP_EPS:
        top = floor + (n - 1) * _FLOOR_ANCHORED_MIN_STEP
    if n == 1:
        return [float(floor)]
    return [float(a) for a in np.linspace(floor, top, n)]


def _envelope_from_channel_results(
    floor_probe: list[dict],
    sweep_fits: list[dict],
    probe_fits: list[dict],
    profile: RobotPlantProfile,
    channel: str,
) -> ChannelEnvelope:
    cap = _channel_cap(profile, channel)
    probed_amps = [row["amp"] for row in floor_probe]
    floor, floor_nf = _floor_from_probe(floor_probe, probed_amps)
    all_fits = sweep_fits + probe_fits
    ceiling, ceiling_nf = _output_ceiling(all_fits, cap)
    linear = _pick_linear_regime_fit(sweep_fits or all_fits)
    K_linear = linear["K"] if linear is not None else 0.0
    sat = _saturating_at_amp(all_fits, K_linear, profile.ceiling_k_sag_threshold)
    return ChannelEnvelope(
        floor=float(floor),
        ceiling=float(ceiling),
        floor_not_found=floor_nf,
        ceiling_not_found=ceiling_nf,
        saturating_at_amp=sat,
    )


def _resolve_profile(name: str) -> RobotPlantProfile:
    try:
        return ROBOT_PLANT_PROFILES[name]
    except KeyError:
        raise SystemExit(
            f"unknown --robot {name!r}; known: {sorted(ROBOT_PLANT_PROFILES)}"
        ) from None


def _selftest_step(
    plant: TwistBasePlantSim, channel: str, amp: float, pre_roll_s: float, step_s: float
) -> tuple[np.ndarray, np.ndarray]:
    plant.reset(0.0, 0.0, 0.0, _SIM_DT)
    n_pre = int(pre_roll_s / _SIM_DT)
    n_step = int(step_s / _SIM_DT)
    cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    for _ in range(n_pre):
        plant.step(cmd["vx"], cmd["vy"], cmd["wz"], _SIM_DT)
    cmd[channel] = amp
    ys = []
    for _ in range(n_step):
        plant.step(cmd["vx"], cmd["vy"], cmd["wz"], _SIM_DT)
        ys.append(getattr(plant, channel))
    t = np.arange(len(ys), dtype=float) * _SIM_DT
    return t, np.asarray(ys, dtype=float)


def _fit_selftest(
    profile: RobotPlantProfile,
) -> tuple[
    TwistBasePlantParams,
    dict[str, list[dict]],
    list[dict],
    VelocityEnvelope,
    dict[str, list[dict]],
]:
    truth = profile.sim_plant
    plant = TwistBasePlantSim(truth)
    canonical: dict[str, FopdtChannelParams] = {}
    per_amplitude: dict[str, list[dict]] = {}
    floor_results: dict[str, list[dict]] = {}
    env_channels: dict[str, ChannelEnvelope] = {}
    traces: list[dict] = []

    def fit_and_record(channel: str, amp: float, source: str) -> None:
        t, ys = _selftest_step(plant, channel, amp, profile.pre_roll_s, profile.step_s)
        fp = fit_fopdt(t, ys, u_step=amp)
        if not fp.converged or not np.isfinite([fp.K, fp.tau, fp.L]).all():
            print(f"  [warn] {channel}@{amp} ({source}): fit failed ({fp.reason})")
            return
        row = {
            "amp": amp,
            "amplitude": amp,
            "direction": "forward",
            "K": fp.K,
            "tau": fp.tau,
            "L": fp.L,
            "r2": fp.r_squared,
            "source": source,
        }
        per_amplitude[channel].append(row)
        traces.append({"channel": channel, "t": t, "y": ys, **row})

    for channel in _CHANNELS:
        floor_results[channel] = []
        for amp, _is_ext in _floor_candidate_amplitudes(profile, channel):
            t, ys = _selftest_step(plant, channel, amp, profile.pre_roll_s, profile.step_s)
            passed, longest, net_disp = _d3_sustained_pass(
                t,
                ys,
                amp,
                profile.floor_motion_threshold,
                profile.floor_fractional_threshold,
                profile.floor_sustained_samples,
                profile.floor_displacement_threshold.get(channel, 0.0),
            )
            floor_results[channel].append(
                {
                    "amp": amp,
                    "motion_detected": passed,
                    "sustained_samples": longest,
                    "net_displacement": net_disp,
                }
            )
            if passed:
                break

        per_amplitude[channel] = []
        floor, _ = _floor_from_probe(
            floor_results[channel], [r["amp"] for r in floor_results[channel]]
        )
        for amp in _floor_anchored_sweep(profile, channel, floor):
            fit_and_record(channel, amp, "sweep")
        for amp in profile.ceiling_probe_amplitudes.get(channel, []):
            fit_and_record(channel, amp, "ceiling_probe")

        if not per_amplitude[channel]:
            raise RuntimeError(f"self-test: no converged fits for {channel!r}")
        sweep_fits = [f for f in per_amplitude[channel] if f["source"] == "sweep"]
        probe_fits = [f for f in per_amplitude[channel] if f["source"] == "ceiling_probe"]
        env_channels[channel] = _envelope_from_channel_results(
            floor_results[channel], sweep_fits, probe_fits, profile, channel
        )
        linear = _pick_linear_regime_fit(sweep_fits) or sweep_fits[0]
        canonical[channel] = FopdtChannelParams(
            K=float(linear["K"]), tau=float(linear["tau"]), L=float(linear["L"])
        )

    fitted = TwistBasePlantParams(vx=canonical["vx"], vy=canonical["vy"], wz=canonical["wz"])
    envelope = VelocityEnvelope(vx=env_channels["vx"], vy=env_channels["vy"], wz=env_channels["wz"])
    print("\nself-test (canonical recovered vs injected FOPDT ground truth):")
    print(f"  {'chan':4} {'K fit/true':>20} {'tau fit/true':>20} {'L fit/true':>20}")
    for ch in _CHANNELS:
        f, g = getattr(fitted, ch), getattr(truth, ch)
        print(
            f"  {ch:4} {f.K:8.3f}/{g.K:<8.3f}   {f.tau:8.3f}/{g.tau:<8.3f}   {f.L:8.3f}/{g.L:<8.3f}"
        )
    print("self-test envelope (linear plant => ceiling clamps to platform cap):")
    for ch in _CHANNELS:
        e = getattr(envelope, ch)
        print(
            f"  {ch:4} floor={e.floor:.3f} (nf={e.floor_not_found}) "
            f"ceiling={e.ceiling:.3f} (nf={e.ceiling_not_found})"
        )
    return fitted, per_amplitude, traces, envelope, floor_results


def _plot_fits(
    traces: list[dict],
    provenance: Provenance,
    profile: RobotPlantProfile,
    out: Path,
    envelope: VelocityEnvelope | VelocityEnvelopeDC | None = None,
    dynamics: DynamicsByAmplitude | None = None,
    *,
    kind: Literal["envelope", "steps", "both"] = "envelope",
) -> None:
    if kind == "steps":
        if traces:
            _plot_step_responses(traces, provenance, profile, out)
        return
    fits_by_ch: dict[str, list[dict]] = {}
    if dynamics is not None:
        for ch in ("vx", "vy", "wz"):
            rows = getattr(dynamics, ch, []) or []
            fits_by_ch[ch] = [
                {
                    "amp": r.amp,
                    "K": r.K,
                    "tau": r.tau,
                    "L": r.L,
                    "r2": getattr(r, "r2", 0.0),
                    "source": getattr(r, "source", "sweep"),
                }
                for r in rows
            ]
    else:
        for tr in traces:
            fits_by_ch.setdefault(tr["channel"], []).append(
                {
                    "amp": tr["amp"],
                    "K": tr["K"],
                    "tau": tr["tau"],
                    "L": tr["L"],
                    "r2": tr.get("r2", 0.0),
                    "source": tr.get("source", "sweep"),
                }
            )

    channels = list(dict.fromkeys(c for c in ("vx", "vy", "wz") if fits_by_ch.get(c)))
    if not channels:
        return

    n_rows = 3 + (1 if kind == "both" and traces else 0)
    n_cols = len(channels)
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(5.5 * n_cols, 3.5 * n_rows), squeeze=False)
    row = 0

    if kind == "both" and traces:
        for ax, ch in zip(axes[row], channels, strict=True):
            _plot_step_subplot(ax, ch, [t for t in traces if t["channel"] == ch])
        row += 1

    for ax, ch in zip(axes[row], channels, strict=True):
        _plot_amp_metric(
            ax,
            ch,
            fits_by_ch[ch],
            envelope,
            ylabel="K  =  output / commanded",
            value_fn=lambda r: r["K"],
            title_suffix="K(amp)",
        )
    row += 1

    for ax, ch in zip(axes[row], channels, strict=True):
        unit = "rad/s" if ch == "wz" else "m/s"
        _plot_amp_metric(
            ax,
            ch,
            fits_by_ch[ch],
            envelope,
            ylabel=f"output  =  K·amp ({unit})",
            value_fn=lambda r: r["K"] * r["amp"],
            title_suffix="output (steady-state)",
        )
    row += 1

    for ax, ch in zip(axes[row], channels, strict=True):
        _plot_amp_metric(
            ax,
            ch,
            fits_by_ch[ch],
            envelope,
            ylabel="τ  (s)",
            value_fn=lambda r: r["tau"],
            title_suffix="τ(amp)",
            mark_envelope=False,
        )

    p = provenance
    fig.suptitle(
        f"{profile.name} FOPDT characterization — {p.robot_id} / {p.surface} / "
        f"{p.mode} / {p.sim_or_hw} — {p.date} ({p.git_sha}) — "
        f"methodology v{getattr(p, 'methodology_version', 1)}"
    )
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def _plot_step_subplot(ax, channel: str, ch_traces: list[dict]) -> None:
    for tr in ch_traces:
        t_arr = tr["t"]
        (line,) = ax.plot(t_arr, tr["y"], lw=1.4, alpha=0.85, label=f"meas @{tr['amp']:g}")
        y_raw = tr.get("y_raw")
        if y_raw is not None and tr.get("n_replaced", 0) > 0:
            ax.plot(t_arr, y_raw, ":", lw=0.9, color=line.get_color(), alpha=0.5)
        yhat = fopdt_step_response(t_arr, tr["K"], tr["tau"], tr["L"], tr["amp"])
        ax.plot(t_arr, yhat, "--", lw=1.4, color=line.get_color(), alpha=0.9)
    unit = "rad/s" if channel == "wz" else "m/s"
    ax.set_title(f"{channel} step response  (solid=meas, dashed=fit)")
    ax.set_xlabel("time since step edge (s)")
    ax.set_ylabel(f"{channel} ({unit})")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=7)


def _plot_step_responses(
    traces: list[dict],
    provenance: Provenance,
    profile: RobotPlantProfile,
    out: Path,
) -> None:
    channels = list(dict.fromkeys(t["channel"] for t in traces))
    if not channels:
        return
    amps = sorted({float(t["amp"]) for t in traces})
    by_key = {(t["channel"], float(t["amp"])): t for t in traces}

    fig, axes = plt.subplots(
        len(amps),
        len(channels),
        figsize=(4.8 * len(channels), 2.6 * len(amps)),
        squeeze=False,
    )
    for r, amp in enumerate(amps):
        for c, ch in enumerate(channels):
            ax = axes[r][c]
            tr = by_key.get((ch, amp))
            if tr is None:
                ax.set_axis_off()
                ax.set_title(f"{ch} @ {amp:g}  (no data)", fontsize=9)
                continue
            _plot_single_step(ax, ch, tr)
    p = provenance
    fig.suptitle(
        f"{profile.name} step responses (per amp) — {p.robot_id} / {p.surface} / "
        f"{p.mode} / {p.sim_or_hw} — {p.date} ({p.git_sha})",
        y=1.00,
    )
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def _plot_single_step(ax, channel: str, tr: dict) -> None:
    t_arr = tr["t"]
    ax.plot(t_arr, tr["y"], "-", color="tab:blue", lw=1.4, label="measured")
    y_raw = tr.get("y_raw")
    if y_raw is not None and tr.get("n_replaced", 0) > 0:
        ax.plot(t_arr, y_raw, ":", color="tab:gray", lw=0.9, alpha=0.7, label="raw")
    yhat = fopdt_step_response(t_arr, tr["K"], tr["tau"], tr["L"], tr["amp"])
    ax.plot(t_arr, yhat, "--", color="tab:red", lw=1.3, alpha=0.9, label="FOPDT fit")

    src = tr.get("source", "sweep")
    src_tag = "ceiling" if src == "ceiling_probe" else "sweep"
    ax.set_title(f"{channel} @ {tr['amp']:g}  ({src_tag})", fontsize=9)
    unit = "rad/s" if channel == "wz" else "m/s"
    ax.set_xlabel("t (s)", fontsize=8)
    ax.set_ylabel(f"{channel} ({unit})", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(True, alpha=0.3)
    ax.annotate(
        f"K={tr['K']:.3f}  τ={tr['tau']:.3f}  L={tr['L']:.3f}  r²={tr.get('r2', 0.0):.2f}"
        + (f"  hampel:{tr['n_replaced']}" if tr.get("n_replaced", 0) > 0 else ""),
        xy=(0.02, 0.97),
        xycoords="axes fraction",
        ha="left",
        va="top",
        fontsize=7,
    )
    ax.legend(loc="lower right", fontsize=6)


def _plot_amp_metric(
    ax,
    channel: str,
    rows: list[dict],
    envelope: VelocityEnvelope | VelocityEnvelopeDC | None,
    *,
    ylabel: str,
    value_fn,
    title_suffix: str,
    mark_envelope: bool = True,
) -> None:
    if not rows:
        ax.set_title(f"{channel}: {title_suffix} (no data)")
        return
    rows = sorted(rows, key=lambda r: r["amp"])
    sweep = [r for r in rows if r.get("source", "sweep") == "sweep"]
    probe = [r for r in rows if r.get("source") == "ceiling_probe"]
    if sweep:
        xs = [r["amp"] for r in sweep]
        ys = [value_fn(r) for r in sweep]
        ax.plot(xs, ys, "o-", color="tab:blue", lw=1.4, label="sweep")
    if probe:
        xs = [r["amp"] for r in probe]
        ys = [value_fn(r) for r in probe]
        ax.plot(xs, ys, "s--", color="tab:orange", lw=1.4, label="ceiling probe")

    unit = "rad/s" if channel == "wz" else "m/s"
    if mark_envelope and envelope is not None:
        ce = getattr(envelope, channel, None)
        if ce is not None:
            ax.axvline(
                ce.floor,
                color="green",
                lw=1.0,
                ls=":",
                label=f"floor = {ce.floor:.3g} {unit}" + (" *" if ce.floor_not_found else ""),
            )
            ax.axvline(
                ce.ceiling,
                color="red",
                lw=1.0,
                ls=":",
                label=f"ceiling = {ce.ceiling:.3g} {unit}" + (" *" if ce.ceiling_not_found else ""),
            )

    ax.set_xlabel(f"commanded amplitude ({unit})")
    ax.set_ylabel(ylabel)
    ax.set_title(f"{channel}: {title_suffix}")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=7)


class _JointStatePoseStream:
    def __init__(self, joint_names: list[str]) -> None:
        self._jx, self._jy, self._jyaw = joint_names
        self._lock = threading.Lock()
        self._pose: PoseStamped | None = None
        self._pose_t: float = 0.0
        self._buffer: list[tuple[float, float, float, float]] = []
        self._buffering: bool = False

    def on_joint_state(self, msg: JointState) -> None:
        if not msg.name:
            return
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            x = float(msg.position[idx[self._jx]])
            y = float(msg.position[idx[self._jy]])
            yaw = float(msg.position[idx[self._jyaw]])
        except (KeyError, IndexError):
            return
        now = time.perf_counter()
        pose = PoseStamped(
            ts=now,
            position=Vector3(x, y, 0.0),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
        )
        with self._lock:
            self._pose, self._pose_t = pose, now
            if self._buffering:
                if not self._buffer or now - self._buffer[-1][0] >= 0.005:
                    self._buffer.append((now, x, y, yaw))

    def latest(self) -> tuple[PoseStamped | None, float]:
        with self._lock:
            return self._pose, self._pose_t

    def start_buffering(self) -> None:
        with self._lock:
            self._buffer = []
            self._buffering = True

    def stop_and_pop(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with self._lock:
            self._buffering = False
            buf = self._buffer
            self._buffer = []
        if not buf:
            empty = np.array([], dtype=float)
            return empty, empty, empty, empty
        arr = np.asarray(buf, dtype=float)
        return arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]


def _prereq_banner(profile: RobotPlantProfile) -> None:
    print(
        f"\n=== HARDWARE MODE ({profile.name}) ===\n"
        "Prereqs:\n"
        f"  1. Another terminal: `dimos run {profile.blueprint}`\n"
        f"     (its ControlCoordinator listens on {_CMD_VEL_TOPIC} and\n"
        f"     publishes {_JOINT_STATE_TOPIC} with positions=[x,y,yaw]).\n"
        "     If it includes a keyboard teleop it must be\n"
        "     publish-only-when-active so it does not fight the SI cmds.\n"
        "  2. This process: strip /nix/store from LD_LIBRARY_PATH (README).\n"
        "Robot is STOPPED before every step. Reposition it, then press\n"
        "ENTER here — the tool owns the cmd topic for the step. Each step\n"
        "ends at --max-dist travelled or --step-s, whichever first.\n"
        "Velocity clamped; zero-Twist on exit / Ctrl-C.\n"
    )


def _fit_hw(
    profile: RobotPlantProfile,
    step_s: float,
    pre_roll_s: float,
    warmup_s: float,
    max_dist: float,
    gate_input: Callable[[str], str] = input,
    gate_keys_label: str = "ENTER=run  s=skip  q=quit",
    savgol_window: int = 11,
    savgol_order: int = 2,
    hampel_window: int = 11,
    hampel_n_sigma: float = 3.0,
) -> tuple[
    TwistBasePlantParams,
    dict[str, list[dict]],
    list[dict],
    VelocityEnvelope,
    dict[str, list[dict]],
]:
    _prereq_banner(profile)
    hw_dt = 1.0 / profile.tick_rate_hz

    cmd_pub = LCMTransport(_CMD_VEL_TOPIC, Twist)

    def publish(vx: float, vy: float, wz: float) -> None:
        cmd_pub.broadcast(
            None,
            Twist(
                linear=Vector3(
                    _clamp(vx, -profile.vx_max, profile.vx_max),
                    _clamp(vy, -profile.vx_max, profile.vx_max),
                    0.0,
                ),
                angular=Vector3(0.0, 0.0, _clamp(wz, -profile.wz_max, profile.wz_max)),
            ),
        )

    def safe_stop() -> None:
        for _ in range(3):
            publish(0.0, 0.0, 0.0)
            time.sleep(0.05)

    joints = make_twist_base_joints(profile.joints_prefix)
    stream = _JointStatePoseStream(joint_names=joints)
    js_sub = LCMTransport(_JOINT_STATE_TOPIC, JointState)
    unsub = js_sub.subscribe(stream.on_joint_state)

    print(f"[hw] waiting up to {warmup_s:.0f}s for {_JOINT_STATE_TOPIC} ...")
    deadline = time.perf_counter() + warmup_s
    while time.perf_counter() < deadline:
        p, _ = stream.latest()
        if p is not None:
            break
        time.sleep(0.05)
    if stream.latest()[0] is None:
        safe_stop()
        unsub()
        raise SystemExit(f"No {_JOINT_STATE_TOPIC} — is `dimos run {profile.blueprint}` up?")

    def run_one_step(channel: str, amp: float, tag: str) -> dict:
        safe_stop()
        resp = (
            gate_input(
                f"\n[{tag} {channel}@{amp}] reposition robot into clear space, {gate_keys_label}: "
            )
            .strip()
            .lower()
        )
        if resp == "q":
            raise KeyboardInterrupt("operator quit")
        if resp == "s":
            print("  skipped")
            return {"action": "skip"}

        stream.start_buffering()
        t_end = time.perf_counter() + pre_roll_s
        while time.perf_counter() < t_end:
            publish(0.0, 0.0, 0.0)
            time.sleep(hw_dt)

        cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        cmd[channel] = amp
        sp, _ = stream.latest()
        if sp is None:
            print("  [abort] lost odom before step")
            stream.stop_and_pop()
            return {"action": "abort"}
        x0, y0 = sp.position.x, sp.position.y
        t0 = time.perf_counter()
        end_reason = "time"
        while True:
            now = time.perf_counter()
            t_rel = now - t0
            if t_rel > step_s:
                break
            publish(cmd["vx"], cmd["vy"], cmd["wz"])
            p, pt = stream.latest()
            if p is None or now - pt > profile.odom_stale_s:
                print(f"  [abort] stale odom ({now - pt:.2f}s)")
                end_reason = "stale"
                break
            dist = math.hypot(p.position.x - x0, p.position.y - y0)
            if dist >= max_dist:
                end_reason = "dist"
                break
            time.sleep(hw_dt)
        ts_abs, x_buf, y_buf, yaw_buf = stream.stop_and_pop()
        safe_stop()

        if len(ts_abs) < max(5, savgol_window):
            print(f"  [warn] {channel}@{amp}: too few samples, skip")
            return {"action": "abort"}
        ts_in = ts_abs - t0
        ts_rel, vx_all, vy_all, dyaw_all = reconstruct_body_velocities(
            ts_in, x_buf, y_buf, yaw_buf, window=savgol_window, order=savgol_order
        )
        n_dropped = len(ts_in) - len(ts_rel)
        if n_dropped:
            print(f"  dedupe: dropped {n_dropped} near-zero-dt samples")
        ys_all = {"vx": vx_all, "vy": vy_all, "wz": dyaw_all}[channel]
        pre_mask = ts_rel < 0.0
        post_mask = ~pre_mask
        if post_mask.sum() < max(5, savgol_window):
            print(f"  [warn] {channel}@{amp}: too few post-step samples, skip")
            return {"action": "abort"}
        noise_std: float | None = None
        if pre_mask.sum() >= 3:
            noise_std = float(np.std(ys_all[pre_mask]))
        ts_fit = ts_rel[post_mask]
        ys_raw = ys_all[post_mask]
        clip_max = max(5.0 * abs(amp), 0.5) if amp != 0.0 else 5.0
        ys_clipped, n_clipped = _physical_clip(ys_raw, clip_max)
        if n_clipped:
            print(f"  physical-clip: replaced {n_clipped}/{len(ys_raw)} samples > {clip_max:g}")
        ys_filt, n_replaced = _hampel(ys_clipped, hampel_window, hampel_n_sigma)
        if n_replaced:
            print(f"  hampel: replaced {n_replaced}/{len(ys_raw)} outliers")
        return {
            "action": "ok",
            "ts_fit": ts_fit,
            "ys_filt": ys_filt,
            "ys_raw": ys_raw,
            "n_replaced": n_replaced,
            "noise_std": noise_std,
            "end_reason": end_reason,
        }

    def fit_step_result(r: dict, channel: str, amp: float) -> Any | None:
        ts_fit, ys_filt = r["ts_fit"], r["ys_filt"]
        dt_med = float(np.median(np.diff(ts_fit))) if len(ts_fit) > 1 else 0.0
        fp = fit_fopdt(
            ts_fit,
            ys_filt,
            u_step=amp,
            min_deadtime=dt_med,
            noise_std=r["noise_std"],
            two_stage=r["noise_std"] is not None,
        )
        if not fp.converged or not np.isfinite([fp.K, fp.tau, fp.L]).all():
            print(f"  [warn] {channel}@{amp}: fit failed ({fp.reason})")
            return None
        print(
            f"  {channel}@{amp}: K={fp.K:.3f} tau={fp.tau:.3f} "
            f"L={fp.L:.3f}  ({len(r['ys_raw'])} samples, ended on {r['end_reason']})"
        )
        return fp

    def record_fit(channel: str, amp: float, r: dict, fp: Any, source: str) -> None:
        per_amplitude[channel].append(
            {
                "amp": amp,
                "amplitude": amp,
                "direction": "forward",
                "K": fp.K,
                "tau": fp.tau,
                "L": fp.L,
                "r2": fp.r_squared,
                "source": source,
            }
        )
        traces.append(
            {
                "channel": channel,
                "amp": amp,
                "t": np.asarray(r["ts_fit"], dtype=float),
                "y_raw": r["ys_raw"],
                "n_replaced": r["n_replaced"],
                "y": r["ys_filt"],
                "K": fp.K,
                "tau": fp.tau,
                "L": fp.L,
                "r2": fp.r_squared,
                "source": source,
            }
        )

    canonical: dict[str, FopdtChannelParams] = {}
    per_amplitude: dict[str, list[dict]] = {}
    floor_results: dict[str, list[dict]] = {}
    env_channels: dict[str, ChannelEnvelope] = {}
    traces: list[dict] = []
    try:
        for channel in profile.excited_channels:
            per_amplitude[channel] = []
            floor_results[channel] = []

            print(f"\n=== [{channel}] floor probe ===")
            for amp, is_ext in _floor_candidate_amplitudes(profile, channel):
                tag = "FLOOR+" if is_ext else "FLOOR"
                r = run_one_step(channel, amp, tag)
                if r["action"] == "skip":
                    continue
                if r["action"] != "ok":
                    floor_results[channel].append(
                        {
                            "amp": amp,
                            "motion_detected": False,
                            "sustained_samples": 0,
                            "net_displacement": 0.0,
                        }
                    )
                    continue
                passed, longest, net_disp = _d3_sustained_pass(
                    r["ts_fit"],
                    r["ys_filt"],
                    amp,
                    profile.floor_motion_threshold,
                    profile.floor_fractional_threshold,
                    profile.floor_sustained_samples,
                    profile.floor_displacement_threshold.get(channel, 0.0),
                )
                floor_results[channel].append(
                    {
                        "amp": amp,
                        "motion_detected": bool(passed),
                        "sustained_samples": int(longest),
                        "net_displacement": float(net_disp),
                    }
                )
                print(
                    f"  {channel}@{amp}: motion={'YES' if passed else 'no'} "
                    f"longest_run={longest} net_disp={net_disp:+.3f}"
                )
                if passed:
                    break

            floor, _ = _floor_from_probe(
                floor_results[channel], [r["amp"] for r in floor_results[channel]]
            )
            sweep_amps = _floor_anchored_sweep(profile, channel, floor)
            print(f"\n=== [{channel}] sweep (floor={floor:.3f} -> {sweep_amps}) ===")
            for amp in sweep_amps:
                r = run_one_step(channel, amp, "SWEEP")
                if r["action"] != "ok":
                    continue
                fp = fit_step_result(r, channel, amp)
                if fp is None:
                    continue
                record_fit(channel, amp, r, fp, source="sweep")

            print(f"\n=== [{channel}] ceiling probe ===")
            for amp in profile.ceiling_probe_amplitudes.get(channel, []):
                r = run_one_step(channel, amp, "CEILING")
                if r["action"] != "ok":
                    continue
                fp = fit_step_result(r, channel, amp)
                if fp is None:
                    continue
                record_fit(channel, amp, r, fp, source="ceiling_probe")

            sweep_fits = [f for f in per_amplitude[channel] if f["source"] == "sweep"]
            probe_fits = [f for f in per_amplitude[channel] if f["source"] == "ceiling_probe"]
            if not sweep_fits:
                raise RuntimeError(f"hw SI: no converged sweep fits for {channel!r}")
            env_channels[channel] = _envelope_from_channel_results(
                floor_results[channel], sweep_fits, probe_fits, profile, channel
            )
            linear = _pick_linear_regime_fit(sweep_fits) or sweep_fits[0]
            canonical[channel] = FopdtChannelParams(
                K=float(linear["K"]), tau=float(linear["tau"]), L=float(linear["L"])
            )
            print(
                f"\n[{channel}] canonical (linear-regime, amp={linear['amp']:g}): "
                f"K={linear['K']:.3f} tau={linear['tau']:.3f} L={linear['L']:.3f} "
                f"r2={linear['r2']:.2f}"
            )
            ce = env_channels[channel]
            print(
                f"[{channel}] envelope: floor={ce.floor:.3f} (nf={ce.floor_not_found}) "
                f"ceiling={ce.ceiling:.3f} (nf={ce.ceiling_not_found})"
            )
    except KeyboardInterrupt:
        raise SystemExit(
            "\n[hw] aborted by operator — robot stopped, no artifact written."
        ) from None
    finally:
        safe_stop()
        unsub()

    fallback_env = ChannelEnvelope(
        floor=0.0, ceiling=0.0, floor_not_found=True, ceiling_not_found=True
    )
    for ch in _CHANNELS:
        if ch not in canonical:
            canonical[ch] = canonical["vx"]
            per_amplitude[ch] = []
            floor_results.setdefault(ch, [])
            env_channels[ch] = fallback_env
            print(f"  [note] {ch} not excited on hw — placeholder {ch} = vx")
    envelope = VelocityEnvelope(vx=env_channels["vx"], vy=env_channels["vy"], wz=env_channels["wz"])
    return (
        TwistBasePlantParams(vx=canonical["vx"], vy=canonical["vy"], wz=canonical["wz"]),
        per_amplitude,
        traces,
        envelope,
        floor_results,
    )


class CharacterizerConfig(ModuleConfig):
    robot: str = "go2"
    mode: Literal["hw", "self-test", "re-derive"] = "hw"
    artifact: str | None = None
    out: str | None = None
    robot_id: str | None = None
    surface: str = "concrete"
    gait_mode: str = "default"
    step_s: float | None = None
    pre_roll_s: float | None = None
    odom_warmup: float | None = None
    max_dist: float | None = None
    gate_source: Literal["stdin", "stream"] = "stdin"
    savgol_window: int = 11
    savgol_order: int = 2
    hampel_window: int = 11
    hampel_n_sigma: float = 3.0
    two_stage_fit: bool = True


class Characterizer(Module):
    config: CharacterizerConfig

    gate: In[Int8]

    _gate_queue: queue.Queue[str]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._gate_queue = queue.Queue()

    @rpc
    def start(self) -> None:
        super().start()
        if self.config.gate_source == "stream":
            self.register_disposable(Disposable(self.gate.subscribe(self._on_gate_event)))
        self._run()

    def _on_gate_event(self, msg: Int8) -> None:
        code = int(msg.data)
        translated = {GATE_ADVANCE: "", GATE_SKIP: "s", GATE_QUIT: "q"}.get(code, "")
        self._gate_queue.put(translated)

    def _wait_gate_stream(self, prompt: str) -> str:
        print(prompt, end="", flush=True)
        return self._gate_queue.get()

    def _run(self) -> None:
        cfg = self.config
        profile = _resolve_profile(cfg.robot)
        step_s = cfg.step_s if cfg.step_s is not None else profile.step_s
        pre_roll_s = cfg.pre_roll_s if cfg.pre_roll_s is not None else profile.pre_roll_s
        warmup_s = cfg.odom_warmup if cfg.odom_warmup is not None else profile.odom_warmup_s
        max_dist = cfg.max_dist if cfg.max_dist is not None else profile.max_dist_m
        robot_id = cfg.robot_id if cfg.robot_id is not None else profile.robot_id
        out_root = Path(cfg.out).expanduser() if cfg.out else DEFAULT_OUT_DIR

        if cfg.mode == "re-derive":
            self._run_re_derive(cfg, profile, robot_id, out_root)
            return

        if cfg.mode == "hw":
            if cfg.gate_source == "stream":
                gate_input: Callable[[str], str] = self._wait_gate_stream
                gate_keys_label = "focus pygame window: ENTER=run  K=skip  Backspace=quit"
            else:
                gate_input = input
                gate_keys_label = "ENTER=run  s=skip  q=quit"
            fitted, per_amplitude, traces, envelope, floor_results = _fit_hw(
                profile,
                step_s,
                pre_roll_s,
                warmup_s,
                max_dist,
                gate_input=gate_input,
                gate_keys_label=gate_keys_label,
                savgol_window=cfg.savgol_window,
                savgol_order=cfg.savgol_order,
                hampel_window=cfg.hampel_window,
                hampel_n_sigma=cfg.hampel_n_sigma,
            )
        else:
            fitted, per_amplitude, traces, envelope, floor_results = _fit_selftest(profile)

        provenance = Provenance(
            robot_id=robot_id,
            surface=cfg.surface,
            mode=cfg.gait_mode,
            date=date.today().isoformat(),
            git_sha=git_sha(),
            sim_or_hw="hw" if cfg.mode == "hw" else "self-test",
            characterization_session_dir=(
                f"(real {profile.name}, LCM SI)" if cfg.mode == "hw" else "(in-process self-test)"
            ),
        )

        env_dc = VelocityEnvelopeDC(
            vx=ChannelEnvelopeDC(**asdict(envelope.vx)),
            vy=ChannelEnvelopeDC(**asdict(envelope.vy)),
            wz=ChannelEnvelopeDC(**asdict(envelope.wz)),
        )
        dyn_by_amp = DynamicsByAmplitude(
            vx=[
                AmplitudeFitDC(
                    amp=e["amp"],
                    K=e["K"],
                    tau=e["tau"],
                    L=e["L"],
                    r2=e.get("r2", 0.0),
                    source=e.get("source", "sweep"),
                )
                for e in per_amplitude.get("vx", [])
            ],
            vy=[
                AmplitudeFitDC(
                    amp=e["amp"],
                    K=e["K"],
                    tau=e["tau"],
                    L=e["L"],
                    r2=e.get("r2", 0.0),
                    source=e.get("source", "sweep"),
                )
                for e in per_amplitude.get("vy", [])
            ],
            wz=[
                AmplitudeFitDC(
                    amp=e["amp"],
                    K=e["K"],
                    tau=e["tau"],
                    L=e["L"],
                    r2=e.get("r2", 0.0),
                    source=e.get("source", "sweep"),
                )
                for e in per_amplitude.get("wz", [])
            ],
        )
        floor_dc = FloorProbeResults(
            vx=[FloorProbeResultDC(**r) for r in floor_results.get("vx", [])],
            vy=[FloorProbeResultDC(**r) for r in floor_results.get("vy", [])],
            wz=[FloorProbeResultDC(**r) for r in floor_results.get("wz", [])],
        )

        artifact = derive_config(
            fitted,
            provenance,
            per_amplitude=per_amplitude,
            vx_max=profile.vx_max,
            wz_max=profile.wz_max,
            velocity_envelope=env_dc,
            dynamics_by_amplitude=dyn_by_amp,
            floor_probe_results=floor_dc,
            min_speed_floor=profile.min_speed_floor,
        )
        if cfg.mode == "hw" and "vy" not in profile.excited_channels:
            artifact.caveats.append(
                f"vy was NOT characterized on hardware ({profile.name} does not "
                "strafe in this gait); plant.vy / feedforward.K_vy are a "
                "placeholder copy of vx. The benchmark paths are vx+wz only, so "
                "this does not affect tuning; re-characterize vy if a "
                "lateral-capable gait is used."
            )

        out_dir = out_root / robot_id
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = (
            out_dir
            / f"{robot_id}_config_{cfg.mode}_{cfg.surface}_{provenance.date}_{provenance.git_sha}.json"
        )
        artifact.to_json(out_path)
        env_png = out_path.with_suffix(".png")
        steps_png = out_path.with_name(out_path.stem + "_steps.png")
        _plot_fits(
            traces,
            provenance,
            profile,
            env_png,
            envelope=envelope,
            kind="envelope",
        )
        _plot_fits(traces, provenance, profile, steps_png, kind="steps")

        tag = "ROBOT-VALID" if artifact.valid_for_tuning else "NOT robot-valid (plumbing check)"
        print("\nEnvelope plot (the primary deliverable — K, output, τ vs amp):")
        print(f"  {env_png.resolve()}")
        print("Per-amp step responses (eyeball the FOPDT fits):")
        print(f"  {steps_png.resolve()}")
        print(f"Config artifact [{tag}] (machine handoff for the benchmark):")
        print(f"  {out_path.resolve()}")

    def _run_re_derive(
        self,
        cfg: CharacterizerConfig,
        profile: RobotPlantProfile,
        robot_id: str,
        out_root: Path,
    ) -> None:
        if not cfg.artifact:
            raise SystemExit("--mode re-derive requires --artifact <path-to-existing-json>")
        src_path = Path(cfg.artifact).expanduser()
        print(f"[re-derive] reading {src_path}")
        src = TuningConfig.from_json(src_path)
        if src.dynamics_by_amplitude is None:
            raise SystemExit(
                f"{src_path}: artifact has no dynamics_by_amplitude — "
                "this artifact is methodology v1 (sparse sweep), nothing to "
                "re-derive. Re-run characterization in v2 mode."
            )

        new = re_derive_config(
            src,
            vx_max=profile.vx_max,
            wz_max=profile.wz_max,
            floor_probe_amplitudes=dict(profile.floor_probe_amplitudes),
            min_speed_floor=profile.min_speed_floor,
            sag_threshold=profile.ceiling_k_sag_threshold,
        )
        new.caveats.insert(
            0,
            f"Re-derived on {date.today().isoformat()} from {src_path.name} "
            f"using the operational-ceiling logic (max(K·amp) clamped to "
            f"envelope). Plant + FF passed through unchanged.",
        )

        out_dir = out_root / robot_id
        out_dir.mkdir(parents=True, exist_ok=True)
        suffix = "rederived"
        out_path = out_dir / f"{src_path.stem}__{suffix}.json"
        new.to_json(out_path)

        env = new.velocity_envelope
        png_path = out_path.with_suffix(".png")
        _plot_fits(
            [],  # no raw step traces in the artifact — row 1 omitted
            new.provenance,
            profile,
            png_path,
            envelope=env,
            dynamics=new.dynamics_by_amplitude,
        )

        print("\nRe-derived velocity_envelope:")
        for ch in ("vx", "vy", "wz"):
            e = getattr(env, ch)
            sat = f" sat@={e.saturating_at_amp:g}" if e.saturating_at_amp is not None else ""
            print(
                f"  {ch}: floor={e.floor:.3f} (nf={e.floor_not_found}) "
                f"ceiling={e.ceiling:.3f} (nf={e.ceiling_not_found}){sat}"
            )
        print()
        print(f"New artifact: {out_path.resolve()}")
        print(f"New PNG:      {png_path.resolve()}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Twist-base characterization -> tuning artifact")
    ap.add_argument("--robot", default="go2", help=f"one of {sorted(ROBOT_PLANT_PROFILES)}")
    ap.add_argument("--mode", choices=["hw", "self-test", "re-derive"], default="hw")
    ap.add_argument(
        "--artifact",
        default=None,
        help="re-derive mode: existing artifact JSON to recompute envelope from",
    )
    ap.add_argument(
        "--out",
        default=None,
        help=f"output dir (default: {DEFAULT_OUT_DIR}/<robot_id>/)",
    )
    ap.add_argument("--robot-id", default=None, help="provenance id (default: profile.robot_id)")
    ap.add_argument("--surface", default="concrete")
    ap.add_argument("--gait-mode", default="default")
    ap.add_argument(
        "--step-s",
        type=float,
        default=None,
        help="per-step excitation duration (s); default from profile",
    )
    ap.add_argument(
        "--pre-roll-s", type=float, default=None, help="zero-command settle before each step (s)"
    )
    ap.add_argument(
        "--odom-warmup", type=float, default=None, help="how long to wait for first odom (s)"
    )
    ap.add_argument(
        "--max-dist",
        type=float,
        default=None,
        help="per-step travel cap (m); ends the step early at speed",
    )
    args = ap.parse_args()

    instance = Characterizer(
        robot=args.robot,
        mode=args.mode,
        artifact=args.artifact,
        out=args.out,
        robot_id=args.robot_id,
        surface=args.surface,
        gait_mode=args.gait_mode,
        step_s=args.step_s,
        pre_roll_s=args.pre_roll_s,
        odom_warmup=args.odom_warmup,
        max_dist=args.max_dist,
    )
    instance.start()


if __name__ == "__main__":
    main()
