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

"""Tool 1 of the twist-base tuning deliverable: characterization.

**This is a hardware tool.** It measures a real velocity-commanded
base's per-axis response (vx, vy, wz), fits FOPDT per channel, runs the
DERIVE step, and emits the versioned config artifact. Robot-agnostic:
everything robot-specific comes from the selected ``RobotPlantProfile``
(``--robot``, default ``go2``).

    # terminal 1 (the robot's bring-up blueprint, see the profile):
    dimos run <profile.blueprint>
    # terminal 2 (strip /nix/store from LD_LIBRARY_PATH — see README):
    uv run python -m dimos.utils.benchmarking.characterization \\
        --robot go2 --mode hw --surface concrete

`--mode hw` (default) drives the real robot via the same path the
benchmark does: an in-process ``ControlCoordinator`` with the
``transport_lcm`` twist-base adapter spins up to give us a ``joint_state``
Out stream sourced from the adapter's odometry. Signal-injection itself
stays a standalone Twist publisher (SI is open-loop by nature). Each
step is **operator-gated**: before every step the robot is stopped and
we wait for ENTER. Safe (velocity clamp, zero-Twist on exit/SIGINT,
stale-odom abort, distance + time caps).

`--mode self-test` is a **plumbing check, NOT a tuning artifact**: it
steps the profile's in-process FOPDT sim plant and recovers it. It only
proves the measure->fit->derive code runs; the artifact is stamped
`valid_for_tuning=false`. Used by pytest/CI without a robot.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
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

# Well-known LCM topics — every ControlCoordinator blueprint honors this
# contract (twist_command In = /cmd_vel, joint_state Out =
# /coordinator/joint_state). Adding a robot to the tools means adding a
# RobotPlantProfile, not a new topic / module / adapter.
_CMD_VEL_TOPIC = "/cmd_vel"
_JOINT_STATE_TOPIC = "/coordinator/joint_state"
from dimos.utils.benchmarking.plant import (
    ROBOT_PLANT_PROFILES,
    FopdtChannelParams,
    RobotPlantProfile,
    TwistBasePlantParams,
    TwistBasePlantSim,
)
from dimos.utils.benchmarking.tuning import Provenance, derive_config, git_sha
from dimos.utils.characterization.modeling.fopdt import fit_fopdt, fopdt_step_response

# Fixed twist-base velocity-tuple order (estimator output / channel
# index). NOT robot-specific — the per-robot excited subset is
# profile.excited_channels.
_CHANNELS = ("vx", "vy", "wz")
_SIM_DT = 0.02  # in-process self-test integration step (not robot-specific)

REPORTS_DIR = Path(__file__).parent / "reports"
# New default landing dir for tuning artifacts: <repo>/data/characterization/<robot_id>/.
# REPORTS_DIR is retained as the package-local location for the README / docs,
# not as the artifact output default.
DEFAULT_OUT_DIR = get_project_root() / "data" / "characterization"


def reconstruct_body_velocities(
    ts: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    yaw: np.ndarray,
    window: int = 5,
    order: int = 2,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Recover body-frame (vx, vy, dyaw) from a buffered pose trace.

    The pipeline is intentionally smooth-position-then-differentiate
    (not differentiate-then-smooth): odom measures position directly,
    and numerical differentiation amplifies high-frequency noise that no
    amount of post-hoc smoothing fully recovers from. On a legged base
    the dominant "noise" is body bob at gait frequency — buffering raw
    pose, applying Savitzky-Golay to the position trace, then
    central-differencing the smoothed series gives a much cleaner
    velocity signal for the FOPDT fit. Matches the historical
    ``analyze.py`` pipeline.

    ``window`` must be odd and > ``order``; falls back to no smoothing
    if the buffer is too short."""
    from scipy.signal import savgol_filter

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
    return vx, vy, dyaw


def _hampel(ys: np.ndarray, window: int = 11, n_sigma: float = 3.0) -> tuple[np.ndarray, int]:
    """Hampel outlier filter — replace points >n_sigma·MAD from the local
    median with the local median. Window-centered; preserves trace length.

    Returns (filtered_array, n_replaced). MAD is scaled by 1.4826 so
    n_sigma is in Gaussian-sigma units. Spikes from leg impacts / odom
    estimator glitches get surgically removed; the rest of the trace is
    untouched (the goal is to keep FOPDT step shape readable, not
    smooth-out the dynamics)."""
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


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _resolve_profile(name: str) -> RobotPlantProfile:
    try:
        return ROBOT_PLANT_PROFILES[name]
    except KeyError:
        raise SystemExit(
            f"unknown --robot {name!r}; known: {sorted(ROBOT_PLANT_PROFILES)}"
        ) from None


# --- self-test (in-process FOPDT plant; NOT robot-valid) -----------------


def _fit_selftest(profile: RobotPlantProfile) -> tuple[TwistBasePlantParams, dict, list[dict]]:
    """Step the profile's FOPDT sim plant and recover it. Plumbing check
    only — proves the measure->fit->derive code path runs."""
    truth = profile.sim_plant
    plant = TwistBasePlantSim(truth)
    n_pre = int(profile.pre_roll_s / _SIM_DT)
    n_step = int(profile.step_s / _SIM_DT)
    pooled: dict[str, FopdtChannelParams] = {}
    per_amplitude: dict[str, list[dict]] = {}
    traces: list[dict] = []

    for channel in _CHANNELS:
        fits = []
        per_amplitude[channel] = []
        for amp in profile.si_amplitudes[channel]:
            plant.reset(0.0, 0.0, 0.0, _SIM_DT)
            cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
            for _ in range(n_pre):
                plant.step(cmd["vx"], cmd["vy"], cmd["wz"], _SIM_DT)
            cmd[channel] = amp
            ys = []
            for _ in range(n_step):
                plant.step(cmd["vx"], cmd["vy"], cmd["wz"], _SIM_DT)
                ys.append(getattr(plant, channel))
            t = np.arange(len(ys), dtype=float) * _SIM_DT
            fp = fit_fopdt(t, np.asarray(ys, dtype=float), u_step=amp)
            if not fp.converged or not np.isfinite([fp.K, fp.tau, fp.L]).all():
                print(f"  [warn] {channel}@{amp}: fit failed ({fp.reason})")
                continue
            fits.append(fp)
            per_amplitude[channel].append(
                {"amplitude": amp, "direction": "forward", "K": fp.K, "tau": fp.tau, "L": fp.L}
            )
            traces.append(
                {
                    "channel": channel,
                    "amp": amp,
                    "t": np.asarray(t, dtype=float),
                    "y": np.asarray(ys, dtype=float),
                    "K": fp.K,
                    "tau": fp.tau,
                    "L": fp.L,
                    "r2": fp.r_squared,
                }
            )
        if not fits:
            raise RuntimeError(f"self-test: no converged fits for {channel!r}")
        pooled[channel] = FopdtChannelParams(
            K=float(np.mean([f.K for f in fits])),
            tau=float(np.mean([f.tau for f in fits])),
            L=float(np.mean([f.L for f in fits])),
        )
    fitted = TwistBasePlantParams(vx=pooled["vx"], vy=pooled["vy"], wz=pooled["wz"])
    print("\nself-test (recovered vs injected FOPDT ground truth):")
    print(f"  {'chan':4} {'K fit/true':>20} {'tau fit/true':>20} {'L fit/true':>20}")
    for ch in _CHANNELS:
        f, g = getattr(fitted, ch), getattr(truth, ch)
        print(
            f"  {ch:4} {f.K:8.3f}/{g.K:<8.3f}   {f.tau:8.3f}/{g.tau:<8.3f}   {f.L:8.3f}/{g.L:<8.3f}"
        )
    return fitted, per_amplitude, traces


# --- fit-quality graph (the human-facing deliverable) -------------------


def _plot_fits(
    traces: list[dict], provenance: Provenance, profile: RobotPlantProfile, out: Path
) -> None:
    """One column per channel; each step's measured velocity overlaid
    with its fitted FOPDT step response. This is the artifact a human
    reads to judge whether the model matches the real robot."""
    if not traces:
        return
    channels = list(dict.fromkeys(t["channel"] for t in traces))
    fig, axes = plt.subplots(1, len(channels), figsize=(6.0 * len(channels), 4.6), squeeze=False)
    for ax, ch in zip(axes[0], channels, strict=True):
        for tr in [t for t in traces if t["channel"] == ch]:
            t_arr = tr["t"]
            (line,) = ax.plot(t_arr, tr["y"], lw=1.4, alpha=0.85, label=f"meas @{tr['amp']:g}")
            y_raw = tr.get("y_raw")
            if y_raw is not None and tr.get("n_replaced", 0) > 0:
                ax.plot(t_arr, y_raw, ":", lw=0.9, color=line.get_color(), alpha=0.5)
            yhat = fopdt_step_response(t_arr, tr["K"], tr["tau"], tr["L"], tr["amp"])
            ax.plot(t_arr, yhat, "--", lw=1.4, color=line.get_color(), alpha=0.9)
            row = list(t2["amp"] for t2 in traces if t2["channel"] == ch).index(tr["amp"])
            ann = (
                f"@{tr['amp']:g}: K={tr['K']:.3f} τ={tr['tau']:.3f} "
                f"L={tr['L']:.3f} r²={tr['r2']:.2f}"
            )
            if tr.get("n_replaced", 0) > 0:
                ann += f" (hampel: {tr['n_replaced']})"
            ax.annotate(
                ann,
                xy=(0.02, 0.97 - 0.06 * row),
                xycoords="axes fraction",
                ha="left",
                va="top",
                fontsize=7,
                color=line.get_color(),
            )
        unit = "rad/s" if ch == "wz" else "m/s"
        ax.set_title(f"{ch}  (solid = filtered, dotted = raw, dashed = FOPDT fit)")
        ax.set_xlabel("time since step edge (s)")
        ax.set_ylabel(f"{ch} ({unit})")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="lower right", fontsize=7)
    p = provenance
    fig.suptitle(
        f"{profile.name} FOPDT characterization — {p.robot_id} / {p.surface} / "
        f"{p.mode} / {p.sim_or_hw} — {p.date} ({p.git_sha})"
    )
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


# --- hardware SI (real robot over LCM, operator-gated, safe) -------------


class _JointStatePoseStream:
    """Pose stream sourced from a coordinator's ``joint_state`` Out.
    Positions are [x, y, yaw] (twist-base adapter convention). Buffers
    raw pose samples between :meth:`start_buffering` and
    :meth:`stop_and_pop` so callers can reconstruct body-frame
    velocity at step end (via :func:`reconstruct_body_velocities`)
    rather than per-sample. Smooth-then-differentiate is much cleaner
    than differentiate-then-smooth on legged-robot data — see that
    function's docstring."""

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
        # The caller waits a grace period after coord.start before
        # sampling, so the (0,0,0) placeholder from ConnectedTwistBase
        # (emitted before the adapter receives its first /odom) does
        # not get latched as the start pose.
        now = time.perf_counter()
        pose = PoseStamped(
            ts=now,
            position=Vector3(x, y, 0.0),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
        )
        with self._lock:
            self._pose, self._pose_t = pose, now
            if self._buffering:
                self._buffer.append((now, x, y, yaw))

    def latest(self) -> tuple[PoseStamped | None, float]:
        with self._lock:
            return self._pose, self._pose_t

    def start_buffering(self) -> None:
        """Begin recording raw pose samples — called at step start."""
        with self._lock:
            self._buffer = []
            self._buffering = True

    def stop_and_pop(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Stop buffering and return (ts, x, y, yaw) arrays for the step."""
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
) -> tuple[TwistBasePlantParams, dict, list[dict]]:
    _prereq_banner(profile)
    hw_dt = 1.0 / profile.tick_rate_hz

    # Signal-injection is open-loop — publish Twist directly on the coord's
    # twist_command topic (/cmd_vel). The operator coord's velocity task
    # picks it up and routes through whichever adapter (transport_lcm for
    # Go2, flowbase Portal RPC for FlowBase, ...) the operator brought up.
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

    # Observation: subscribe to the operator coord's /coordinator/joint_state
    # Out directly over LCM. JointState positions carry [x, y, yaw] because
    # ConnectedTwistBase populates them from adapter.read_odometry(). Body
    # velocity recovered by pose-differentiation in the stream (commanded
    # velocities in the JointState are not measured).
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

    pooled: dict[str, FopdtChannelParams] = {}
    per_amplitude: dict[str, list[dict]] = {}
    traces: list[dict] = []
    try:
        for channel in profile.excited_channels:
            fits = []
            per_amplitude[channel] = []
            for amp in profile.si_amplitudes[channel]:
                safe_stop()
                resp = (
                    gate_input(
                        f"\n[{channel}@{amp}] reposition robot into clear space, "
                        f"{gate_keys_label}: "
                    )
                    .strip()
                    .lower()
                )
                if resp == "q":
                    raise KeyboardInterrupt("operator quit")
                if resp == "s":
                    print("  skipped")
                    continue

                # pre-roll zeros (settle). Buffer during pre-roll too so
                # the pre-step samples give us a noise_std estimate
                # (robot at rest, zero commanded — pure odom noise).
                stream.start_buffering()
                t_end = time.perf_counter() + pre_roll_s
                while time.perf_counter() < t_end:
                    publish(0.0, 0.0, 0.0)
                    time.sleep(hw_dt)

                # step. Ends on whichever comes first: travelled distance
                # >= max_dist (the real-space bound — at high speed the
                # time cap would run the robot out of the test area), or
                # t_rel > step_s (time safety cap; also the terminator for
                # wz, which spins in place and never accumulates distance).
                # Body-frame velocity is reconstructed from buffered pose
                # AFTER the step — smooth-then-differentiate gives a much
                # cleaner trace on a legged base than per-tick differentiation.
                cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
                cmd[channel] = amp
                sp, _ = stream.latest()
                if sp is None:
                    print("  [abort] lost odom before step")
                    continue
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
                    continue
                ts_rel = ts_abs - t0
                vx_all, vy_all, dyaw_all = reconstruct_body_velocities(
                    ts_rel, x_buf, y_buf, yaw_buf, window=savgol_window, order=savgol_order
                )
                ys_all = {"vx": vx_all, "vy": vy_all, "wz": dyaw_all}[channel]
                # Split pre/post step. Pre is zero-commanded baseline -
                # std of that velocity = our noise estimate.
                pre_mask = ts_rel < 0.0
                post_mask = ~pre_mask
                if post_mask.sum() < max(5, savgol_window):
                    print(f"  [warn] {channel}@{amp}: too few post-step samples, skip")
                    continue
                noise_std: float | None = None
                if pre_mask.sum() >= 3:
                    noise_std = float(np.std(ys_all[pre_mask]))
                ts_fit = ts_rel[post_mask]
                ys_raw = ys_all[post_mask]
                ys_filt, n_replaced = _hampel(ys_raw, hampel_window, hampel_n_sigma)
                if n_replaced:
                    print(f"  hampel: replaced {n_replaced}/{len(ys_raw)} outliers")
                # Floor L at the actual data sample period — fit cannot
                # physically resolve deadtime finer than odom rate.
                dt_med = float(np.median(np.diff(ts_fit))) if len(ts_fit) > 1 else 0.0
                fp = fit_fopdt(
                    ts_fit,
                    ys_filt,
                    u_step=amp,
                    min_deadtime=dt_med,
                    noise_std=noise_std,
                    two_stage=noise_std is not None,
                )
                if not fp.converged or not np.isfinite([fp.K, fp.tau, fp.L]).all():
                    print(f"  [warn] {channel}@{amp}: fit failed ({fp.reason})")
                    continue
                print(
                    f"  {channel}@{amp}: K={fp.K:.3f} tau={fp.tau:.3f} "
                    f"L={fp.L:.3f}  ({len(ys_raw)} samples, ended on {end_reason})"
                )
                fits.append(fp)
                per_amplitude[channel].append(
                    {"amplitude": amp, "direction": "forward", "K": fp.K, "tau": fp.tau, "L": fp.L}
                )
                traces.append(
                    {
                        "channel": channel,
                        "amp": amp,
                        "t": np.asarray(ts_fit, dtype=float),
                        "y_raw": ys_raw,
                        "n_replaced": n_replaced,
                        "y": ys_filt,
                        "K": fp.K,
                        "tau": fp.tau,
                        "L": fp.L,
                        "r2": fp.r_squared,
                    }
                )
            if not fits:
                raise RuntimeError(f"hw SI: no converged fits for {channel!r}")
            pooled[channel] = FopdtChannelParams(
                K=float(np.mean([f.K for f in fits])),
                tau=float(np.mean([f.tau for f in fits])),
                L=float(np.mean([f.L for f in fits])),
            )
    except KeyboardInterrupt:
        # finally below does safe_stop + unsub — don't duplicate
        raise SystemExit(
            "\n[hw] aborted by operator — robot stopped, no artifact written."
        ) from None
    finally:
        safe_stop()
        unsub()

    # Channels not excited (e.g. vy on a non-strafing robot) are
    # placeholdered = vx so FF / profile stay sane; flagged in caveats.
    for ch in _CHANNELS:
        if ch not in pooled:
            pooled[ch] = pooled["vx"]
            per_amplitude[ch] = []
            print(f"  [note] {ch} not excited on hw — placeholder {ch} = vx")
    return (
        TwistBasePlantParams(vx=pooled["vx"], vy=pooled["vy"], wz=pooled["wz"]),
        per_amplitude,
        traces,
    )


class CharacterizerConfig(ModuleConfig):
    """Config for :class:`Characterizer`. Each field mirrors a CLI flag on
    the existing ``characterization`` entrypoint; ``None`` means "fall
    back to the selected ``RobotPlantProfile``".

    ``gate_source`` selects how each SI step is gated. ``"stdin"``
    (default, CLI mode) reads ENTER/s/q from the terminal; ``"stream"``
    (blueprint mode) consumes events from the ``gate`` In port wired to
    a co-running :class:`~dimos.robot.unitree.keyboard_teleop.KeyboardTeleop`
    that publishes ENTER/K/Backspace as ``"advance"``/``"skip"``/``"quit"``.
    """

    robot: str = "go2"
    mode: Literal["hw", "self-test"] = "hw"
    out: str | None = None
    robot_id: str | None = None
    surface: str = "concrete"
    gait_mode: str = "default"
    step_s: float | None = None
    pre_roll_s: float | None = None
    odom_warmup: float | None = None
    max_dist: float | None = None
    gate_source: Literal["stdin", "stream"] = "stdin"
    # Savitzky-Golay smoothing applied to the POSITION trace before
    # central-differencing to body-frame velocity. This is the main
    # noise-rejection knob for legged-robot data: gait-frequency body
    # bob lives in the raw pose; smooth-then-differentiate keeps it out
    # of the velocity signal. Window must be odd and > order. Set
    # window=0 or window <= order to disable.
    # Default 11 = ~2 gait cycles at 10 Hz tick rate / 2 Hz Go2 trot;
    # synthetic-noise sanity check (3cm gait bob + 5mm odom noise) shows
    # window=5 leaves vx-std ~= 0.18 m/s, window=11 drops it to ~= 0.05.
    savgol_window: int = 11
    savgol_order: int = 2
    # Hampel outlier filter on the post-reconstruction velocity trace
    # before FOPDT fitting. Catches residual single-sample spikes that
    # survive savgol smoothing (e.g. odom estimator hiccups). Set
    # window=0 to disable.
    hampel_window: int = 11
    hampel_n_sigma: float = 3.0
    # Two-stage FOPDT fit: estimate L directly from data (first time the
    # response crosses ~5*noise_std above the pre-step noise floor),
    # then fit K and tau with L pinned. Decouples L from tau - joint
    # LM-fit can trade off a tiny L for a slightly inflated tau when the
    # data is noisy. Auto-disabled if noise_std can't be estimated
    # (need >=3 pre-step samples in the pose buffer).
    two_stage_fit: bool = True


class Characterizer(Module):
    """Module wrapper around the FOPDT characterization sequence.

    Driven via the CLI shim in :func:`main` (``gate_source="stdin"``) or
    composed into the all-in-one blueprint
    ``unitree-go2-characterization`` (``gate_source="stream"``, gating
    events arrive on the ``gate`` In port from a co-running
    :class:`~dimos.robot.unitree.keyboard_teleop.KeyboardTeleop`).
    Stdin-driven gating cannot work inside the blueprint runner because
    deployed modules run in forkserver-spawned worker subprocesses that
    don't share the parent CLI's TTY — hence the stream-based path.

    ``start()`` blocks for the full operator-gated SI loop, then returns.
    """

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
        # Translate the pygame-side gate codes to the legacy CLI vocab so
        # _fit_hw's response check (``""``=run, ``"s"``=skip, ``"q"``=quit)
        # is unchanged.
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

        if cfg.mode == "hw":
            if cfg.gate_source == "stream":
                gate_input: Callable[[str], str] = self._wait_gate_stream
                gate_keys_label = "focus pygame window: ENTER=run  K=skip  Backspace=quit"
            else:
                gate_input = input
                gate_keys_label = "ENTER=run  s=skip  q=quit"
            fitted, per_amplitude, traces = _fit_hw(
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
            fitted, per_amplitude, traces = _fit_selftest(profile)

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
        artifact = derive_config(
            fitted,
            provenance,
            per_amplitude=per_amplitude,
            vx_max=profile.vx_max,
            wz_max=profile.wz_max,
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
        plot_path = out_path.with_suffix(".png")
        _plot_fits(traces, provenance, profile, plot_path)

        tag = "ROBOT-VALID" if artifact.valid_for_tuning else "NOT robot-valid (plumbing check)"
        print("\nFOPDT fit graph (the deliverable — model vs real data):")
        print(f"  {plot_path.resolve()}")
        print(f"Config artifact [{tag}] (machine handoff for the benchmark):")
        print(f"  {out_path.resolve()}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Twist-base characterization -> tuning artifact")
    ap.add_argument("--robot", default="go2", help=f"one of {sorted(ROBOT_PLANT_PROFILES)}")
    ap.add_argument("--mode", choices=["hw", "self-test"], default="hw")
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
