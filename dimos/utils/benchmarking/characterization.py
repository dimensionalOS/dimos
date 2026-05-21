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
from datetime import date
import math
from pathlib import Path
import threading
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from dimos.control.components import make_twist_base_joints
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState

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
            yhat = fopdt_step_response(t_arr, tr["K"], tr["tau"], tr["L"], tr["amp"])
            ax.plot(t_arr, yhat, "--", lw=1.4, color=line.get_color(), alpha=0.9)
            row = list(t2["amp"] for t2 in traces if t2["channel"] == ch).index(tr["amp"])
            ax.annotate(
                f"@{tr['amp']:g}: K={tr['K']:.3f} τ={tr['tau']:.3f} "
                f"L={tr['L']:.3f} r²={tr['r2']:.2f}",
                xy=(0.02, 0.97 - 0.06 * row),
                xycoords="axes fraction",
                ha="left",
                va="top",
                fontsize=7,
                color=line.get_color(),
            )
        unit = "rad/s" if ch == "wz" else "m/s"
        ax.set_title(f"{ch}  (solid = measured, dashed = FOPDT fit)")
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
    """Pose + body-velocity stream sourced from a coordinator's
    ``joint_state`` Out. Reuses the benchmark observer's math: positions
    are [x, y, yaw] (twist-base adapter convention); body-frame velocity
    is recovered by EMA-smoothed pose differentiation. Drop-in
    replacement for the old standalone /odom LCM subscriber +
    in-house ``_PoseVelocityEstimator``."""

    def __init__(self, joint_names: list[str], alpha: float = 0.5) -> None:
        self._jx, self._jy, self._jyaw = joint_names
        self._alpha = alpha
        self._lock = threading.Lock()
        self._pose: PoseStamped | None = None
        self._pose_t: float = 0.0
        self._prev_pose: PoseStamped | None = None
        self._prev_t: float | None = None
        self._vx = self._vy = self._wz = 0.0

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
            if self._prev_pose is not None and self._prev_t is not None:
                dt = now - self._prev_t
                if dt > 0:
                    dx = pose.position.x - self._prev_pose.position.x
                    dy = pose.position.y - self._prev_pose.position.y
                    y0 = self._prev_pose.orientation.euler[2]
                    y1 = pose.orientation.euler[2]
                    dyaw = (y1 - y0 + math.pi) % (2 * math.pi) - math.pi
                    c, s = math.cos(y1), math.sin(y1)
                    bx = (dx / dt) * c + (dy / dt) * s
                    by = -(dx / dt) * s + (dy / dt) * c
                    a = self._alpha
                    self._vx = a * bx + (1 - a) * self._vx
                    self._vy = a * by + (1 - a) * self._vy
                    self._wz = a * (dyaw / dt) + (1 - a) * self._wz
            self._prev_pose, self._prev_t = pose, now
            self._pose, self._pose_t = pose, now

    def latest(self) -> tuple[PoseStamped | None, float, tuple[float, float, float]]:
        with self._lock:
            return self._pose, self._pose_t, (self._vx, self._vy, self._wz)

    def reset_velocity(self) -> None:
        """Drop EMA state — called at pre-roll so each step starts clean."""
        with self._lock:
            self._vx = self._vy = self._wz = 0.0
            self._prev_pose = None
            self._prev_t = None


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
        p, _, _ = stream.latest()
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
                    input(
                        f"\n[{channel}@{amp}] reposition robot into clear space, "
                        f"ENTER=run  s=skip  q=quit: "
                    )
                    .strip()
                    .lower()
                )
                if resp == "q":
                    raise KeyboardInterrupt("operator quit")
                if resp == "s":
                    print("  skipped")
                    continue

                # pre-roll zeros (settle + prime estimator)
                stream.reset_velocity()
                t_end = time.perf_counter() + pre_roll_s
                while time.perf_counter() < t_end:
                    publish(0.0, 0.0, 0.0)
                    time.sleep(hw_dt)

                # step. Ends on whichever comes first: travelled distance
                # >= max_dist (the real-space bound — at high speed the
                # time cap would run the robot out of the test area), or
                # t_rel > step_s (time safety cap; also the terminator for
                # wz, which spins in place and never accumulates distance).
                cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
                cmd[channel] = amp
                ts: list[float] = []
                ys: list[float] = []
                sp, _, _ = stream.latest()
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
                    p, pt, v = stream.latest()
                    if p is None or now - pt > profile.odom_stale_s:
                        print(f"  [abort] stale odom ({now - pt:.2f}s)")
                        end_reason = "stale"
                        break
                    dist = math.hypot(p.position.x - x0, p.position.y - y0)
                    if dist >= max_dist:
                        end_reason = "dist"
                        break
                    ts.append(t_rel)
                    ys.append(v[_CHANNELS.index(channel)])
                    time.sleep(hw_dt)
                safe_stop()

                if len(ys) < 5:
                    print(f"  [warn] {channel}@{amp}: too few samples, skip")
                    continue
                fp = fit_fopdt(np.asarray(ts), np.asarray(ys), u_step=amp)
                if not fp.converged or not np.isfinite([fp.K, fp.tau, fp.L]).all():
                    print(f"  [warn] {channel}@{amp}: fit failed ({fp.reason})")
                    continue
                print(
                    f"  {channel}@{amp}: K={fp.K:.3f} tau={fp.tau:.3f} "
                    f"L={fp.L:.3f}  ({len(ys)} samples, ended on {end_reason})"
                )
                fits.append(fp)
                per_amplitude[channel].append(
                    {"amplitude": amp, "direction": "forward", "K": fp.K, "tau": fp.tau, "L": fp.L}
                )
                traces.append(
                    {
                        "channel": channel,
                        "amp": amp,
                        "t": np.asarray(ts, dtype=float),
                        "y": np.asarray(ys, dtype=float),
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


def main() -> None:
    ap = argparse.ArgumentParser(description="Twist-base characterization -> tuning artifact")
    ap.add_argument("--robot", default="go2", help=f"one of {sorted(ROBOT_PLANT_PROFILES)}")
    ap.add_argument("--mode", choices=["hw", "self-test"], default="hw")
    ap.add_argument("--out", default=str(REPORTS_DIR))
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

    profile = _resolve_profile(args.robot)
    step_s = args.step_s if args.step_s is not None else profile.step_s
    pre_roll_s = args.pre_roll_s if args.pre_roll_s is not None else profile.pre_roll_s
    warmup_s = args.odom_warmup if args.odom_warmup is not None else profile.odom_warmup_s
    max_dist = args.max_dist if args.max_dist is not None else profile.max_dist_m
    robot_id = args.robot_id if args.robot_id is not None else profile.robot_id

    if args.mode == "hw":
        fitted, per_amplitude, traces = _fit_hw(profile, step_s, pre_roll_s, warmup_s, max_dist)
    else:
        fitted, per_amplitude, traces = _fit_selftest(profile)

    provenance = Provenance(
        robot_id=robot_id,
        surface=args.surface,
        mode=args.gait_mode,
        date=date.today().isoformat(),
        git_sha=git_sha(),
        sim_or_hw="hw" if args.mode == "hw" else "self-test",
        characterization_session_dir=(
            f"(real {profile.name}, LCM SI)" if args.mode == "hw" else "(in-process self-test)"
        ),
    )
    cfg = derive_config(
        fitted,
        provenance,
        per_amplitude=per_amplitude,
        vx_max=profile.vx_max,
        wz_max=profile.wz_max,
    )
    if args.mode == "hw" and "vy" not in profile.excited_channels:
        cfg.caveats.append(
            f"vy was NOT characterized on hardware ({profile.name} does not "
            "strafe in this gait); plant.vy / feedforward.K_vy are a "
            "placeholder copy of vx. The benchmark paths are vx+wz only, so "
            "this does not affect tuning; re-characterize vy if a "
            "lateral-capable gait is used."
        )

    out_path = (
        Path(args.out).expanduser()
        / f"{robot_id}_config_{args.mode}_{args.surface}_{provenance.date}_{provenance.git_sha}.json"
    )
    cfg.to_json(out_path)
    plot_path = out_path.with_suffix(".png")
    _plot_fits(traces, provenance, profile, plot_path)

    tag = "ROBOT-VALID" if cfg.valid_for_tuning else "NOT robot-valid (plumbing check)"
    print("\nFOPDT fit graph (the deliverable — model vs real data):")
    print(f"  {plot_path.resolve()}")
    print(f"Config artifact [{tag}] (machine handoff for the benchmark):")
    print(f"  {out_path.resolve()}")


if __name__ == "__main__":
    main()
