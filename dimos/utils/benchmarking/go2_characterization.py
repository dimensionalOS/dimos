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

"""Tool 1 of the Go2 tuning deliverable: characterization.

**This is a hardware tool.** It measures the real Go2's per-axis velocity
response (vx, vy, wz), fits FOPDT per channel, runs the DERIVE step, and
emits the versioned config artifact.

    # On dimensional-gpu-0, terminal 1:
    dimos run unitree-go2-webrtc-keyboard-teleop
    # terminal 2 (strip /nix/store from LD_LIBRARY_PATH — see README):
    uv run python -m dimos.utils.benchmarking.go2_characterization \\
        --mode hw --surface concrete

`--mode hw` (default) drives the real robot over LCM (`/cmd_vel` out,
`/go2/odom` in). It is **operator-gated**: before every step it stops the
robot and waits for you to reposition it (with the keyboard teleop from
the blueprint above) and press ENTER. This bounds drift to a single step
and is safe (velocity clamp, zero-Twist on exit/SIGINT, stale-odom
abort, timeout).

`--mode self-test` is a **plumbing check, NOT a tuning artifact**: it
steps an in-process FOPDT `Go2PlantSim` seeded with the vendored
ground truth and recovers it. It only proves the measure->fit->derive
code runs; the artifact is stamped `valid_for_tuning=false`. Used by
pytest/CI so regressions are caught without a robot.
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

from dimos.utils.benchmarking.go2_tuning import Provenance, derive_config, git_sha
from dimos.utils.benchmarking.plant import (
    GO2_PLANT_FITTED,
    FopdtChannelParams,
    Go2PlantParams,
    Go2PlantSim,
)
from dimos.utils.characterization.modeling.fopdt import fit_fopdt, fopdt_step_response

# Space-cheap SI plan: a few amplitudes per channel. vy is a real channel
# (the Go2 base strafes) so it gets its own sweep — not a copy of vx.
_SI_AMPLITUDES: dict[str, list[float]] = {
    "vx": [0.3, 0.6, 0.9],
    "vy": [0.2, 0.4],
    "wz": [0.4, 0.8, 1.2],
}
_CHANNELS = ("vx", "vy", "wz")  # velocity-tuple order (estimator output)
# Channels excited on the real robot. vy is omitted: the Go2 does not
# strafe on /cmd_vel.linear.y in the default gait, so exciting it only
# produces a degenerate fit. vy is placeholdered (= vx) post-hoc.
_HW_CHANNELS = ("vx", "wz")
_PRE_ROLL_S = 1.0
# Real-robot default: gait initiation + command latency means the Go2
# needs several seconds to ramp to and hold the commanded velocity, much
# longer than the bare FOPDT settle. Operator-overridable (--step-s).
_STEP_S = 8.0
# Per-step travel cap (m). At high speed the time cap would run the
# robot out of the test area, so distance is the real bound; step_s is
# the safety upper bound and the terminator for wz (spins in place).
_MAX_DIST_M = 6.0

# Hardware safety envelope (Go2 Rung-1 saturation) + control rate.
VX_MAX = 1.0  # m/s
WZ_MAX = 1.5  # rad/s
_HW_DT = 1.0 / 10.0  # Go2 control + odom tick
_SIM_DT = 0.02
_ODOM_WARMUP_S = 10.0  # WebRTC connect + first odom (override: --odom-warmup)
_ODOM_STALE_S = 1.0

REPORTS_DIR = Path(__file__).parent / "reports"


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# --- self-test (in-process FOPDT plant; NOT robot-valid) -----------------


def _fit_selftest() -> tuple[Go2PlantParams, dict, list[dict]]:
    """Step the vendored FOPDT sim plant and recover it. Plumbing check
    only — proves the measure->fit->derive code path runs."""
    plant = Go2PlantSim(GO2_PLANT_FITTED)
    n_pre = int(_PRE_ROLL_S / _SIM_DT)
    n_step = int(_STEP_S / _SIM_DT)
    pooled: dict[str, FopdtChannelParams] = {}
    per_amplitude: dict[str, list[dict]] = {}
    traces: list[dict] = []

    for channel in _CHANNELS:
        fits = []
        per_amplitude[channel] = []
        for amp in _SI_AMPLITUDES[channel]:
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
    fitted = Go2PlantParams(vx=pooled["vx"], vy=pooled["vy"], wz=pooled["wz"])
    print("\nself-test (recovered vs injected FOPDT ground truth):")
    print(f"  {'chan':4} {'K fit/true':>20} {'tau fit/true':>20} {'L fit/true':>20}")
    for ch in _CHANNELS:
        f, g = getattr(fitted, ch), getattr(GO2_PLANT_FITTED, ch)
        print(
            f"  {ch:4} {f.K:8.3f}/{g.K:<8.3f}   {f.tau:8.3f}/{g.tau:<8.3f}   {f.L:8.3f}/{g.L:<8.3f}"
        )
    return fitted, per_amplitude, traces


# --- fit-quality graph (the human-facing deliverable) -------------------


def _plot_fits(traces: list[dict], provenance: Provenance, out: Path) -> None:
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
        f"Go2 FOPDT characterization — {p.robot_id} / {p.surface} / "
        f"{p.mode} / {p.sim_or_hw} — {p.date} ({p.git_sha})"
    )
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


# --- hardware SI (real Go2 over LCM, operator-gated, safe) ---------------


class _PoseVelocityEstimator:
    """Differentiate consecutive ``PoseStamped`` to body-frame (vx,vy,wz);
    EMA-smoothed (Go2 publishes pose only). Ported from the R&D hw loop."""

    def __init__(self, alpha: float = 0.5) -> None:
        self._pp = None
        self._pt: float | None = None
        self._vx = self._vy = self._wz = 0.0
        self._a = alpha

    def update(self, pose, t: float) -> tuple[float, float, float]:
        if self._pp is None or self._pt is None:
            self._pp, self._pt = pose, t
            return 0.0, 0.0, 0.0
        dt = t - self._pt
        if dt <= 0:
            return self._vx, self._vy, self._wz
        dx = pose.position.x - self._pp.position.x
        dy = pose.position.y - self._pp.position.y
        y0, y1 = self._pp.orientation.euler[2], pose.orientation.euler[2]
        dyaw = (y1 - y0 + math.pi) % (2 * math.pi) - math.pi
        yaw = y1
        c, s = math.cos(yaw), math.sin(yaw)
        bx = (dx / dt) * c + (dy / dt) * s
        by = -(dx / dt) * s + (dy / dt) * c
        bw = dyaw / dt
        self._vx = self._a * bx + (1 - self._a) * self._vx
        self._vy = self._a * by + (1 - self._a) * self._vy
        self._wz = self._a * bw + (1 - self._a) * self._wz
        self._pp, self._pt = pose, t
        return self._vx, self._vy, self._wz


def _prereq_banner() -> None:
    print(
        "\n=== HARDWARE MODE ===\n"
        "Prereqs (run on dimensional-gpu-0):\n"
        "  1. Another terminal: `dimos run unitree-go2-webrtc-keyboard-teleop`\n"
        "     (publishes /go2/odom, consumes /cmd_vel; its teleop is\n"
        "     publish-only-when-active so it stays silent while idle and\n"
        "     does NOT fight the SI commands).\n"
        "  2. This process: strip /nix/store from LD_LIBRARY_PATH (see README)\n"
        "Robot is STOPPED before every step. Reposition it with the keyboard\n"
        "teleop (WASD/QE), then RELEASE all keys (teleop goes silent) and\n"
        "press ENTER here — the tool then owns /cmd_vel for the step.\n"
        "Each step ends at --max-dist travelled (default 6 m) or --step-s,\n"
        "whichever first. Velocity clamped; zero-Twist on exit / Ctrl-C.\n"
    )


def _fit_hw(
    step_s: float, pre_roll_s: float, warmup_s: float, max_dist: float
) -> tuple[Go2PlantParams, dict, list[dict]]:
    from dimos.core.transport import LCMTransport
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Twist import Twist
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    _prereq_banner()
    cmd_pub = LCMTransport("/cmd_vel", Twist)
    odom_sub = LCMTransport("/go2/odom", PoseStamped)
    lock = threading.Lock()
    box: dict = {"pose": None, "t": 0.0}

    def _on_odom(msg) -> None:
        with lock:
            box["pose"] = msg
            box["t"] = time.perf_counter()

    odom_sub.subscribe(_on_odom)

    def publish(vx: float, vy: float, wz: float) -> None:
        cmd_pub.broadcast(
            None,
            Twist(
                linear=Vector3(_clamp(vx, -VX_MAX, VX_MAX), 0.0, 0.0),
                angular=Vector3(0.0, 0.0, _clamp(wz, -WZ_MAX, WZ_MAX)),
            ),
        )

    def safe_stop() -> None:
        for _ in range(3):
            publish(0.0, 0.0, 0.0)
            time.sleep(0.05)

    # No custom SIGINT handler: Ctrl-C must raise KeyboardInterrupt so it
    # also breaks out of the blocking input() prompt. The try/finally
    # below guarantees a zero-Twist stop on any exit (Ctrl-C, q, error).

    # odom warmup
    print(f"[hw] waiting up to {warmup_s:.0f}s for /go2/odom ...")
    deadline = time.perf_counter() + warmup_s
    while time.perf_counter() < deadline:
        with lock:
            if box["pose"] is not None:
                break
        time.sleep(0.05)
    with lock:
        if box["pose"] is None:
            safe_stop()
            raise SystemExit("No /go2/odom — is `dimos run unitree-go2-webrtc-keyboard-teleop` up?")

    pooled: dict[str, FopdtChannelParams] = {}
    per_amplitude: dict[str, list[dict]] = {}
    traces: list[dict] = []
    try:
        for channel in _HW_CHANNELS:
            fits = []
            per_amplitude[channel] = []
            for amp in _SI_AMPLITUDES[channel]:
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

                est = _PoseVelocityEstimator()
                # pre-roll zeros (settle + prime estimator)
                t_end = time.perf_counter() + pre_roll_s
                while time.perf_counter() < t_end:
                    publish(0.0, 0.0, 0.0)
                    with lock:
                        p = box["pose"]
                    est.update(p, time.perf_counter())
                    time.sleep(_HW_DT)

                # step. Ends on whichever comes first: travelled distance
                # >= max_dist (the real-space bound — at high speed the
                # time cap would run the robot out of the test area), or
                # t_rel > step_s (time safety cap; also the terminator for
                # wz, which spins in place and never accumulates distance).
                cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
                cmd[channel] = amp
                ts: list[float] = []
                ys: list[float] = []
                with lock:
                    sp = box["pose"]
                x0, y0 = sp.position.x, sp.position.y
                t0 = time.perf_counter()
                end_reason = "time"
                while True:
                    now = time.perf_counter()
                    t_rel = now - t0
                    if t_rel > step_s:
                        break
                    publish(cmd["vx"], cmd["vy"], cmd["wz"])
                    with lock:
                        p, pt = box["pose"], box["t"]
                    if p is None or now - pt > _ODOM_STALE_S:
                        print(f"  [abort] stale odom ({now - pt:.2f}s)")
                        end_reason = "stale"
                        break
                    dist = math.hypot(p.position.x - x0, p.position.y - y0)
                    if dist >= max_dist:
                        end_reason = "dist"
                        break
                    v = est.update(p, now)
                    ts.append(t_rel)
                    ys.append(v[_CHANNELS.index(channel)])
                    time.sleep(_HW_DT)
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
        safe_stop()
        raise SystemExit(
            "\n[hw] aborted by operator — robot stopped, no artifact written."
        ) from None
    finally:
        safe_stop()

    # vy is NOT excited on hardware: the real Go2 (default gait) does not
    # strafe on /cmd_vel.linear.y, so a vy step yields a degenerate K≈0
    # fit that would corrupt the model. Placeholder vy = vx (sane FF /
    # profile); flagged in the artifact caveats.
    pooled["vy"] = pooled["vx"]
    per_amplitude["vy"] = []
    print("  [note] vy not excited on hw (no lateral motion) — placeholder vy = vx")
    return (
        Go2PlantParams(vx=pooled["vx"], vy=pooled["vy"], wz=pooled["wz"]),
        per_amplitude,
        traces,
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="Go2 characterization -> tuning config artifact")
    ap.add_argument("--mode", choices=["hw", "self-test"], default="hw")
    ap.add_argument("--out", default=str(REPORTS_DIR))
    ap.add_argument("--robot-id", default="go2")
    ap.add_argument("--surface", default="concrete")
    ap.add_argument("--gait-mode", default="default")
    ap.add_argument(
        "--step-s",
        type=float,
        default=_STEP_S,
        help="per-step excitation duration (s); the robot must reach "
        "and hold the commanded speed within this window",
    )
    ap.add_argument(
        "--pre-roll-s",
        type=float,
        default=_PRE_ROLL_S,
        help="zero-command settle before each step (s)",
    )
    ap.add_argument(
        "--odom-warmup",
        type=float,
        default=_ODOM_WARMUP_S,
        help="how long to wait for first /go2/odom (s)",
    )
    ap.add_argument(
        "--max-dist",
        type=float,
        default=_MAX_DIST_M,
        help="per-step travel cap (m); ends the step early at high speed "
        "so the robot stays in the test area",
    )
    args = ap.parse_args()

    if args.mode == "hw":
        fitted, per_amplitude, traces = _fit_hw(
            args.step_s, args.pre_roll_s, args.odom_warmup, args.max_dist
        )
    else:
        fitted, per_amplitude, traces = _fit_selftest()

    provenance = Provenance(
        robot_id=args.robot_id,
        surface=args.surface,
        mode=args.gait_mode,
        date=date.today().isoformat(),
        git_sha=git_sha(),
        sim_or_hw="hw" if args.mode == "hw" else "self-test",
        characterization_session_dir=(
            "(real Go2, LCM SI)" if args.mode == "hw" else "(in-process self-test)"
        ),
    )
    cfg = derive_config(fitted, provenance, per_amplitude=per_amplitude)
    if args.mode == "hw":
        cfg.caveats.append(
            "vy was NOT characterized on hardware (the Go2 does not strafe "
            "on /cmd_vel.linear.y in this gait); plant.vy / feedforward.K_vy "
            "are a placeholder copy of vx. The benchmark paths are vx+wz "
            "only, so this does not affect tuning; re-characterize vy if a "
            "lateral-capable gait is used."
        )

    out_path = (
        Path(args.out).expanduser()
        / f"go2_config_{args.mode}_{args.surface}_{provenance.date}_{provenance.git_sha}.json"
    )
    cfg.to_json(out_path)
    plot_path = out_path.with_suffix(".png")
    _plot_fits(traces, provenance, plot_path)

    tag = "ROBOT-VALID" if cfg.valid_for_tuning else "NOT robot-valid (plumbing check)"
    print("\nFOPDT fit graph (the deliverable — model vs real data):")
    print(f"  {plot_path.resolve()}")
    print(f"Config artifact [{tag}] (machine handoff for the benchmark):")
    print(f"  {out_path.resolve()}")


if __name__ == "__main__":
    main()
