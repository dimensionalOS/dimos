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

"""Tool 2 of the twist-base tuning deliverable: operating-point benchmark.

Consumes the config artifact from ``characterization``, runs the stock
baseline P-controller (bare by default = the plant's physical tracking
limit; ``--ff`` / ``--profile`` are opt-in comparison arms) across a
speed ladder on a fixed real-space-constrained path set, scores each
(path, speed), and writes back the operating-point map +
tolerance->max-safe-speed inversion (artifact section 5). Robot-agnostic:
everything robot-specific comes from the ``RobotPlantProfile`` (``--robot``).

The tool talks to whichever operator coord is up via two well-known
topics: publishes Twist on ``/cmd_vel`` (the coord's ``twist_command``
In) and subscribes to ``/coordinator/joint_state`` (positions =
[x,y,yaw] from the adapter's read_odometry). Adding a robot = adding a
``RobotPlantProfile`` — no in-process coord, no new blueprint, no
per-robot topic dance.

    uv run python -m dimos.utils.benchmarking.benchmark \\
        --robot go2 --config reports/go2_config_hw_<...>.json --mode hw
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import threading
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from dimos.control.components import make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.feedforward_gain_compensator import FeedforwardGainConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.benchmarking.paths import circle, single_corner, square, straight_line
from dimos.utils.benchmarking.plant import ROBOT_PLANT_PROFILES, RobotPlantProfile
from dimos.utils.benchmarking.scoring import ExecutedTrajectory, TrajectoryTick, score_run
from dimos.utils.benchmarking.tuning import (
    OperatingPoint,
    OperatingPointMap,
    TuningConfig,
    invert_tolerance,
)
from dimos.utils.benchmarking.velocity_profile import VelocityProfileConfig

# Well-known topic the operator coord publishes its JointState Out on.
# Positions carry [x,y,yaw] (ConnectedTwistBase populates them from
# adapter.read_odometry). The tool subscribes to this for trajectory
# recording; the baseline task — which actually drives the robot — lives
# inside the operator coord (see TaskConfig type="baseline_path_follower"
# wired into each robot's coord blueprint).
_JOINT_STATE_TOPIC = "/coordinator/joint_state"
_BASELINE_TASK_NAME = "baseline_follower"  # task name in operator coord blueprint

_ARRIVED_STATES = frozenset({"arrived", "completed"})
_FAILED_STATES = frozenset({"aborted"})

REPORTS_DIR = Path(__file__).parent / "reports"


def _resolve_profile(name: str) -> RobotPlantProfile:
    try:
        return ROBOT_PLANT_PROFILES[name]
    except KeyError:
        raise SystemExit(
            f"unknown --robot {name!r}; known: {sorted(ROBOT_PLANT_PROFILES)}"
        ) from None


def _shift_path_to_start_at_pose(path: NavPath, start_pose: PoseStamped) -> NavPath:
    """Rigid-transform a robot-centric reference path into the odom frame
    anchored at the robot's current pose (so it need not be positioned
    precisely — only roughly aimed). Used in BOTH sim and hw so scoring
    is in the executed frame regardless of where the plant starts."""
    px0, py0 = path.poses[0].position.x, path.poses[0].position.y
    pyaw0 = path.poses[0].orientation.euler[2]
    sx, sy = start_pose.position.x, start_pose.position.y
    dyaw = start_pose.orientation.euler[2] - pyaw0
    cd, sd = math.cos(dyaw), math.sin(dyaw)
    new = []
    for p in path.poses:
        rx, ry = p.position.x - px0, p.position.y - py0
        new.append(
            PoseStamped(
                position=Vector3(sx + rx * cd - ry * sd, sy + rx * sd + ry * cd, 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, p.orientation.euler[2] + dyaw)),
            )
        )
    return NavPath(poses=new)


class _JointStateRecorder:
    """Subscribes to a coordinator's ``joint_state`` Out and turns each
    tick into a ``TrajectoryTick``. Recovers body-frame velocity by
    pose differentiation (``read_velocities`` returns last-commanded for
    ``transport_lcm``, not measured — same for hw GO2Connection and the
    sim FopdtPlantConnection). EMA-smoothed (alpha=0.5)."""

    def __init__(self, joint_names: list[str], alpha: float = 0.5) -> None:
        self._jx, self._jy, self._jyaw = joint_names
        self._alpha = alpha
        self._lock = threading.Lock()
        self._ticks: list[TrajectoryTick] = []
        self._first_pose: PoseStamped | None = None
        self._t0: float | None = None
        # diff state
        self._prev_pose: PoseStamped | None = None
        self._prev_t: float | None = None
        self._vx = self._vy = self._wz = 0.0
        # commanded telemetry: most recent JointState.velocity (the adapter's
        # last write) for this hardware's joints
        self._cmd_vx = self._cmd_vy = self._cmd_wz = 0.0

    def on_joint_state(self, msg: JointState) -> None:
        # ConnectedTwistBase publishes positions = odometry [x, y, yaw]
        # and velocities = last commanded (transport_lcm convention).
        # Caller waits a grace period after coord.start before sampling
        # the latest pose so the first /odom has time to propagate
        # through the adapter and one tick — that avoids latching onto
        # the [0,0,0] placeholder ConnectedTwistBase emits before the
        # adapter has seen any odom.
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

        # commanded telemetry (optional — used only to colour the recorded
        # cmd_twist column; behaviour is identical with or without it)
        if msg.velocity:
            try:
                self._cmd_vx = float(msg.velocity[idx[self._jx]])
                self._cmd_vy = float(msg.velocity[idx[self._jy]])
                self._cmd_wz = float(msg.velocity[idx[self._jyaw]])
            except (KeyError, IndexError):
                pass

        with self._lock:
            if self._first_pose is None:
                self._first_pose = pose
            if self._t0 is None:
                self._t0 = now
            t_rel = now - self._t0

            if self._prev_pose is None or self._prev_t is None:
                self._prev_pose, self._prev_t = pose, now
                self._ticks.append(
                    TrajectoryTick(
                        t=t_rel,
                        pose=pose,
                        cmd_twist=Twist(
                            linear=Vector3(self._cmd_vx, self._cmd_vy, 0.0),
                            angular=Vector3(0.0, 0.0, self._cmd_wz),
                        ),
                        actual_twist=Twist(
                            linear=Vector3(0.0, 0.0, 0.0),
                            angular=Vector3(0.0, 0.0, 0.0),
                        ),
                    )
                )
                return

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

            self._ticks.append(
                TrajectoryTick(
                    t=t_rel,
                    pose=pose,
                    cmd_twist=Twist(
                        linear=Vector3(self._cmd_vx, self._cmd_vy, 0.0),
                        angular=Vector3(0.0, 0.0, self._cmd_wz),
                    ),
                    actual_twist=Twist(
                        linear=Vector3(self._vx, self._vy, 0.0),
                        angular=Vector3(0.0, 0.0, self._wz),
                    ),
                )
            )

    def first_pose(self, timeout_s: float, grace_s: float = 0.5) -> PoseStamped:
        # Wait at minimum until coord+adapter have had time to receive a
        # first /odom and propagate it through one tick (otherwise we
        # latch onto the ConnectedTwistBase [0,0,0] placeholder). After
        # the grace period the latest pose is the real current one.
        time.sleep(grace_s)
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            with self._lock:
                if self._prev_pose is not None:
                    return self._prev_pose
            time.sleep(0.02)
        raise RuntimeError(f"no odom within {timeout_s:.1f}s")

    def snapshot(self) -> list[TrajectoryTick]:
        with self._lock:
            return list(self._ticks)

    def reset_trajectory(self) -> None:
        """Clear recorded ticks and the t=0 anchor — called before each
        run so each (path, speed) is scored on its own time axis."""
        with self._lock:
            self._ticks.clear()
            self._t0 = None


def _invoke(coord: RPCClient, method: str, **kwargs: object) -> object:
    """RPC `task_invoke(_BASELINE_TASK_NAME, method, kwargs)` on the operator
    coord. Centralises the .task_invoke wrapping so the run loop reads as
    plain method calls on a remote object."""
    return coord.task_invoke(
        task_name=_BASELINE_TASK_NAME,
        method=method,
        kwargs=dict(kwargs),
    )


def _run_baseline(
    profile: RobotPlantProfile,
    coord: RPCClient,
    recorder: _JointStateRecorder,
    path: NavPath,
    speed: float,
    k_angular: float,
    ff_config: FeedforwardGainConfig | None,
    profile_config: VelocityProfileConfig | None,
    timeout_s: float,
    label: str,
) -> tuple[ExecutedTrajectory, NavPath]:
    """Send a path to the operator coord's ``baseline_follower`` task and
    wait for it to terminate. The task is pre-added by the operator's
    blueprint (priority 20, claims base/{vx,vy,wz}) so it preempts the
    operator's teleop velocity task while a run is active. We only RPC
    configure/start/cancel; the coord owns the tick-loop compute and the
    adapter that drives the robot. ``ff_config``/``profile_config`` are
    optional arms (``None`` = bare = the physical-limit measurement).

    Path is anchored to the robot's first observed pose so the operator
    only has to roughly aim the robot. Returns the executed trajectory
    (recorded from /coordinator/joint_state) and the anchored reference."""
    pose0 = recorder.first_pose(timeout_s=profile.odom_warmup_s)
    path_w = _shift_path_to_start_at_pose(path, pose0)

    # Reset accumulated trajectory so each run starts at t=0.
    recorder.reset_trajectory()

    if not _invoke(
        coord,
        "configure",
        speed=speed,
        k_angular=k_angular,
        ff_config=ff_config,
        velocity_profile_config=profile_config,
    ):
        print(f"  [{label}] configure rejected — task still active from prior run?")
        return ExecutedTrajectory(ticks=recorder.snapshot(), arrived=False), path_w

    if not _invoke(coord, "start_path", path=path_w, current_odom=pose0):
        print(f"  [{label}] start_path rejected")
        return ExecutedTrajectory(ticks=recorder.snapshot(), arrived=False), path_w

    arrived = False
    t_start = time.perf_counter()
    deadline = t_start + timeout_s
    terminated = False
    try:
        while time.perf_counter() < deadline:
            st = _invoke(coord, "get_state")
            if st in _ARRIVED_STATES:
                arrived = True
                terminated = True
                print(f"  [{label}] arrived in {time.perf_counter() - t_start:.1f}s")
                break
            if st in _FAILED_STATES:
                terminated = True
                print(f"  [{label}] task aborted (state={st})")
                break
            time.sleep(0.05)
        if not terminated:
            print(f"  [{label}] timeout {timeout_s:.0f}s — cancelling")
    finally:
        # Best-effort cancel; safe to ignore if already terminal.
        try:
            _invoke(coord, "cancel")
        except Exception:
            pass
    return ExecutedTrajectory(ticks=recorder.snapshot(), arrived=arrived), path_w


# --- benchmark ----------------------------------------------------------


def _path_set() -> dict:
    """Real-space-constrained fixed path set (locked — do not widen)."""
    return {
        "straight_line": straight_line(),
        "single_corner": single_corner(leg_length=2.0, angle_deg=90.0),
        "square": square(side=2.0),
        "circle": circle(radius=1.0),
    }


def _run_ladder(
    cfg: TuningConfig,
    profile: RobotPlantProfile,
    speeds: list[float],
    timeout_s: float,
    mode: str,
    use_ff: bool,
    use_profile: bool,
) -> tuple[list[OperatingPoint], list[dict]]:
    # Bare stock baseline by default: this is the physical-limit
    # measurement. FF / velocity profile are opt-in comparison arms.
    ff = cfg.feedforward.to_runtime() if use_ff else None
    k_angular = float(cfg.recommended_controller.params.get("k_angular", 0.5))

    # Long-lived: RPC client to the operator's coord + LCM subscription
    # to /coordinator/joint_state. Shared across all (path, speed) runs.
    coord_rpc: RPCClient = RPCClient(None, ControlCoordinator)
    joints = make_twist_base_joints(profile.joints_prefix)
    recorder = _JointStateRecorder(joint_names=joints)
    js_sub = LCMTransport(_JOINT_STATE_TOPIC, JointState)
    js_unsub = js_sub.subscribe(recorder.on_joint_state)

    points: list[OperatingPoint] = []
    runs: list[dict] = []  # for the XY trajectory overlay
    try:
        for name, path in _path_set().items():
            for speed in speeds:
                prof_cfg = (
                    cfg.velocity_profile.to_runtime(max_linear_speed=speed) if use_profile else None
                )
                if mode == "hw":
                    resp = (
                        input(
                            f"\n[{name} v={speed:.2f}] reposition+aim robot, "
                            f"ENTER=run  s=skip  q=quit: "
                        )
                        .strip()
                        .lower()
                    )
                    if resp == "q":
                        raise KeyboardInterrupt
                    if resp == "s":
                        print("  skipped")
                        continue
                traj, ref = _run_baseline(
                    profile,
                    coord_rpc,
                    recorder,
                    path,
                    speed,
                    k_angular,
                    ff,
                    prof_cfg,
                    timeout_s,
                    f"{name}@{speed:.2f}",
                )
                # Score/plot against the executed-frame reference (anchored path).
                s = score_run(ref, traj)
                points.append(
                    OperatingPoint(
                        path=name,
                        speed=speed,
                        cte_max=s.cte_max,
                        cte_rms=s.cte_rms,
                        arrived=s.arrived,
                    )
                )
                runs.append(
                    {
                        "path": name,
                        "speed": speed,
                        "cte_max": s.cte_max,
                        "arrived": s.arrived,
                        "ref": [(p.position.x, p.position.y) for p in ref.poses],
                        "exec": [(tk.pose.position.x, tk.pose.position.y) for tk in traj.ticks],
                    }
                )
                print(
                    f"  {name:14} v={speed:.2f}  cte_max={s.cte_max * 100:6.1f}cm  "
                    f"cte_rms={s.cte_rms * 100:6.1f}cm  arrived={s.arrived}"
                )
    finally:
        js_unsub()
        coord_rpc.stop_rpc_client()
    return points, runs


def _plot(points: list[OperatingPoint], out: Path, robot_name: str, arm: str) -> None:
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for name in sorted({p.path for p in points}):
        xs = [p.speed for p in points if p.path == name]
        ys = [p.cte_max * 100 for p in points if p.path == name]
        ax.plot(xs, ys, marker="o", label=name)
    ax.set_xlabel("commanded speed (m/s)")
    ax.set_ylabel("cte_max (cm)")
    label = "BARE baseline (physical limit)" if arm == "bare" else f"baseline+{arm}"
    ax.set_title(f"{robot_name} {label}: cross-track error vs speed")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def _canonicalize(ref: list, exec_: list) -> tuple[list, list]:
    """Rigid-transform a run into the canonical path frame: reference
    start -> (0,0), initial heading -> +x. The same transform is applied
    to the executed trajectory. Makes every run of a path overlay on one
    identical reference sharing the origin — so speeds are comparable
    regardless of where the robot physically started (paths are anchored
    to the robot's first odom pose, which differs between runs)."""
    if len(ref) < 2:
        return ref, exec_
    ox, oy = ref[0]
    # heading from the first reference point that is meaningfully distinct
    th = 0.0
    for px, py in ref[1:]:
        if math.hypot(px - ox, py - oy) > 1e-6:
            th = math.atan2(py - oy, px - ox)
            break
    c, s = math.cos(-th), math.sin(-th)

    def tf(pts):
        out = []
        for x, y in pts:
            dx, dy = x - ox, y - oy
            out.append((dx * c - dy * s, dx * s + dy * c))
        return out

    return tf(ref), tf(exec_)


def _plot_xy(runs: list[dict], out: Path, robot_name: str, arm: str) -> None:
    """One subplot per path: the reference path (black) overlaid with the
    executed trajectory at each speed, all normalized to the canonical
    path frame (common origin) so speeds are directly comparable. This is
    the diagnostic view — you see exactly where/how the robot cuts
    corners as speed rises."""
    if not runs:
        return
    paths = list(dict.fromkeys(r["path"] for r in runs))
    n = len(paths)
    cols = min(n, 2)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(6.0 * cols, 5.0 * rows), squeeze=False)
    flat = [ax for row in axes for ax in row]
    for ax, name in zip(flat, paths, strict=False):
        prs = [r for r in runs if r["path"] == name]
        ref_drawn = False
        for r in prs:
            ref_c, ex_c = _canonicalize(r["ref"], r["exec"])
            if not ref_drawn:
                ax.plot(
                    [p[0] for p in ref_c],
                    [p[1] for p in ref_c],
                    "k-",
                    lw=2.0,
                    label="reference",
                )
                ax.plot(0.0, 0.0, "ko", ms=5)  # common start
                ref_drawn = True
            if not ex_c:
                continue
            ax.plot(
                [p[0] for p in ex_c],
                [p[1] for p in ex_c],
                lw=1.3,
                label=f"v={r['speed']:g} (cte_max={r['cte_max'] * 100:.0f}cm"
                f"{'' if r['arrived'] else ', NOT arrived'})",
            )
        ax.set_title(name)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    for ax in flat[n:]:
        ax.set_visible(False)
    label = "BARE baseline (physical limit)" if arm == "bare" else f"baseline+{arm}"
    fig.suptitle(f"{robot_name} {label}: executed trajectory vs reference path")
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def _prereq_banner(profile: RobotPlantProfile, mode: str) -> None:
    if mode == "hw":
        bp = profile.blueprint
        kind = "HARDWARE"
    else:
        bp = profile.sim_blueprint
        kind = "SIM"
    print(
        f"\n=== {kind} MODE ({profile.name}) ===\n"
        f"Prereqs:\n"
        f"  1. Another terminal: `dimos run {bp}`\n"
        f"     (its ControlCoordinator publishes {_JOINT_STATE_TOPIC} and\n"
        f"     hosts the `{_BASELINE_TASK_NAME}` task at priority 20).\n"
        f"  2. This process: strip /nix/store from LD_LIBRARY_PATH (README).\n"
        f"Each (path,speed): reposition+aim, then ENTER. We RPC the operator\n"
        f"coord to configure + start_path; the task drives the robot from\n"
        f"inside its tick loop. Coord ticks at {profile.tick_rate_hz:g}Hz.\n"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="Twist-base operating-point benchmark")
    ap.add_argument("--robot", default="go2", help=f"one of {sorted(ROBOT_PLANT_PROFILES)}")
    ap.add_argument("--config", required=True, help="config artifact from characterization")
    ap.add_argument("--mode", choices=["hw", "sim"], default="hw")
    ap.add_argument("--speeds", default="0.3,0.5,0.7,0.9,1.0")
    ap.add_argument("--tolerances", default="5,10,15", help="cm")
    ap.add_argument("--timeout", type=float, default=60.0, help="per (path,speed) run timeout (s)")
    ap.add_argument(
        "--ff",
        action="store_true",
        help="OPT-IN arm: apply the artifact's derived feedforward "
        "(default OFF — bare stock baseline = the physical-limit measurement)",
    )
    ap.add_argument(
        "--profile",
        action="store_true",
        help="OPT-IN arm: apply the artifact's derived curvature velocity "
        "profile (default OFF — bare stock baseline)",
    )
    args = ap.parse_args()

    profile = _resolve_profile(args.robot)
    config_path = Path(args.config).expanduser()
    cfg = TuningConfig.from_json(config_path)  # asserts schema_version
    speeds = [float(s) for s in args.speeds.split(",")]
    tolerances = [float(t) for t in args.tolerances.split(",")]
    arm = "+".join(x for x, on in (("ff", args.ff), ("profile", args.profile)) if on) or "bare"

    # The sim-derived ff/profile are only meaningless on the real robot
    # if you actually apply them; the bare baseline doesn't use them.
    if args.mode == "hw" and (args.ff or args.profile) and not cfg.valid_for_tuning:
        sys.exit(
            f"Refusing --mode hw with --{arm} and a non-robot-valid config "
            f"({config_path.name}, sim_or_hw={cfg.provenance.sim_or_hw!r}): its "
            "feedforward/profile were derived from the sim plant. Re-run "
            "`characterization --mode hw` first, drop --ff/--profile for "
            "the bare physical-limit run, or use --mode sim."
        )
    if args.mode == "sim":
        print(
            "[pre-check] --mode sim: validates wiring against the FOPDT sim "
            "plant only; the operating-point map is NOT a real-robot result."
        )

    _prereq_banner(profile, args.mode)

    arm_desc = (
        "BARE stock baseline (no FF, no profile) — the plant's physical tracking limit"
        if arm == "bare"
        else f"baseline + {arm} (comparison arm, vs the bare physical limit)"
    )
    print(
        f"{profile.name} {args.mode} speed ladder {speeds} over {len(_path_set())} paths\n"
        f"  controller: {arm_desc}\n"
        f"  k_angular={cfg.recommended_controller.params.get('k_angular')}"
    )
    try:
        points, runs = _run_ladder(
            cfg,
            profile,
            speeds,
            args.timeout,
            args.mode,
            use_ff=args.ff,
            use_profile=args.profile,
        )
    except KeyboardInterrupt:
        raise SystemExit(
            "\n[hw] aborted by operator — robot stopped, artifact not modified."
        ) from None
    inversion = invert_tolerance(points, tolerances)
    opm = OperatingPointMap(speeds=speeds, points=points, tolerance_inversion=inversion)

    sha = cfg.provenance.git_sha
    rid = cfg.provenance.robot_id
    # Only the BARE run defines section 5 (the canonical physical-limit
    # operating-point map). Comparison arms emit standalone artifacts so
    # they never clobber the physical-limit map in the config.
    if arm == "bare":
        cfg.operating_point_map = opm
        cfg.to_json(config_path)
        artifact_msg = f"Augmented artifact (section 5 = physical limit): {config_path.resolve()}"
    else:
        artifact_msg = (
            f"Config NOT modified (arm '{arm}' is a comparison, not the "
            f"physical-limit map). See standalone outputs below."
        )
    bench_path = REPORTS_DIR / f"{rid}_benchmark_{arm}_{sha}.json"
    bench_path.parent.mkdir(parents=True, exist_ok=True)
    bench_path.write_text(json.dumps(asdict(opm), indent=2))
    plot_path = REPORTS_DIR / f"{rid}_benchmark_cte_vs_speed_{arm}_{sha}.png"
    _plot(points, plot_path, profile.name, arm)
    xy_path = REPORTS_DIR / f"{rid}_benchmark_xy_{arm}_{sha}.png"
    _plot_xy(runs, xy_path, profile.name, arm)

    print(f"\n{artifact_msg}")
    print(f"Benchmark json     : {bench_path.resolve()}")
    print(f"CTE-vs-speed plot  : {plot_path.resolve()}")
    print(f"XY trajectory plot : {xy_path.resolve()}  <-- the diagnostic view")
    print("\nOperating-point recommendation:")
    for row in inversion:
        if row.max_speed is None:
            print(
                f"  tolerance {row.tol_cm:g} cm: NO tested speed keeps every "
                f"path within tolerance — relax the tolerance or slow the fleet."
            )
        else:
            print(
                f"  For tolerance {row.tol_cm:g} cm, run at speed "
                f"{row.max_speed:.2f} m/s with this profile "
                f"(binding path: {row.binding_path})."
            )


if __name__ == "__main__":
    main()
