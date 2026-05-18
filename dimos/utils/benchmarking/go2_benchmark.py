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

"""Tool 2 of the Go2 tuning deliverable: the operating-point benchmark.

Consumes the config artifact from ``go2_characterization``, runs the
HARDCODED baseline P-controller with the derived feedforward + velocity
profile across a speed ladder on a fixed real-space-constrained path set,
scores each (path, speed), and writes back the operating-point map +
tolerance->max-safe-speed inversion (artifact section 5).

    uv run python -m dimos.utils.benchmarking.go2_benchmark \\
        --config reports/go2_config_sim_mujoco_<date>_<sha>.json

The sim harness (the baseline controller driven through a real
``ControlCoordinator`` + the FOPDT ``Go2SimTwistBaseAdapter``) is inlined
below — it is small, baseline-only, and used by nothing else.
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
from typing import TYPE_CHECKING, Protocol

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.task import (
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    JointStateSnapshot,
)
from dimos.control.tasks.baseline_path_follower_task import (
    BaselinePathFollowerTask,
    BaselinePathFollowerTaskConfig,
)
from dimos.control.tasks.feedforward_gain_compensator import FeedforwardGainConfig
from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.utils.benchmarking.go2_tuning import (
    Go2TuningConfig,
    OperatingPoint,
    OperatingPointMap,
    invert_tolerance,
)
from dimos.utils.benchmarking.paths import circle, single_corner, square, straight_line
from dimos.utils.benchmarking.scoring import ExecutedTrajectory, TrajectoryTick, score_run
from dimos.utils.benchmarking.velocity_profile import PathSpeedCap, VelocityProfileConfig

if TYPE_CHECKING:
    from dimos.hardware.drive_trains.go2_sim.adapter import Go2SimTwistBaseAdapter

# Go2 hardware control rate.
GO2_TICK_RATE_HZ = 10.0
_base_joints = make_twist_base_joints("base")
_ARRIVED_STATES = frozenset({"arrived", "completed"})
_FAILED_STATES = frozenset({"aborted"})


# --- inlined baseline sim harness (was runner.py + sim_blueprint.py) -----


class _PathFollowerLike(Protocol):
    def start_path(self, path: NavPath, current_odom: PoseStamped) -> bool: ...
    def update_odom(self, odom: PoseStamped) -> None: ...
    def compute(self, state) -> object: ...
    def get_state(self) -> str: ...


class _VelocityProfileProxyTask:
    """Curvature velocity-profile cap (the DERIVE consumer seam). Caps
    commanded ``|vx|`` to the profile at the robot's path index, scaling
    ``wz`` to preserve geometry; pure pass-through otherwise. No
    control-law change."""

    def __init__(self, inner: _PathFollowerLike, cap: PathSpeedCap) -> None:
        self._inner = inner
        self._cap = cap
        self._xy = (0.0, 0.0)

    @property
    def name(self) -> str:
        return self._inner.name

    def claim(self):
        return self._inner.claim()

    def is_active(self) -> bool:
        return self._inner.is_active()

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        self._inner.on_preempted(by_task, joints)

    def on_buttons(self, *a, **k):
        return self._inner.on_buttons(*a, **k)

    def on_cartesian_command(self, *a, **k):
        return self._inner.on_cartesian_command(*a, **k)

    def set_target_by_name(self, *a, **k):
        return self._inner.set_target_by_name(*a, **k)

    def set_velocities_by_name(self, *a, **k):
        return self._inner.set_velocities_by_name(*a, **k)

    def get_state(self) -> str:
        return self._inner.get_state()

    def update_odom(self, odom: PoseStamped) -> None:
        self._xy = (float(odom.position.x), float(odom.position.y))
        self._inner.update_odom(odom)

    def start_path(self, path: NavPath, current_odom: PoseStamped) -> bool:
        self._cap.for_path(path)
        self._xy = (float(current_odom.position.x), float(current_odom.position.y))
        return self._inner.start_path(path, current_odom)

    def compute(self, state):
        out = self._inner.compute(state)
        if out is None or out.mode != ControlMode.VELOCITY or out.velocities is None:
            return out
        vx, vy, wz = ([*out.velocities, 0.0, 0.0, 0.0])[:3]
        cx, cy, cz = self._cap.cap(self._xy[0], self._xy[1], vx, vy, wz)
        return JointCommandOutput(
            joint_names=out.joint_names,
            velocities=[cx, cy, cz],
            mode=ControlMode.VELOCITY,
        )


def _go2_sim_base() -> HardwareComponent:
    return HardwareComponent(
        hardware_id="base",
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints("base"),
        adapter_type="go2_sim_twist_base",
    )


def _odom_to_pose(odom: list[float]) -> PoseStamped:
    return PoseStamped(
        position=Vector3(odom[0], odom[1], 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, odom[2])),
    )


def _vels_to_twist(v: list[float]) -> Twist:
    return Twist(linear=Vector3(v[0], v[1], 0.0), angular=Vector3(0.0, 0.0, v[2]))


def _run_baseline_sim(
    path: NavPath,
    speed: float,
    k_angular: float,
    ff_config: FeedforwardGainConfig,
    profile_config: VelocityProfileConfig,
    timeout_s: float,
) -> tuple[ExecutedTrajectory, NavPath]:
    """Production baseline P-controller in sim against the FOPDT
    ``Go2SimTwistBaseAdapter``, with the derived FF + curvature profile.
    Returns the trajectory and the reference path in the executed frame
    (sim runs in the path's own frame, so it is ``path`` unchanged)."""
    coord = ControlCoordinator(
        tick_rate=GO2_TICK_RATE_HZ,
        hardware=[_go2_sim_base()],
        tasks=[
            TaskConfig(name="vel_base", type="velocity", joint_names=_base_joints, priority=10),
        ],
    )

    def _make() -> _PathFollowerLike:
        base = BaselinePathFollowerTask(
            name="baseline_follower",
            config=BaselinePathFollowerTaskConfig(
                speed=speed, k_angular=k_angular, ff_config=ff_config
            ),
            global_config=global_config,
        )
        return _VelocityProfileProxyTask(base, PathSpeedCap(profile_config))

    coord.start()
    try:
        adapter: Go2SimTwistBaseAdapter = coord._hardware["base"].adapter
        start = path.poses[0]
        adapter.set_initial_pose(start.position.x, start.position.y, start.orientation.euler[2])
        adapter.connect()

        task = _make()
        coord.add_task(task)
        task.start_path(path, _odom_to_pose(adapter.read_odometry()))

        ticks: list[TrajectoryTick] = []
        period = 1.0 / GO2_TICK_RATE_HZ
        t0 = time.perf_counter()
        next_sample = t0
        arrived = False
        while True:
            now = time.perf_counter()
            t_rel = now - t0
            if t_rel > timeout_s:
                break
            pose = _odom_to_pose(adapter.read_odometry())
            task.update_odom(pose)
            ticks.append(
                TrajectoryTick(
                    t=t_rel,
                    pose=pose,
                    cmd_twist=_vels_to_twist(adapter._cmd),
                    actual_twist=_vels_to_twist(adapter.read_velocities()),
                )
            )
            s = task.get_state()
            if s in _ARRIVED_STATES:
                arrived = True
                break
            if s in _FAILED_STATES:
                break
            next_sample += period
            sleep_for = next_sample - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
        return ExecutedTrajectory(ticks=ticks, arrived=arrived), path
    finally:
        coord.stop()


# --- hw harness (real Go2 over LCM; closed-loop baseline) ---------------
#
# Ported from the R&D `_run_path_follower_hw`. Talks LCM to a separately
# running `dimos run unitree-go2-webrtc-keyboard-teleop` (publishes
# /go2/odom, consumes /cmd_vel; its teleop is publish-only-when-active,
# so between gated paths you reposition with the keyboard, release keys
# — teleop goes silent — then the tool owns /cmd_vel for the run). No
# new module — inlined here, the small estimator/anchor duplicated with
# go2_characterization by choice (no shared-module addition).

VX_MAX = 1.0
WZ_MAX = 1.5
_ODOM_WARMUP_S = 10.0  # WebRTC connect + first odom (override: --odom-warmup)
_ODOM_STALE_S = 1.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _twist_clamped(vx: float, wz: float) -> Twist:
    return Twist(
        linear=Vector3(_clamp(vx, -VX_MAX, VX_MAX), 0.0, 0.0),
        angular=Vector3(0.0, 0.0, _clamp(wz, -WZ_MAX, WZ_MAX)),
    )


class _PoseVelocityEstimator:
    """Consecutive ``PoseStamped`` -> EMA body-frame (vx,vy,wz)."""

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
        c, s = math.cos(y1), math.sin(y1)
        bx = (dx / dt) * c + (dy / dt) * s
        by = -(dx / dt) * s + (dy / dt) * c
        self._vx = self._a * bx + (1 - self._a) * self._vx
        self._vy = self._a * by + (1 - self._a) * self._vy
        self._wz = self._a * (dyaw / dt) + (1 - self._a) * self._wz
        self._pp, self._pt = pose, t
        return self._vx, self._vy, self._wz


def _shift_path_to_start_at_pose(path: NavPath, start_pose: PoseStamped) -> NavPath:
    """Rigid-transform a robot-centric reference path into the odom frame
    anchored at the robot's current pose (so it need not be positioned
    precisely — only roughly aimed)."""
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


def _run_baseline_hw(
    link: dict,
    path: NavPath,
    speed: float,
    k_angular: float,
    ff_config: FeedforwardGainConfig,
    profile_config: VelocityProfileConfig,
    timeout_s: float,
    label: str,
) -> tuple[ExecutedTrajectory, NavPath]:
    """Closed-loop baseline on the real Go2: anchor the path to the
    robot's current pose, then track at 10 Hz off real odom. Safe:
    velocity clamp, stale-odom abort, timeout, zero-Twist on exit.
    Returns the trajectory and the anchored reference path (odom frame)
    — score/plot must use this, not the robot-centric input path."""
    cmd_pub, get_odom = link["pub"], link["get"]

    base = BaselinePathFollowerTask(
        name=f"baseline_{label}",
        config=BaselinePathFollowerTaskConfig(
            speed=speed, k_angular=k_angular, ff_config=ff_config
        ),
        global_config=global_config,
    )
    task = _VelocityProfileProxyTask(base, PathSpeedCap(profile_config))

    pose0, _ = get_odom()
    path_w = _shift_path_to_start_at_pose(path, pose0)
    task.start_path(path_w, pose0)

    ticks: list[TrajectoryTick] = []
    est = _PoseVelocityEstimator()
    period = 1.0 / GO2_TICK_RATE_HZ
    t0 = time.perf_counter()
    arrived = False
    try:
        while True:
            now = time.perf_counter()
            t_rel = now - t0
            if t_rel > timeout_s:
                print(f"  [{label}] timeout {timeout_s:.0f}s")
                break
            pose, age = get_odom()
            if pose is None or age > _ODOM_STALE_S:
                print(f"  [{label}] ABORT stale odom ({age:.2f}s)")
                break
            task.update_odom(pose)
            ev = est.update(pose, now)
            state = CoordinatorState(
                joints=JointStateSnapshot(
                    joint_velocities={"base/vx": ev[0], "base/vy": ev[1], "base/wz": ev[2]},
                    timestamp=now,
                ),
                t_now=now,
                dt=period,
            )
            out = task.compute(state)
            vx, wz = (
                (out.velocities[0], out.velocities[2])
                if (out is not None and out.velocities is not None)
                else (0.0, 0.0)
            )
            tw = _twist_clamped(vx, wz)
            cmd_pub.broadcast(None, tw)
            ticks.append(
                TrajectoryTick(
                    t=t_rel,
                    pose=pose,
                    cmd_twist=tw,
                    actual_twist=Twist(
                        linear=Vector3(ev[0], ev[1], 0.0),
                        angular=Vector3(0.0, 0.0, ev[2]),
                    ),
                )
            )
            st = task.get_state()
            if st in _ARRIVED_STATES:
                arrived = True
                print(f"  [{label}] arrived in {t_rel:.1f}s")
                break
            if st in _FAILED_STATES:
                print(f"  [{label}] task aborted")
                break
            time.sleep(max(0.0, t0 + len(ticks) * period - time.perf_counter()))
    finally:
        for _ in range(3):
            cmd_pub.broadcast(None, _twist_clamped(0.0, 0.0))
            time.sleep(0.05)
    return ExecutedTrajectory(ticks=ticks, arrived=arrived), path_w


# --- benchmark ----------------------------------------------------------


def _path_set() -> dict:
    """Real-space-constrained fixed path set (locked — do not widen)."""
    return {
        "straight_line": straight_line(),
        "single_corner": single_corner(leg_length=2.0, angle_deg=90.0),
        "square": square(side=2.0),
        "circle": circle(radius=1.0),
    }


REPORTS_DIR = Path(__file__).parent / "reports"


def _open_hw_link(warmup_s: float) -> dict:
    """LCM to a running `dimos run unitree-go2-webrtc-keyboard-teleop`."""
    from dimos.core.transport import LCMTransport

    cmd_pub = LCMTransport("/cmd_vel", Twist)
    odom_sub = LCMTransport("/go2/odom", PoseStamped)
    lock = threading.Lock()
    box: dict = {"pose": None, "t": 0.0}

    def _on(msg) -> None:
        with lock:
            box["pose"] = msg
            box["t"] = time.perf_counter()

    odom_sub.subscribe(_on)

    def get_odom():
        with lock:
            return box["pose"], time.perf_counter() - box["t"]

    print(f"[hw] waiting up to {warmup_s:.0f}s for /go2/odom ...")
    deadline = time.perf_counter() + warmup_s
    while time.perf_counter() < deadline and get_odom()[0] is None:
        time.sleep(0.05)
    if get_odom()[0] is None:
        raise RuntimeError("No /go2/odom — is `dimos run unitree-go2-webrtc-keyboard-teleop` up?")
    return {"pub": cmd_pub, "get": get_odom}


def _run_ladder(
    cfg: Go2TuningConfig, speeds: list[float], timeout_s: float, mode: str, warmup_s: float
) -> tuple[list[OperatingPoint], list[dict]]:
    ff = cfg.feedforward.to_runtime()
    k_angular = float(cfg.recommended_controller.params.get("k_angular", 0.5))
    link = _open_hw_link(warmup_s) if mode == "hw" else None
    points: list[OperatingPoint] = []
    runs: list[dict] = []  # for the XY trajectory overlay
    try:
        for name, path in _path_set().items():
            for speed in speeds:
                profile = cfg.velocity_profile.to_runtime(max_linear_speed=speed)
                if mode == "hw":
                    for _ in range(3):
                        link["pub"].broadcast(None, _twist_clamped(0.0, 0.0))
                        time.sleep(0.05)
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
                    traj, ref = _run_baseline_hw(
                        link,
                        path,
                        speed,
                        k_angular,
                        ff,
                        profile,
                        timeout_s,
                        f"{name}@{speed:.2f}",
                    )
                else:
                    traj, ref = _run_baseline_sim(path, speed, k_angular, ff, profile, timeout_s)
                # Score/plot against the executed-frame reference: in hw
                # that's the pose-anchored path, not the robot-centric input.
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
        if link is not None:
            for _ in range(3):
                link["pub"].broadcast(None, _twist_clamped(0.0, 0.0))
                time.sleep(0.05)
    return points, runs


def _plot(points: list[OperatingPoint], out: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for name in sorted({p.path for p in points}):
        xs = [p.speed for p in points if p.path == name]
        ys = [p.cte_max * 100 for p in points if p.path == name]
        ax.plot(xs, ys, marker="o", label=name)
    ax.set_xlabel("commanded speed (m/s)")
    ax.set_ylabel("cte_max (cm)")
    ax.set_title("Go2 baseline tracking: cross-track error vs speed")
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
    regardless of where the robot physically started (hw anchors each run
    to its current odom pose; sim already starts at the path origin)."""
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


def _plot_xy(runs: list[dict], out: Path) -> None:
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
    fig.suptitle("Go2 baseline: executed trajectory vs reference path")
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description="Go2 operating-point benchmark")
    ap.add_argument("--config", required=True, help="config artifact from go2_characterization")
    ap.add_argument("--mode", choices=["hw", "sim"], default="hw")
    ap.add_argument("--speeds", default="0.3,0.5,0.7,0.9,1.0")
    ap.add_argument("--tolerances", default="5,10,15", help="cm")
    ap.add_argument("--timeout", type=float, default=60.0, help="per (path,speed) run timeout (s)")
    ap.add_argument(
        "--odom-warmup",
        type=float,
        default=_ODOM_WARMUP_S,
        help="how long to wait for first /go2/odom (s)",
    )
    args = ap.parse_args()

    config_path = Path(args.config).expanduser()
    cfg = Go2TuningConfig.from_json(config_path)  # asserts schema_version
    speeds = [float(s) for s in args.speeds.split(",")]
    tolerances = [float(t) for t in args.tolerances.split(",")]

    if args.mode == "hw" and not cfg.valid_for_tuning:
        sys.exit(
            "Refusing --mode hw with a non-robot-valid config "
            f"({config_path.name}, sim_or_hw={cfg.provenance.sim_or_hw!r}). Its "
            "feedforward/profile were derived from the sim plant — running "
            "them on the real robot is meaningless. Re-run "
            "`go2_characterization --mode hw` first, or use --mode sim for a "
            "pre-check."
        )
    if args.mode == "sim":
        print(
            "[pre-check] --mode sim: validates wiring against the FOPDT sim "
            "plant only; the operating-point map is NOT a real-robot result."
        )

    print(
        f"{args.mode} speed ladder {speeds} over {len(_path_set())} paths "
        f"(baseline k_angular={cfg.recommended_controller.params.get('k_angular')}):"
    )
    try:
        points, runs = _run_ladder(cfg, speeds, args.timeout, args.mode, args.odom_warmup)
    except KeyboardInterrupt:
        raise SystemExit(
            "\n[hw] aborted by operator — robot stopped, artifact not modified."
        ) from None
    inversion = invert_tolerance(points, tolerances)
    cfg.operating_point_map = OperatingPointMap(
        speeds=speeds, points=points, tolerance_inversion=inversion
    )

    cfg.to_json(config_path)  # augment input artifact in place (section 5)
    sha = cfg.provenance.git_sha
    bench_path = REPORTS_DIR / f"go2_benchmark_{sha}.json"
    bench_path.parent.mkdir(parents=True, exist_ok=True)
    bench_path.write_text(json.dumps(asdict(cfg.operating_point_map), indent=2))
    plot_path = REPORTS_DIR / f"go2_benchmark_cte_vs_speed_{sha}.png"
    _plot(points, plot_path)
    xy_path = REPORTS_DIR / f"go2_benchmark_xy_{sha}.png"
    _plot_xy(runs, xy_path)

    print(f"\nAugmented artifact : {config_path.resolve()}")
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
