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
everything robot-specific comes from the ``RobotProfile`` (``--robot``).

    uv run python -m dimos.utils.benchmarking.benchmark \\
        --robot go2 --config reports/go2_config_hw_<...>.json --mode hw

The sim harness (the baseline driven through a real ``ControlCoordinator``
+ the FOPDT sim adapter) is inlined below — small, baseline-only.
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
from dimos.utils.benchmarking.paths import circle, single_corner, square, straight_line
from dimos.utils.benchmarking.plant import ROBOT_PROFILES, RobotProfile
from dimos.utils.benchmarking.scoring import ExecutedTrajectory, TrajectoryTick, score_run
from dimos.utils.benchmarking.tuning import (
    OperatingPoint,
    OperatingPointMap,
    TuningConfig,
    invert_tolerance,
)
from dimos.utils.benchmarking.velocity_profile import PathSpeedCap, VelocityProfileConfig

if TYPE_CHECKING:
    from dimos.hardware.drive_trains.fopdt_sim_base.adapter import FopdtTwistBaseAdapter

_base_joints = make_twist_base_joints("base")
_ARRIVED_STATES = frozenset({"arrived", "completed"})
_FAILED_STATES = frozenset({"aborted"})

REPORTS_DIR = Path(__file__).parent / "reports"


def _resolve_profile(name: str) -> RobotProfile:
    try:
        return ROBOT_PROFILES[name]
    except KeyError:
        raise SystemExit(f"unknown --robot {name!r}; known: {sorted(ROBOT_PROFILES)}") from None


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _twist_clamped(vx: float, wz: float, vx_max: float, wz_max: float) -> Twist:
    return Twist(
        linear=Vector3(_clamp(vx, -vx_max, vx_max), 0.0, 0.0),
        angular=Vector3(0.0, 0.0, _clamp(wz, -wz_max, wz_max)),
    )


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


def _sim_base(profile: RobotProfile) -> HardwareComponent:
    return HardwareComponent(
        hardware_id="base",
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints("base"),
        adapter_type=profile.sim_adapter_key,
        adapter_kwargs={"params": profile.sim_plant},
    )


def _odom_to_pose(odom: list[float]) -> PoseStamped:
    return PoseStamped(
        position=Vector3(odom[0], odom[1], 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, odom[2])),
    )


def _vels_to_twist(v: list[float]) -> Twist:
    return Twist(linear=Vector3(v[0], v[1], 0.0), angular=Vector3(0.0, 0.0, v[2]))


def _run_baseline_sim(
    profile: RobotProfile,
    path: NavPath,
    speed: float,
    k_angular: float,
    ff_config: FeedforwardGainConfig | None,
    profile_config: VelocityProfileConfig | None,
    timeout_s: float,
) -> tuple[ExecutedTrajectory, NavPath]:
    """Stock baseline P-controller in sim against the profile's FOPDT
    sim adapter. ``ff_config``/``profile_config`` are OPTIONAL comparison
    arms (``None`` = bare controller — the physical-limit measurement).
    Returns the trajectory and the reference path in the executed frame
    (sim runs in the path's own frame, so it is ``path`` unchanged)."""
    coord = ControlCoordinator(
        tick_rate=profile.tick_rate_hz,
        hardware=[_sim_base(profile)],
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
        if profile_config is None:
            return base
        return _VelocityProfileProxyTask(base, PathSpeedCap(profile_config))

    coord.start()
    try:
        adapter: FopdtTwistBaseAdapter = coord._hardware["base"].adapter
        start = path.poses[0]
        adapter.set_initial_pose(start.position.x, start.position.y, start.orientation.euler[2])
        adapter.connect()

        task = _make()
        coord.add_task(task)
        task.start_path(path, _odom_to_pose(adapter.read_odometry()))

        ticks: list[TrajectoryTick] = []
        period = 1.0 / profile.tick_rate_hz
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


# --- hw harness (real robot over LCM; closed-loop baseline) -------------
#
# Ported from the R&D `_run_path_follower_hw`. Talks LCM to a separately
# running `dimos run <profile.blueprint>` (publishes the odom topic,
# consumes the cmd topic; if that blueprint includes a keyboard teleop it
# must be publish-only-when-active so it does not fight the run). No new
# module — the small estimator/anchor are duplicated with
# characterization by choice (no shared-module addition).


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
    profile: RobotProfile,
    link: dict,
    path: NavPath,
    speed: float,
    k_angular: float,
    ff_config: FeedforwardGainConfig | None,
    profile_config: VelocityProfileConfig | None,
    timeout_s: float,
    label: str,
) -> tuple[ExecutedTrajectory, NavPath]:
    """Closed-loop stock baseline on the real robot: anchor the path to
    the robot's current pose, then track at the profile tick rate off
    real odom. ``ff_config``/``profile_config`` are OPTIONAL arms
    (``None`` = bare = the physical-limit measurement). Safe: velocity
    clamp, stale-odom abort, timeout, zero-Twist on exit. Returns the
    trajectory and the anchored reference path (odom frame) — score/plot
    must use this, not the robot-centric input path."""
    cmd_pub, get_odom = link["pub"], link["get"]

    def stop_twist() -> Twist:
        return _twist_clamped(0.0, 0.0, profile.vx_max, profile.wz_max)

    base = BaselinePathFollowerTask(
        name=f"baseline_{label}",
        config=BaselinePathFollowerTaskConfig(
            speed=speed, k_angular=k_angular, ff_config=ff_config
        ),
        global_config=global_config,
    )
    task = (
        base
        if profile_config is None
        else _VelocityProfileProxyTask(base, PathSpeedCap(profile_config))
    )

    pose0, _ = get_odom()
    path_w = _shift_path_to_start_at_pose(path, pose0)
    task.start_path(path_w, pose0)

    ticks: list[TrajectoryTick] = []
    est = _PoseVelocityEstimator()
    period = 1.0 / profile.tick_rate_hz
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
            if pose is None or age > profile.odom_stale_s:
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
            tw = _twist_clamped(vx, wz, profile.vx_max, profile.wz_max)
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
            cmd_pub.broadcast(None, stop_twist())
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


def _open_hw_link(profile: RobotProfile, warmup_s: float) -> dict:
    """LCM to a running `dimos run <profile.blueprint>`."""
    from dimos.core.transport import LCMTransport

    cmd_pub = LCMTransport(profile.cmd_topic, Twist)
    odom_sub = LCMTransport(profile.odom_topic, PoseStamped)
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

    print(f"[hw] waiting up to {warmup_s:.0f}s for {profile.odom_topic} ...")
    deadline = time.perf_counter() + warmup_s
    while time.perf_counter() < deadline and get_odom()[0] is None:
        time.sleep(0.05)
    if get_odom()[0] is None:
        raise RuntimeError(f"No {profile.odom_topic} — is `dimos run {profile.blueprint}` up?")
    return {"pub": cmd_pub, "get": get_odom}


def _run_ladder(
    cfg: TuningConfig,
    profile: RobotProfile,
    speeds: list[float],
    timeout_s: float,
    mode: str,
    warmup_s: float,
    use_ff: bool,
    use_profile: bool,
) -> tuple[list[OperatingPoint], list[dict]]:
    # Bare stock baseline by default: this is the physical-limit
    # measurement. FF / velocity profile are opt-in comparison arms.
    ff = cfg.feedforward.to_runtime() if use_ff else None
    k_angular = float(cfg.recommended_controller.params.get("k_angular", 0.5))
    link = _open_hw_link(profile, warmup_s) if mode == "hw" else None
    points: list[OperatingPoint] = []
    runs: list[dict] = []  # for the XY trajectory overlay
    try:
        for name, path in _path_set().items():
            for speed in speeds:
                prof_cfg = (
                    cfg.velocity_profile.to_runtime(max_linear_speed=speed) if use_profile else None
                )
                if mode == "hw":
                    for _ in range(3):
                        link["pub"].broadcast(
                            None, _twist_clamped(0.0, 0.0, profile.vx_max, profile.wz_max)
                        )
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
                        profile,
                        link,
                        path,
                        speed,
                        k_angular,
                        ff,
                        prof_cfg,
                        timeout_s,
                        f"{name}@{speed:.2f}",
                    )
                else:
                    traj, ref = _run_baseline_sim(
                        profile, path, speed, k_angular, ff, prof_cfg, timeout_s
                    )
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
                link["pub"].broadcast(
                    None, _twist_clamped(0.0, 0.0, profile.vx_max, profile.wz_max)
                )
                time.sleep(0.05)
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


def main() -> None:
    ap = argparse.ArgumentParser(description="Twist-base operating-point benchmark")
    ap.add_argument("--robot", default="go2", help=f"one of {sorted(ROBOT_PROFILES)}")
    ap.add_argument("--config", required=True, help="config artifact from characterization")
    ap.add_argument("--mode", choices=["hw", "sim"], default="hw")
    ap.add_argument("--speeds", default="0.3,0.5,0.7,0.9,1.0")
    ap.add_argument("--tolerances", default="5,10,15", help="cm")
    ap.add_argument("--timeout", type=float, default=60.0, help="per (path,speed) run timeout (s)")
    ap.add_argument(
        "--odom-warmup",
        type=float,
        default=None,
        help="how long to wait for first odom (s); default from profile",
    )
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
    warmup_s = args.odom_warmup if args.odom_warmup is not None else profile.odom_warmup_s
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
            warmup_s,
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
