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

"""Path-following benchmark — a controller-agnostic publisher/recorder.

This module knows NOTHING about the controller's internals. It talks to whatever
follows paths purely over the transport:

    OUT  path   (nav_msgs/Path)         -- the reference path for one run
    OUT  speed  (std_msgs/Float32, m/s) -- the target/cruise speed for one run
    IN   odom   (geometry_msgs/PoseStamped) -- the robot's executed pose
    IN   cmd_vel(geometry_msgs/Twist)       -- the command sent to the robot
    IN   gate   (std_msgs/Int8)             -- operator advance/skip/quit (hw)

For each (path, speed) in a fixed battery it: waits for an operator gate (so the
robot can be teleoped into position), anchors the reference path to the robot's
current odom, publishes ``speed`` then ``path``, records incoming odom + cmd_vel,
detects completion **from odom alone** (within a goal tolerance of the path's
last pose AND near-zero velocity for a short dwell, or a per-run timeout), and
writes the run (reference path + executed trace + metadata) to disk as one flat
JSON. It does NOT score inline — scoring is a separate offline step (see
:mod:`dimos.control.benchmarking.score`).

Launch as a SEPARATE process from the controller::

    dimos run unitree-go2-rpp-benchmark         # the benchmark
    dimos run unitree-go2-rpp-controller        # the controller (other terminal)
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import queue
import threading
import time
from typing import Any, Literal

import numpy as np
from reactivex.disposable import Disposable

from dimos.control.benchmarking.gate import GATE_QUIT, GATE_SKIP
from dimos.control.benchmarking.paths import (
    circle,
    rounded_square,
    single_corner,
    smooth_corner,
    square,
    straight_line,
)
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.utils.logging_config import setup_logger
from dimos.utils.path_utils import get_project_root

logger = setup_logger()

DEFAULT_OUT_DIR = get_project_root() / "data" / "benchmark"

# Recording schema version — bump if the on-disk JSON shape changes.
RECORDING_SCHEMA = 1


def path_set() -> dict[str, NavPath]:
    """Real-space-constrained fixed path battery.

    Sharp corners (single_corner, square) keep the infinite-curvature 90°
    geometry; their curved counterparts (smooth_corner, rounded_square) fillet
    the vertices so a tracker can hold them at speed.
    """
    return {
        "straight_line": straight_line(),
        "single_corner": single_corner(leg_length=2.0, angle_deg=90.0),
        "smooth_corner": smooth_corner(leg_length=2.0, angle_deg=90.0, arc_radius=0.5),
        "square": square(side=2.0),
        "rounded_square": rounded_square(side=2.0, arc_radius=0.5),
        "circle": circle(radius=1.0),
    }


def shift_path_to_start_at_pose(path: NavPath, start_pose: PoseStamped) -> NavPath:
    """Rigid-transform a robot-centric reference path into the odom frame anchored
    at the robot's current pose, so the operator only has to roughly aim the
    robot. Scoring is then in the executed frame regardless of where the plant
    starts."""
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


# ---------------------------------------------------------------------------
# Recording (flat per-run JSON) + round-trip
# ---------------------------------------------------------------------------


@dataclass
class RunRecording:
    """One (path, speed) run: the anchored reference + executed trace + metadata.

    ``reference`` is a list of ``[x, y, yaw]``; ``ticks`` is a list of
    ``[t, x, y, yaw, cmd_vx, cmd_vy, cmd_wz]``. The offline scorer rebuilds a
    ``Path`` + ``ExecutedTrajectory`` from these and runs ``score_run``.
    """

    robot: str
    path: str
    speed: float
    arrived: bool
    reason: str
    goal_tolerance: float
    velocity_threshold: float
    timeout: float
    reference: list[list[float]] = field(default_factory=list)
    ticks: list[list[float]] = field(default_factory=list)
    schema: int = RECORDING_SCHEMA

    @classmethod
    def from_path(
        cls,
        *,
        robot: str,
        path_name: str,
        speed: float,
        reference: NavPath,
        goal_tolerance: float,
        velocity_threshold: float,
        timeout: float,
    ) -> RunRecording:
        ref = [[p.position.x, p.position.y, p.orientation.euler[2]] for p in reference.poses]
        return cls(
            robot=robot,
            path=path_name,
            speed=float(speed),
            arrived=False,
            reason="",
            goal_tolerance=goal_tolerance,
            velocity_threshold=velocity_threshold,
            timeout=timeout,
            reference=ref,
        )

    def to_json(self, out_path: str | Path) -> None:
        Path(out_path).write_text(json.dumps(self.__dict__, indent=2))

    @classmethod
    def from_json(cls, in_path: str | Path) -> RunRecording:
        return cls(**json.loads(Path(in_path).read_text()))

    def reference_path(self) -> NavPath:
        """Rebuild the reference ``Path`` (for the offline scorer)."""
        return NavPath(
            poses=[
                PoseStamped(
                    position=Vector3(x, y, 0.0),
                    orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
                )
                for x, y, yaw in self.reference
            ]
        )


# ---------------------------------------------------------------------------
# Odom recorder — accumulates ticks; recovers body-frame velocity by diff
# ---------------------------------------------------------------------------


class OdomRecorder:
    """Subscribe to ``odom`` (and ``cmd_vel``); turn each odom into a tick.

    Body-frame velocity is recovered by pose differentiation (legged-base odom
    reports pose, not body velocity), EMA-smoothed (``alpha``). The live body
    speed is what the completion monitor reads; the recorded ``cmd_*`` columns
    come from the most recent ``cmd_vel``.
    """

    def __init__(self, alpha: float = 0.5) -> None:
        self._alpha = alpha
        self._lock = threading.Lock()
        self._ticks: list[list[float]] = []
        self._t0: float | None = None
        self._prev_pose: PoseStamped | None = None
        self._prev_t: float | None = None
        self._vx = self._vy = self._wz = 0.0
        self._cmd_vx = self._cmd_vy = self._cmd_wz = 0.0
        self._latest_pose: PoseStamped | None = None

    def on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._cmd_vx = float(msg.linear.x)
            self._cmd_vy = float(msg.linear.y)
            self._cmd_wz = float(msg.angular.z)

    def on_odom(self, pose: PoseStamped, now: float | None = None) -> None:
        now = time.perf_counter() if now is None else now
        with self._lock:
            self._latest_pose = pose
            if self._t0 is None:
                self._t0 = now
            t_rel = now - self._t0
            if self._prev_pose is None or self._prev_t is None:
                self._prev_pose, self._prev_t = pose, now
            else:
                dt = now - self._prev_t
                if dt > 0:
                    dx = pose.position.x - self._prev_pose.position.x
                    dy = pose.position.y - self._prev_pose.position.y
                    y1 = pose.orientation.euler[2]
                    dyaw = (y1 - self._prev_pose.orientation.euler[2] + math.pi) % (
                        2 * math.pi
                    ) - math.pi
                    c, s = math.cos(y1), math.sin(y1)
                    bx = (dx / dt) * c + (dy / dt) * s
                    by = -(dx / dt) * s + (dy / dt) * c
                    a = self._alpha
                    self._vx = a * bx + (1 - a) * self._vx
                    self._vy = a * by + (1 - a) * self._vy
                    self._wz = a * (dyaw / dt) + (1 - a) * self._wz
                    self._prev_pose, self._prev_t = pose, now
            self._ticks.append(
                [
                    t_rel,
                    pose.position.x,
                    pose.position.y,
                    pose.orientation.euler[2],
                    self._cmd_vx,
                    self._cmd_vy,
                    self._cmd_wz,
                ]
            )

    def body_speed(self) -> tuple[float, float]:
        """(linear body speed, |angular|) from the latest differentiation."""
        with self._lock:
            return math.hypot(self._vx, self._vy), abs(self._wz)

    def latest_pose(self) -> PoseStamped | None:
        with self._lock:
            return self._latest_pose

    def snapshot(self) -> list[list[float]]:
        with self._lock:
            return [list(t) for t in self._ticks]

    def reset(self) -> None:
        with self._lock:
            self._ticks.clear()
            self._t0 = None
            self._prev_pose = self._prev_t = None
            self._vx = self._vy = self._wz = 0.0

    def wait_first_pose(self, timeout_s: float, poll_s: float = 0.02) -> PoseStamped:
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            pose = self.latest_pose()
            if pose is not None:
                return pose
            time.sleep(poll_s)
        raise RuntimeError(f"no odom within {timeout_s:.1f}s")


# ---------------------------------------------------------------------------
# Completion monitor — odom-only arrival detection
# ---------------------------------------------------------------------------


class CompletionMonitor:
    """Declare a run complete from odom alone.

    Tracks forward progress along the reference path (windowed nearest index, so
    closed paths don't trip arrival at the start) and declares completion once
    the robot has covered ``progress_frac`` of the path, is within
    ``goal_tolerance`` of the last pose, and has near-zero linear+angular speed
    sustained for ``dwell_s``.
    """

    def __init__(
        self,
        reference: NavPath,
        *,
        goal_tolerance: float,
        velocity_threshold: float,
        angular_threshold: float,
        dwell_s: float,
        progress_frac: float = 0.7,
        window: int = 20,
    ) -> None:
        self._xy = np.array([[p.position.x, p.position.y] for p in reference.poses], dtype=float)
        self._goal = self._xy[-1]
        self._n = len(self._xy)
        self._goal_tol = goal_tolerance
        self._v_thresh = velocity_threshold
        self._w_thresh = angular_threshold
        self._dwell_s = dwell_s
        self._window = window
        self._progress_idx = 0
        self._progress_threshold = max(1, int(progress_frac * (self._n - 1)))
        self._settled_since: float | None = None

    def update(self, x: float, y: float, lin_speed: float, ang_speed: float, t: float) -> bool:
        pos = np.array([x, y])
        lo = self._progress_idx
        hi = min(self._n, lo + self._window + 1)
        idx = lo + int(np.argmin(np.sum((self._xy[lo:hi] - pos) ** 2, axis=1)))
        self._progress_idx = max(self._progress_idx, idx)

        near_goal = float(np.linalg.norm(self._goal - pos)) < self._goal_tol
        stopped = lin_speed < self._v_thresh and ang_speed < self._w_thresh
        covered = self._progress_idx >= self._progress_threshold
        if covered and near_goal and stopped:
            if self._settled_since is None:
                self._settled_since = t
            return (t - self._settled_since) >= self._dwell_s
        self._settled_since = None
        return False


# ---------------------------------------------------------------------------
# Benchmarker module
# ---------------------------------------------------------------------------


class BenchmarkerConfig(ModuleConfig):
    """Config for :class:`Benchmarker`.

    ``gate_source`` selects how each (path, speed) cell is paced. ``"stream"``
    (default, hardware) waits for the operator's gate (ENTER/skip/quit from the
    controller's KeyboardTeleop) so they can reposition the robot first;
    ``"auto"`` advances immediately (headless sim / validation).
    """

    robot: str = "go2"
    speeds: str = "0.3,0.5,0.7,0.9,1.0"
    tolerances: str = "5,10,15"  # cm — carried through to the offline scorer
    goal_tolerance: float = 0.25  # m — arrival radius around the last pose
    velocity_threshold: float = 0.05  # m/s — "stopped" linear-speed gate
    angular_threshold: float = 0.10  # rad/s — "stopped" angular-speed gate
    settle_dwell_s: float = 0.5  # s within tolerance+stopped before declaring done
    timeout: float = 60.0  # s per run before giving up (recorded not-arrived)
    odom_warmup_s: float = 10.0  # s to wait for the first odom each run
    speed_settle_s: float = 0.3  # s between publishing speed and the path
    gate_source: Literal["stream", "auto"] = "stream"
    out_dir: str | None = None


class Benchmarker(Module):
    """Publishes the path battery + speeds, records odom, detects odom-based
    completion, and writes per-run recordings. Pure pub/sub — never calls into
    the controller."""

    config: BenchmarkerConfig

    path: Out[NavPath]
    speed: Out[Float32]
    odom: In[PoseStamped]
    cmd_vel: In[Twist]
    gate: In[Int8]

    _gate_queue: queue.Queue[int]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._gate_queue = queue.Queue()
        self._recorder = OdomRecorder()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._recorder.on_odom)))
        if self.cmd_vel.transport is not None:
            self.register_disposable(Disposable(self.cmd_vel.subscribe(self._recorder.on_cmd_vel)))
        if self.config.gate_source == "stream" and self.gate.transport is not None:
            self.register_disposable(Disposable(self.gate.subscribe(self._on_gate_event)))
        # Run on a background thread so start() returns immediately (the session
        # is operator-paced and easily outlives the start RPC's timeout).
        threading.Thread(target=self._run, name="benchmarker", daemon=True).start()

    def _on_gate_event(self, msg: Int8) -> None:
        self._gate_queue.put(int(msg.data))

    def _wait_gate(self) -> int:
        return self._gate_queue.get()

    # -- the session loop ----------------------------------------------------

    def _run(self) -> None:
        cfg = self.config
        speeds = [float(s) for s in cfg.speeds.split(",") if s.strip()]
        out_root = Path(cfg.out_dir).expanduser() if cfg.out_dir else DEFAULT_OUT_DIR / cfg.robot
        out_root.mkdir(parents=True, exist_ok=True)

        logger.info(
            f"Benchmarker: {cfg.robot} speeds={speeds} over {len(path_set())} paths "
            f"(gate_source={cfg.gate_source}, out={out_root})"
        )

        idx = 0
        for path_name, path in path_set().items():
            for speed in speeds:
                if cfg.gate_source == "stream":
                    logger.info(
                        f"[{path_name} v={speed:.2f}] reposition+aim robot, then ENTER "
                        f"(K=skip, Backspace=quit)"
                    )
                    ev = self._wait_gate()
                    if ev == GATE_QUIT:
                        logger.info("operator quit — ending session")
                        return
                    if ev == GATE_SKIP:
                        logger.info("  skipped")
                        continue
                    # GATE_ADVANCE falls through to run.
                try:
                    rec = self._run_one(cfg.robot, path_name, path, speed)
                except RuntimeError as e:
                    logger.warning(f"[{path_name} v={speed:.2f}] {e}; skipping")
                    continue
                out_path = out_root / f"{cfg.robot}_{path_name}_v{speed:.2f}_{idx:03d}.json"
                rec.to_json(out_path)
                idx += 1
                logger.info(
                    f"  [{path_name} v={speed:.2f}] {rec.reason} "
                    f"(arrived={rec.arrived}, ticks={len(rec.ticks)}) -> {out_path.name}"
                )
        logger.info(f"Benchmarker: done — {idx} run(s) recorded under {out_root}")

    def _run_one(self, robot: str, path_name: str, path: NavPath, speed: float) -> RunRecording:
        """Publish one (path, speed), record odom until odom-based completion or
        timeout, and return the recording."""
        cfg = self.config
        pose0 = self._recorder.wait_first_pose(cfg.odom_warmup_s)
        path_w = shift_path_to_start_at_pose(path, pose0)

        rec = RunRecording.from_path(
            robot=robot,
            path_name=path_name,
            speed=speed,
            reference=path_w,
            goal_tolerance=cfg.goal_tolerance,
            velocity_threshold=cfg.velocity_threshold,
            timeout=cfg.timeout,
        )

        # Fresh trace; publish speed first so the follower arms at this speed,
        # then the anchored path (which arms the follower).
        self._recorder.reset()
        self.speed.publish(Float32(data=float(speed)))
        time.sleep(cfg.speed_settle_s)
        self.path.publish(path_w)

        monitor = CompletionMonitor(
            path_w,
            goal_tolerance=cfg.goal_tolerance,
            velocity_threshold=cfg.velocity_threshold,
            angular_threshold=cfg.angular_threshold,
            dwell_s=cfg.settle_dwell_s,
        )
        arrived, reason = self._await_completion(monitor, cfg.timeout)
        rec.arrived = arrived
        rec.reason = reason
        rec.ticks = self._recorder.snapshot()
        return rec

    def _await_completion(self, monitor: CompletionMonitor, timeout_s: float) -> tuple[bool, str]:
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            pose = self._recorder.latest_pose()
            if pose is not None:
                lin, ang = self._recorder.body_speed()
                if monitor.update(pose.position.x, pose.position.y, lin, ang, time.perf_counter()):
                    return True, "goal+stop"
            time.sleep(0.05)
        return False, "timeout"


def main() -> None:
    """Run the benchmark standalone (blueprint-free). Mostly for headless/auto
    use; the operator-paced flow is launched via the benchmark blueprint."""
    import argparse

    ap = argparse.ArgumentParser(description="Path-following benchmark (pub/sub)")
    ap.add_argument("--robot", default="go2")
    ap.add_argument("--speeds", default="0.3,0.5,0.7,0.9,1.0")
    ap.add_argument("--gate-source", choices=["stream", "auto"], default="auto")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    Benchmarker(
        robot=args.robot,
        speeds=args.speeds,
        gate_source=args.gate_source,
        timeout=args.timeout,
        out_dir=args.out,
    ).start()


if __name__ == "__main__":
    main()
