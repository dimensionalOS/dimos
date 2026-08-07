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

"""Closed-loop episodes: referee world -> planner -> controller -> matched sim.

One episode = one scenario. The planner sees the referee's analytic cloud and
the controller sees a 29 Hz zero-order-hold pose (the pointlio cadence) — the
twist it emits then rides the exact command chain the sim-to-real fit
validated: per-axis hardware slew, fitted transport delay, fitted actuator lag.
"""

from __future__ import annotations

import collections
from dataclasses import dataclass, field, replace
import math
import time as _time

import mujoco
import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.control.controller import TrajectoryController
from dimos.navigation.motion.control.profile import encode_precision
from dimos.navigation.motion.control.referee import world
from dimos.navigation.motion.planner.referee.geometry import AvoidanceConfig
from dimos.navigation.motion.planner.referee.planners.base import load as load_planner
from dimos.navigation.motion.planner.referee.scenarios import Scenario
from dimos.navigation.motion.planner.referee.types import Path as RefereePath
from dimos.navigation.motion.simulation.evaluate import (
    FITTED_ACTUATOR_TAU,
    FITTED_COMMAND_DELAY,
    FITTED_PHYSICS,
)
from dimos.navigation.motion.simulation.policy import FreePolicy
from dimos.navigation.motion.simulation.walk import (
    COMMAND_SLEW,
    CONTROL_DT,
    NOMINAL_GAIT_HEIGHT,
    TORQUE_LIMITS,
    actuator_step,
    projected_gravity,
)

# A published path shorter than this is a refusal (referee score.py STALL_ARC).
STALL_ARC = 0.3


@dataclass
class EpisodeConfig:
    """Everything an episode holds fixed. Defaults are the matched sim."""

    physics: dict[str, float] = field(default_factory=lambda: dict(FITTED_PHYSICS))
    command_delay: float = FITTED_COMMAND_DELAY
    actuator_tau: float = FITTED_ACTUATOR_TAU
    odom_hz: float = 29.0  # pointlio cadence (mid360_athens_stairs: 28.9)
    odom_latency: float = 0.0  # pipeline latency knob; staleness is odom_hz's job
    replan_hz: float = 5.0  # reality replans; 0 = plan-once (lenient diagnostic mode)
    planner: str = "target"  # referee registry name or "module:factory"
    settle: float = 0.5  # policy warmup before the first command (walk.py)
    timeout: float = 40.0
    goal_tol: float = 0.20  # planar distance that counts as arrival (m)
    fall_tilt: float = 1.05  # rad (~60 deg) body tilt = fell
    fall_height: float = 0.12  # trunk z under this = collapsed
    annotate_clearance: bool = True  # hand the controller the path room hint


@dataclass
class DomainRandomization:
    """Per-episode draws around the fitted mechanisms — sim-to-sim, never
    per-tick (per-tick jitter would break the mechanism identities the fit
    established; per-episode draws express the flat-basin uncertainty).

    Default ranges bracket the two viewer-confirmed fits: delay 9 ms (joint)
    to 32 ms (himloco-only), tau 15 to 23 ms — and odom latency spans what
    the athens db cannot observe. ``physics`` takes explicit per-key ranges.
    """

    command_delay: tuple[float, float] = (0.0, 0.040)
    actuator_tau: tuple[float, float] = (0.005, 0.030)
    odom_latency: tuple[float, float] = (0.0, 0.040)
    physics: dict[str, tuple[float, float]] = field(default_factory=dict)

    def draw(self, cfg: EpisodeConfig, rng: np.random.Generator) -> EpisodeConfig:
        """A fresh EpisodeConfig with sampled mechanisms; cfg is untouched."""
        physics = dict(cfg.physics)
        for key, (lo, hi) in self.physics.items():
            physics[key] = float(rng.uniform(lo, hi))
        return replace(
            cfg,
            physics=physics,
            command_delay=float(rng.uniform(*self.command_delay)),
            actuator_tau=float(rng.uniform(*self.actuator_tau)),
            odom_latency=float(rng.uniform(*self.odom_latency)),
        )


@dataclass
class EpisodeResult:
    scenario: Scenario
    outcome: str  # "goal" | "collision" | "fall" | "timeout" | "refused"
    t: np.ndarray  # control ticks (s)
    pos: np.ndarray  # (n, 3) trunk position
    yaw: np.ndarray  # (n,) trunk yaw
    tilt: np.ndarray  # (n,) trunk tilt from vertical (rad)
    twist_cmd: np.ndarray  # (n, 3) controller output (vx, vy, vyaw)
    used_cmd: np.ndarray  # (n, 3) after delay + hardware slew
    contact: np.ndarray  # (n,) bool, wall touch at this tick
    plan: Path  # the (last) planned path, world frame
    plans: list[RefereePath]  # every plan the episode produced
    plan_t: list[float]  # when each plan became active (s)
    plan_min_clear: list[float]  # min planned clearance per plan (annotation space)
    plan_ms: list[float]  # planner CPU time per call
    time_to_goal: float | None
    cfg: EpisodeConfig  # what actually ran (post-DR values included)

    @property
    def reached(self) -> bool:
        return self.outcome == "goal"


def _tilt(quat_wxyz: np.ndarray) -> float:
    """Angle between the body z axis and world up."""
    w, x, y, z = quat_wxyz
    # R[2,2] of the rotation matrix
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return float(math.acos(max(-1.0, min(1.0, r22))))


def _yaw(quat_wxyz: np.ndarray) -> float:
    w, x, y, z = quat_wxyz
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _pose_stamped(t: float, xy_yaw: tuple[float, float, float], frame_id: str) -> PoseStamped:
    return PoseStamped(
        ts=t,
        frame_id=frame_id,
        position=Vector3(xy_yaw[0], xy_yaw[1], 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, xy_yaw[2])),
    )


def _path_arc(ref: RefereePath) -> float:
    xy = np.array([[p.position.x, p.position.y] for p in ref.poses])
    if len(xy) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


def outline_indices(
    xy: np.ndarray, yaws: np.ndarray, step: float = 0.35, yaw_step: float = 0.5
) -> list[int]:
    """Waypoints worth a footprint: every ``step`` of arc or ``yaw_step`` of
    rotation (fans advance in yaw at zero displacement), plus the endpoint."""
    if len(xy) == 0:
        return []
    picks = [0]
    arc = 0.0
    for i in range(1, len(xy)):
        arc += float(np.linalg.norm(xy[i] - xy[i - 1]))
        if arc >= step or abs(math.remainder(yaws[i] - yaws[picks[-1]], math.tau)) >= yaw_step:
            picks.append(i)
            arc = 0.0
    if picks[-1] != len(xy) - 1:
        picks.append(len(xy) - 1)
    return picks


PLAN_RGBA = (0.25, 0.55, 1.0, 0.22)


def _draw_plan(viewer: object, ref: RefereePath, sc: Scenario) -> None:
    """Expected body poses on the floor: one translucent footprint box per
    outline waypoint, oriented by the plan's own yaw. Redrawn per (re)plan."""
    scn = viewer.user_scn  # type: ignore[attr-defined]
    xy = np.array([[p.position.x, p.position.y] for p in ref.poses]).reshape(-1, 2)
    yaws = np.array([p.orientation.euler[2] for p in ref.poses])
    e = sc.emb
    scn.ngeom = 0
    for i in outline_indices(xy, yaws):
        if scn.ngeom >= scn.maxgeom:
            break
        yaw = float(yaws[i])
        c, s = math.cos(yaw), math.sin(yaw)
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(  # type: ignore[attr-defined]
            g,
            mujoco.mjtGeom.mjGEOM_BOX,
            np.array([e.length / 2.0, e.width / 2.0, 0.001]),
            # body centre sits center_off along body x from the pose point
            np.array([xy[i][0] + c * e.center_off, xy[i][1] + s * e.center_off, 0.003]),
            np.array([c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]),
            np.array(PLAN_RGBA, dtype=np.float32),
        )
        scn.ngeom += 1


def run_episode(
    sc: Scenario,
    controller: TrajectoryController,
    policy: FreePolicy,
    cfg: EpisodeConfig | None = None,
    view: bool = False,
    speed: float = 1.0,
    dr: DomainRandomization | None = None,
    rng: np.random.Generator | None = None,
) -> EpisodeResult:
    """Run one closed-loop episode; returns everything the judge needs."""
    cfg = cfg or EpisodeConfig()
    if dr is not None:
        cfg = dr.draw(cfg, rng if rng is not None else np.random.default_rng())
    model, data = world.load_world(sc, physics=cfg.physics)
    world.reset_to_start(model, data, sc, policy.default_pose)
    walls = world.wall_geom_ids(model)

    planner = load_planner(cfg.planner)(sc, AvoidanceConfig())
    planner.reset()
    controller.reset()
    cloud = world.planner_cloud(sc)

    sim_dt = model.opt.timestep
    decim = max(1, round(CONTROL_DT / sim_dt))
    # Pose and plan are both mujoco world coordinates here, so that is what the
    # label says. A controller has no frame of its own to name.
    frame_id = "world"

    hist: collections.deque[np.ndarray] = collections.deque(maxlen=policy.hist)
    last_action = np.zeros(policy.act_dim)
    target = policy.default_pose.copy()
    applied = np.zeros(12)

    def observe(cmd: np.ndarray) -> np.ndarray:
        q = data.qpos[7:19]
        dq = data.qvel[6:18]
        raw = np.concatenate(
            [cmd, data.qvel[3:6], projected_gravity(data.qpos[3:7]), q, dq, last_action]
        )
        extra = policy.obs_per_frame - raw.size
        if extra > 0:
            raw = np.concatenate([raw, [NOMINAL_GAIT_HEIGHT], np.zeros(extra - 1)])
        return policy.normalize(raw)

    def true_pose() -> tuple[float, float, float]:
        return (float(data.qpos[0]), float(data.qpos[1]), _yaw(data.qpos[3:7]))

    vel_cmd = np.zeros(3)  # what the policy sees, after slew
    for _ in range(policy.hist):
        hist.append(observe(vel_cmd))

    # The controller's world: a pose sampled at odom_hz, released odom_latency
    # later. Seeded with the spawn pose so the first ticks are never blind.
    odom_period = 1.0 / cfg.odom_hz
    odom_queue: collections.deque[tuple[float, tuple[float, float, float]]] = collections.deque()
    odom_queue.append((0.0, true_pose()))
    visible_pose = odom_queue[0][1]
    next_odom_t = odom_period

    # Twists cross the same transport the operator commands did.
    delay_queue: collections.deque[tuple[float, np.ndarray]] = collections.deque()
    delay_queue.append((0.0, np.zeros(3)))

    plans: list[RefereePath] = []
    plan_t: list[float] = []
    plan_min_clear: list[float] = []
    plan_ms: list[float] = []
    nav_path = Path(frame_id=frame_id, poses=[])
    clearance: np.ndarray | None = None
    replan_period = math.inf if cfg.replan_hz <= 0 else 1.0 / cfg.replan_hz
    next_plan_t = cfg.settle

    ts, pos, yaw, tilt, twist_cmd, used_cmd, contact = ([], [], [], [], [], [], [])  # type: ignore[var-annotated]
    outcome: str | None = None
    time_to_goal: float | None = None

    viewer_cm = None
    if view:
        from mujoco import viewer as mj_viewer

        viewer_cm = mj_viewer.launch_passive(model, data)
    viewer = viewer_cm.__enter__() if viewer_cm is not None else None

    try:
        wall_clock = _time.perf_counter()
        for step in range(int(cfg.timeout / sim_dt)):
            t = step * sim_dt
            if step % decim == 0:
                # -- odometry: sample, then release what latency allows
                while t >= next_odom_t:
                    odom_queue.append((next_odom_t, true_pose()))
                    next_odom_t += odom_period
                while odom_queue and odom_queue[0][0] <= t - cfg.odom_latency:
                    visible_pose = odom_queue.popleft()[1]

                # -- plan / replan from the controller's own pose
                if t >= next_plan_t:
                    t0 = _time.process_time()
                    ref = planner.plan(cloud, visible_pose, sc.goal)
                    plan_ms.append((_time.process_time() - t0) * 1e3)
                    plans.append(ref)
                    plan_t.append(t)
                    nav_path = world.to_nav_path(ref, ts=t, frame_id=frame_id)
                    # always computed for the judge's plan_tight diagnostic;
                    # the controller only sees it when annotation is on
                    clr = world.path_clearance(ref, cloud, sc.emb)
                    plan_min_clear.append(float(np.min(clr)) if len(clr) else math.inf)
                    # the wire dialect: precision rides the stamps, sim and
                    # robot paths speak identically (control/profile.py)
                    encode_precision(nav_path, clr, t0=t)
                    if cfg.annotate_clearance:
                        clearance = clr
                    next_plan_t = t + replan_period
                    if viewer is not None:
                        _draw_plan(viewer, ref, sc)
                    if not plans[:-1] and _path_arc(ref) < STALL_ARC:
                        outcome = "refused"
                        break

                # -- controller -> transport -> slew -> policy
                cmd_now = np.zeros(3)
                if t >= cfg.settle:
                    tw = controller.update(
                        _pose_stamped(t, visible_pose, frame_id), nav_path, t, clearance
                    )
                    cmd_now = np.array([tw.linear.x, tw.linear.y, tw.angular.z])
                delay_queue.append((t, cmd_now))
                seen = delay_queue[0][1]
                while delay_queue and delay_queue[0][0] <= t - cfg.command_delay:
                    seen = delay_queue.popleft()[1]
                vel_cmd += np.clip(seen - vel_cmd, -COMMAND_SLEW, COMMAND_SLEW)

                if t >= cfg.settle:
                    hist.append(observe(vel_cmd))
                    p_obs = np.concatenate(list(hist)[::-1])
                    last_action, target = policy.act(p_obs, vel_cmd)

                # -- record + terminate
                touched = world.wall_contact(model, data, walls)
                tp = true_pose()
                ts.append(t)
                pos.append(data.qpos[0:3].copy())
                yaw.append(tp[2])
                tilt.append(_tilt(data.qpos[3:7]))
                twist_cmd.append(cmd_now.copy())
                used_cmd.append(vel_cmd.copy())
                contact.append(touched)

                if touched:
                    outcome = "collision"
                    break
                if tilt[-1] > cfg.fall_tilt or data.qpos[2] < cfg.fall_height:
                    outcome = "fall"
                    break
                if math.hypot(tp[0] - sc.goal[0], tp[1] - sc.goal[1]) < cfg.goal_tol:
                    outcome = "goal"
                    time_to_goal = t
                    break

            tau = policy.kp * (target - data.qpos[7:19]) - policy.kd * data.qvel[6:18]
            tau = np.clip(tau, -TORQUE_LIMITS, TORQUE_LIMITS)
            applied = actuator_step(applied, tau, sim_dt, cfg.actuator_tau)
            data.ctrl[:] = applied
            mujoco.mj_step(model, data)

            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.sync()
                wall_clock += sim_dt / max(speed, 1e-6)
                lag = wall_clock - _time.perf_counter()
                if lag > 0:
                    _time.sleep(lag)
                else:
                    wall_clock = _time.perf_counter()
    finally:
        if viewer_cm is not None:
            viewer_cm.__exit__(None, None, None)

    return EpisodeResult(
        scenario=sc,
        outcome=outcome or "timeout",
        t=np.array(ts),
        pos=np.array(pos).reshape(-1, 3),
        yaw=np.array(yaw),
        tilt=np.array(tilt),
        twist_cmd=np.array(twist_cmd).reshape(-1, 3),
        used_cmd=np.array(used_cmd).reshape(-1, 3),
        contact=np.array(contact, dtype=bool),
        plan=nav_path,
        plans=plans,
        plan_t=plan_t,
        plan_min_clear=plan_min_clear,
        plan_ms=plan_ms,
        time_to_goal=time_to_goal,
        cfg=cfg,
    )
