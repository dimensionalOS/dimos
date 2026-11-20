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

"""Reference re-implementation of rl_deploy's state machine (the Rust controller mirrors it).

States: passive -> stand_up -> rl_control (runner picked by gait code) -> lie_down -> passive; joint_damping on a
user request or a fault (stale state, tilt). One call to `tick` per 1 ms control step returns the 16 joint commands.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any

import numpy as np

from dimos.simulation.m20_policy_sim.config import (
    IS_WHEEL,
    JOINTS,
    policy_dir,
    runner_for_gait,
    tile,
)
from dimos.simulation.m20_policy_sim.runner import RobotState, Runner, Targets, projected_gravity

STATES = ("passive", "stand_up", "rl_control", "lie_down", "joint_damping")


@dataclass
class UserCommand:
    """What the operator asks for: a target state, a gait code, normalized velocity axes and runner parameters."""

    target: str = "passive"  # passive | stand | rl_control | lie_down | joint_damping
    gait: int = 0
    axes: np.ndarray = field(default_factory=lambda: np.zeros(3))
    params: dict[str, float] = field(default_factory=dict)
    runner: str = ""  # force a runner by name (for runners no gait code selects)


@dataclass
class JointCommand:
    """Per joint: tau = kp * (q - q_meas) + kd * (dq - dq_meas) + tau_ff, as the motor drivers run it."""

    kp: np.ndarray
    q: np.ndarray
    kd: np.ndarray
    dq: np.ndarray
    tau: np.ndarray

    @staticmethod
    def zero() -> JointCommand:
        z = np.zeros(JOINTS)
        return JointCommand(z.copy(), z.copy(), z.copy(), z.copy(), z.copy())


def cubic(p0: float, v0: float, p1: float, v1: float, t: float, T: float) -> tuple[float, float]:
    """Hermite cubic from (p0, v0) at 0 to (p1, v1) at T (rl_deploy's GetCubicSplinePos/Vel); holds p1 past T."""
    if t >= T:
        return p1, v1
    a2 = (3 * (p1 - p0) - (2 * v0 + v1) * T) / T**2
    a3 = (2 * (p0 - p1) + (v0 + v1) * T) / T**3
    return p0 + v0 * t + a2 * t**2 + a3 * t**3, v0 + 2 * a2 * t + 3 * a3 * t**2


def leg_pose_for_height(robot: dict[str, Any], h: float, hipx: float) -> np.ndarray:
    """The 16-joint pose with each wheel axle `h` below its hip (two-link IK), hind legs mirrored."""
    l1, l2 = robot["thigh_length"], robot["shank_length"]
    h = min(abs(h), 0.999 * (l1 + l2))
    hipy = -math.acos((l1 * l1 + h * h - l2 * l2) / (2 * h * l1))
    knee = math.pi - math.acos((l1 * l1 + l2 * l2 - h * h) / (2 * l1 * l2))
    pose = np.array(
        [
            hipx,
            hipy,
            knee,
            0.0,
            -hipx,
            hipy,
            knee,
            0.0,
            hipx,
            -hipy,
            -knee,
            0.0,
            -hipx,
            -hipy,
            -knee,
            0.0,
        ]
    )
    lo, hi = np.array(robot["joint_lower"]), np.array(robot["joint_upper"])
    return np.clip(pose, lo, hi)


class Controller:
    def __init__(self, cfg: dict[str, Any], policies: Path | None = None) -> None:
        self.cfg = cfg
        self.robot = cfg["robot"]
        self.dt = 1.0 / cfg["timing"]["control_hz"]
        self.decimation = cfg["timing"]["policy_decimation"]
        self.policies = policies or policy_dir()
        self.runners: dict[str, Runner] = {}
        self.state = "passive"
        self.state_time = 0.0
        self.tick_count = 0
        self.runner: Runner | None = None
        self.targets: Targets | None = None
        self.enter_q = np.zeros(JOINTS)
        self.enter_dq = np.zeros(JOINTS)
        self.last_stamp: float | None = None
        self.fault = ""
        self.transitions: list[tuple[float, str, str]] = []

    def runner_named(self, name: str) -> Runner:
        if name not in self.runners:
            r = Runner(name, self.cfg["runners"][name], self.robot, self.policies)
            r.policy_dt = self.dt * self.decimation
            self.runners[name] = r
        return self.runners[name]

    def select_runner(self, cmd: UserCommand) -> Runner:
        name = (
            cmd.runner
            or runner_for_gait(self.cfg, cmd.gait)
            or self.cfg["states"]["rl_control"]["fallback_runner"]
        )
        return self.runner_named(name)

    def enter(self, state: str, s: RobotState) -> None:
        self.transitions.append((self.tick_count * self.dt, self.state, state))
        self.state = state
        self.state_time = 0.0
        self.enter_q, self.enter_dq = s.q.copy(), s.dq.copy()
        if state == "rl_control":
            self.runner = None

    def check_faults(self, s: RobotState) -> str:
        safety = self.cfg["safety"]
        if self.last_stamp is not None and s.stamp - self.last_stamp > safety["state_timeout_s"]:
            return "stale state"
        g = projected_gravity(s.quat)
        if (
            self.state == "rl_control"
            and math.acos(max(-1.0, min(1.0, -g[2]))) > safety["max_tilt_rad"]
        ):
            return "tilt"
        return ""

    def next_state(self, cmd: UserCommand) -> str:
        st = self.cfg["states"]
        t = self.state_time
        if self.state != "joint_damping" and (cmd.target == "joint_damping" or self.fault):
            return "joint_damping"
        if self.state == "passive":
            return "stand_up" if cmd.target in ("stand", "rl_control") else "passive"
        if self.state == "stand_up":
            done = t >= st["stand_up"]["crouch_duration_s"] + st["stand_up"]["rise_duration_s"]
            if done and cmd.target == "rl_control":
                return "rl_control"
            if done and cmd.target in ("lie_down", "passive"):
                return "lie_down"
            return "stand_up"
        if self.state == "rl_control":
            return "lie_down" if cmd.target in ("lie_down", "passive") else "rl_control"
        if self.state == "lie_down":
            done = t >= st["lie_down"]["duration_s"] + st["lie_down"]["hold_s"]
            if done and cmd.target in ("stand", "rl_control"):
                return "stand_up"
            return "passive" if done else "lie_down"
        if self.state == "joint_damping":
            if cmd.target == "passive" and not self.fault:
                return "passive"
            return "joint_damping"
        raise ValueError(self.state)

    def tick(self, s: RobotState, cmd: UserCommand) -> JointCommand:
        self.fault = self.fault or self.check_faults(s)
        self.last_stamp = s.stamp
        nxt = self.next_state(cmd)
        if nxt != self.state:
            self.enter(nxt, s)
        out = getattr(self, "run_" + self.state)(s, cmd)
        self.state_time += self.dt
        self.tick_count += 1
        return out

    def run_passive(self, s: RobotState, cmd: UserCommand) -> JointCommand:
        p = self.cfg["states"]["passive"]
        c = JointCommand.zero()
        c.kp, c.kd = tile(p["kp"], JOINTS), tile(p["kd"], JOINTS)
        c.q = s.q.copy()
        return c

    def run_joint_damping(self, s: RobotState, cmd: UserCommand) -> JointCommand:
        p = self.cfg["states"]["joint_damping"]
        c = JointCommand.zero()
        c.kp, c.kd = tile(p["kp"], JOINTS), tile(p["kd"], JOINTS)
        c.q = s.q.copy()
        return c

    def run_stand_up(self, s: RobotState, cmd: UserCommand) -> JointCommand:
        p = self.cfg["states"]["stand_up"]
        c = JointCommand.zero()
        c.kp, c.kd = tile(p["kp"], JOINTS), tile(p["kd"], JOINTS)
        t, T1, T2 = self.state_time, p["crouch_duration_s"], p["rise_duration_s"]
        if t <= T1:
            goal = leg_pose_for_height(self.robot, p["crouch_height"], p["hipx"])
            for i in range(JOINTS):
                c.q[i], c.dq[i] = cubic(self.enter_q[i], self.enter_dq[i], goal[i], 0.0, t, T1)
            c.kd[IS_WHEEL] = 0.0
            c.q[IS_WHEEL], c.dq[IS_WHEEL] = 0.0, 0.0
        else:
            h, _ = cubic(p["crouch_height"], 0, p["stand_height"], 0, t - T1, T2)
            h_next, _ = cubic(p["crouch_height"], 0, p["stand_height"], 0, t - T1 + self.dt, T2)
            c.q = leg_pose_for_height(self.robot, h, p["hipx"])
            c.dq = (leg_pose_for_height(self.robot, h_next, p["hipx"]) - c.q) / self.dt
            c.kd[IS_WHEEL] = p["wheel_kd_after_crouch"]
            c.q[IS_WHEEL], c.dq[IS_WHEEL] = 0.0, 0.0
        return c

    def run_lie_down(self, s: RobotState, cmd: UserCommand) -> JointCommand:
        p = self.cfg["states"]["lie_down"]
        c = JointCommand.zero()
        t, T = self.state_time, p["duration_s"]
        if t <= T:
            goal = leg_pose_for_height(self.robot, p["lie_height"], p["hipx"])
            c.kp, c.kd = tile(p["kp"], JOINTS), tile(p["kd"], JOINTS)
            for i in range(JOINTS):
                c.q[i], c.dq[i] = cubic(self.enter_q[i], self.enter_dq[i], goal[i], 0.0, t, T)
            c.q[IS_WHEEL], c.dq[IS_WHEEL] = 0.0, 0.0
        else:
            c.kd = tile(p["kd"], JOINTS)
        return c

    def run_rl_control(self, s: RobotState, cmd: UserCommand) -> JointCommand:
        r = self.select_runner(cmd)
        if r is not self.runner:
            r.reset()
            self.runner, self.targets, self.policy_tick = r, None, 0
        r.params.update(cmd.params)
        if self.policy_tick % self.decimation == 0:
            self.targets = r.step(s, cmd.axes)
        self.policy_tick += 1
        c = JointCommand.zero()
        c.kp, c.kd = r.kp.copy(), r.kd.copy()
        c.q = self.targets.q.copy()
        c.q[IS_WHEEL] = 0.0
        c.dq[IS_WHEEL] = self.targets.wheel_velocity
        c.kp[IS_WHEEL] = 0.0
        return c
