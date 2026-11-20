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

States (rl_deploy names): passive (idle_state) -> stand_up -> rl_control (runner by gait code, after a stand-in policy
on some gaits) -> lie_down; joint_damping on a request or a fault (stale state, tilt, joint beyond its range).
One call to `tick` per 1 ms control step returns the 16 joint commands. See SPEC.md for the decoded rules.
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
from dimos.simulation.m20_policy_sim.runner import RobotState, Runner, Targets

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


def roll_pitch(quat: np.ndarray) -> tuple[float, float]:
    w, x, y, z = quat
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


@dataclass
class StandIn:
    spec: dict[str, Any]
    duration: float
    elapsed: float = 0.0

    def params(self) -> dict[str, float]:
        return self.spec["params"] if self.elapsed < self.duration else self.spec["settle_params"]

    def done(self) -> bool:
        return self.elapsed >= self.duration + self.spec["settle_s"]


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
        self.time = 0.0
        self.tick_count = 0
        self.runner: Runner | None = None
        self.targets: Targets | None = None
        self.enter_q = np.zeros(JOINTS)
        self.enter_dq = np.zeros(JOINTS)
        self.last_stamp: float | None = None
        self.fault = ""
        self.blocked_target: str | None = None
        self.rl_entries = 0
        self.stand_in: StandIn | None = None
        self.gait = 0
        self.entry_raw_gait = 0
        self.policy_tick = 0
        self.transitions: list[tuple[float, str, str]] = []

    def runner_named(self, name: str) -> Runner:
        if name not in self.runners:
            r = Runner(name, self.cfg["runners"][name], self.robot, self.policies)
            r.policy_dt = self.dt * self.decimation
            self.runners[name] = r
        return self.runners[name]

    def check_faults(self, s: RobotState) -> str:
        safety = self.cfg["safety"]
        if self.time < safety["startup_grace_s"]:
            return ""
        if self.last_stamp is not None and s.stamp - self.last_stamp > safety["state_timeout_s"]:
            return "stale state"
        lo, hi = np.array(self.robot["joint_lower"]), np.array(self.robot["joint_upper"])
        margin = safety["joint_limit_margin_rad"]
        if np.any(s.q < lo - margin) or np.any(s.q > hi + margin):
            return "joint beyond its range"
        roll, pitch = (abs(math.degrees(a)) for a in roll_pitch(s.quat))
        st = self.cfg["states"]
        if self.state == "passive" and max(roll, pitch) > st["passive"]["max_tilt_deg"]:
            return "tilt"
        if self.state == "stand_up" and max(roll, pitch) > st["stand_up"]["max_tilt_deg"]:
            return "tilt"
        if self.state == "rl_control" and self.runner is not None:
            spec = self.runner.spec
            max_pitch = spec.get("max_pitch_deg", st["rl_control"]["max_pitch_deg"])
            if spec.get("tilt_check", True) and (
                roll > st["rl_control"]["max_roll_deg"] or pitch > max_pitch
            ):
                return "tilt"
        return ""

    def effective_target(self, cmd: UserCommand) -> str:
        """After a fault the operator's standing request is ignored until they send a different one."""
        if self.blocked_target is not None:
            if cmd.target == self.blocked_target:
                return "passive"
            self.blocked_target = None
        return cmd.target

    def next_state(self, target: str) -> str:
        st = self.cfg["states"]
        t = self.state_time
        if self.state != "joint_damping" and (target == "joint_damping" or self.fault):
            return "joint_damping"
        if self.state == "passive":
            ready = self.time >= st["passive"]["min_s_before_stand"]
            return "stand_up" if ready and target in ("stand", "rl_control") else "passive"
        if self.state == "stand_up":
            if t > 2 * st["stand_up"]["phase_s"]:
                if target == "rl_control":
                    return "rl_control"
                if target in ("lie_down", "passive"):
                    return "lie_down"
            return "stand_up"
        if self.state == "rl_control":
            if target in ("lie_down", "passive") and self.stand_in is None:
                return "lie_down"
            return "rl_control"
        if self.state == "lie_down":
            if t > 2 * st["lie_down"]["phase_s"] and target in ("stand", "rl_control"):
                return "stand_up"
            return "lie_down"
        if self.state == "joint_damping":
            if not self.fault and (t >= st["joint_damping"]["min_s"] or target != "joint_damping"):
                return "passive"
            return "joint_damping"
        raise ValueError(self.state)

    def enter(self, state: str, s: RobotState, cmd: UserCommand) -> None:
        self.transitions.append((self.time, self.state, state))
        if state == "joint_damping" and self.fault:
            self.blocked_target = cmd.target
        self.state = state
        self.state_time = 0.0
        self.enter_q, self.enter_dq = s.q.copy(), s.dq.copy()
        if state == "rl_control":
            rl = self.cfg["states"]["rl_control"]
            self.entry_raw_gait = cmd.gait
            self.gait = cmd.gait if cmd.gait in rl["entry_gaits"] else rl["entry_fallback_gait"]
            self.runner, self.targets = None, None
            self.start_stand_in(self.gait, on_entry=True)
            self.rl_entries += 1

    def start_stand_in(self, gait: int, on_entry: bool) -> None:
        self.stand_in = None
        for spec in self.cfg["states"]["rl_control"].get("stand_in", []):
            if spec["gait"] == gait and (on_entry or spec["on_switch"]):
                self.stand_in = StandIn(
                    spec, spec["first_s"] if self.rl_entries == 0 else spec["later_s"]
                )

    def tick(self, s: RobotState, cmd: UserCommand) -> JointCommand:
        self.fault = self.check_faults(s)
        self.last_stamp = s.stamp
        target = self.effective_target(cmd)
        nxt = self.next_state(target)
        if nxt != self.state:
            self.enter(nxt, s, cmd)
        out = getattr(self, "run_" + self.state)(s, cmd)
        self.state_time += self.dt
        self.time += self.dt
        self.tick_count += 1
        return out

    def gains(self, p: dict[str, Any]) -> JointCommand:
        c = JointCommand.zero()
        c.kp, c.kd = tile(p["kp"], JOINTS), tile(p["kd"], JOINTS)
        return c

    def run_passive(self, s: RobotState, cmd: UserCommand) -> JointCommand:
        return self.gains(self.cfg["states"]["passive"])

    def run_joint_damping(self, s: RobotState, cmd: UserCommand) -> JointCommand:
        return self.gains(self.cfg["states"]["joint_damping"])

    def run_stand_up(self, s: RobotState, cmd: UserCommand) -> JointCommand:
        p = self.cfg["states"]["stand_up"]
        c = self.gains(p)
        t, T = self.state_time, p["phase_s"]
        if t <= T:
            goal = leg_pose_for_height(self.robot, p["crouch_height"], p["hipx"])
            for i in range(JOINTS):
                c.q[i], c.dq[i] = cubic(self.enter_q[i], self.enter_dq[i], goal[i], 0.0, t, T)
        else:
            h, _ = cubic(p["crouch_height"], 0, p["stand_height"], 0, t - T, T)
            h_next, _ = cubic(p["crouch_height"], 0, p["stand_height"], 0, t - T + self.dt, T)
            c.q = leg_pose_for_height(self.robot, h, p["hipx"])
            c.dq = (leg_pose_for_height(self.robot, h_next, p["hipx"]) - c.q) / self.dt
        c.q[IS_WHEEL], c.dq[IS_WHEEL] = 0.0, 0.0
        return c

    def run_lie_down(self, s: RobotState, cmd: UserCommand) -> JointCommand:
        p = self.cfg["states"]["lie_down"]
        t, T = self.state_time, p["phase_s"]
        c = self.gains(p)
        if t <= T:
            goal = leg_pose_for_height(self.robot, p["lie_height"], p["hipx"])
            for i in range(JOINTS):
                c.q[i], c.dq[i] = cubic(self.enter_q[i], self.enter_dq[i], goal[i], 0.0, t, T)
            c.q[IS_WHEEL], c.dq[IS_WHEEL] = 0.0, 0.0
        elif t <= 2 * T:
            c.kp[:] = 0.0
        else:
            c = JointCommand.zero()
        return c

    def run_rl_control(self, s: RobotState, cmd: UserCommand) -> JointCommand:
        params = cmd.params
        if self.stand_in is not None:
            name = self.stand_in.spec["runner"]
            params = self.stand_in.params()
            self.stand_in.elapsed += self.dt
            if self.stand_in.done():
                self.stand_in = None
        else:
            # a gait change after entry (rl_deploy's gait-change marker) is followed as is
            wanted = cmd.gait if cmd.gait != self.entry_raw_gait else self.gait
            if wanted != self.gait:
                self.gait = wanted
                self.start_stand_in(wanted, on_entry=False)
            rl = self.cfg["states"]["rl_control"]
            name = cmd.runner or runner_for_gait(self.cfg, self.gait) or rl["fallback_runner"]
        r = self.runner_named(name)
        if r is not self.runner:
            r.reset()
            self.runner, self.policy_tick = r, 0
        r.params.update(params)
        if self.policy_tick % self.decimation == 0:
            self.targets = r.step(s, cmd.axes)
        self.policy_tick += 1
        c = JointCommand.zero()
        scale = r.gain_scale()
        c.kp, c.kd = r.kp * scale, r.kd * scale
        c.q = self.targets.q.copy()
        c.q[IS_WHEEL] = 0.0
        c.dq[IS_WHEEL] = self.targets.wheel_velocity
        c.kp[IS_WHEEL] = 0.0
        return c
