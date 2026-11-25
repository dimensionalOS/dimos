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

"""One rl_deploy policy runner, built from its m20_controller.toml section: observation in, joint targets out."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any

import numpy as np
import onnxruntime as ort

from dimos.simulation.m20_policy_sim.config import (
    IS_HIPX,
    JOINT_SETS,
    JOINTS,
    LEGS_BY_TYPE,
    LEGS_PER_LEG,
    WHEELS,
    tile,
)


@dataclass
class RobotState:
    """What the hardware reports each 1 ms tick (robot joint order)."""

    quat: np.ndarray  # body orientation w, x, y, z
    gyro: np.ndarray  # body angular velocity (rad/s), body frame
    accel: np.ndarray  # accelerometer specific force (m/s^2), body frame; +9.81 z at rest
    q: np.ndarray  # joint positions (rad), 16
    dq: np.ndarray  # joint velocities (rad/s), 16
    tau: np.ndarray = field(default_factory=lambda: np.zeros(JOINTS))
    height_map: np.ndarray | None = None  # rows*cols relative terrain heights, for height_map obs
    stamp: float = 0.0


def rotation(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def projected_gravity(quat: np.ndarray) -> np.ndarray:
    return np.asarray(rotation(quat).T @ np.array([0.0, 0.0, -1.0]))


def joint_limits(robot: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """rl_deploy's policy target clamp (limit_goal_pos_vel): the configured ranges (wheel entries: velocity), hipx
    widened by policy_hipx_widen of its width at each end."""
    lo = np.array(robot["policy_limit_lower"], np.float64)
    hi = np.array(robot["policy_limit_upper"], np.float64)
    w = robot["policy_hipx_widen"]
    return np.where(IS_HIPX, lo * (1 + w) - hi * w, lo), np.where(
        IS_HIPX, hi * (1 + w) - lo * w, hi
    )


@dataclass
class Targets:
    """A policy step's output in robot order."""

    q: np.ndarray  # leg position targets (wheels: unused)
    wheel_velocity: np.ndarray  # 4 wheel velocity targets
    action: np.ndarray  # the clipped network output (next step's last action)
    velocity_estimate: np.ndarray | None


class GaitClock:
    """rl_deploy's gait clock (get_gait_phase / update_gait_freq) in its f32 arithmetic, as the Rust controller."""

    def __init__(self, spec: dict[str, Any]) -> None:
        self.spec = spec
        self.set_frequency(np.float32(spec["frequency"]))
        self.counter = np.float32(0.0)

    def set_frequency(self, f: np.float32) -> None:
        self.frequency = f
        self.half = np.float32(np.float32(1.0) / f) * np.float32(0.5)
        self.omega = np.float32(float(f) * math.pi * 2.0)

    def fields(self, dt: float) -> dict[str, float]:
        t = self.counter * np.float32(dt)
        sign = -1.0 if math.sin(float(t * self.omega)) < 0 else 1.0
        r = np.fmod(t, self.half)
        return {
            "half_period": float(self.half),
            "sign": sign,
            "remaining": float(self.half - r) if float(r) > 1e-6 else 0.0,
        }

    def advance(self, dt: float, command: np.ndarray) -> None:
        """update_gait_freq after the command is built, then the counter's step."""
        f32 = np.float32
        dt32 = f32(dt)
        period = f32(1.0) / self.frequency
        nxt = np.fmod((self.counter + f32(1.0)) * dt32, period)
        if nxt < np.fmod(self.counter * dt32, period):
            g = self.spec
            vx, vy, wz = (f32(c) for c in command[:3])
            f = max(vx / f32(g["vx_gain"]) + f32(1.0), abs(vy) / f32(g["vy_gain"]) + f32(1.0))
            f = f32(
                min(
                    max((abs(wz) / f32(g["wz_gain"]) + f32(1.0)) * f, f32(g["min_frequency"])),
                    f32(g["max_frequency"]),
                )
            )
            if f != self.frequency:
                self.counter = f32(f32(f32(1.0) / f) * f32(nxt / period)) / dt32
                self.set_frequency(f)
        self.counter = self.counter + f32(1.0)


class StandMode:
    """SixAxisStand's mode input (rl_deploy cal_command): -mode_byte, but 0 while armed and upright; the arming
    lasts 100 upright steps; on gait 0xf003 it re-arms whenever it is off (cmp w2, #0xf003 at 0xcb4cc)."""

    def __init__(self) -> None:
        self.armed, self.count, self.mode = False, 0, -1

    def step(self, mode_byte: bool, gait: int, gravity_z: float) -> None:
        self.mode = -int(mode_byte)
        if gait == 0xF003 and not self.armed:
            self.armed, self.count, self.mode = True, 0, 0
        if gravity_z < -0.9 and self.armed:
            self.mode = 0
            self.count += 1
            if self.count > 99:
                self.armed, self.count, self.mode = False, 0, -1


class Runner:
    def __init__(
        self, name: str, spec: dict[str, Any], robot: dict[str, Any], policy_dir: Path
    ) -> None:
        self.name = name
        self.spec = spec
        self.session = ort.InferenceSession(
            str(policy_dir / spec["policy"]), providers=["CPUExecutionProvider"]
        )
        self.input_names = [i.name for i in self.session.get_inputs()]
        self.default = np.array(spec["default_pose"], np.float64)
        self.kp = tile(spec["kp"], JOINTS)
        self.kd = tile(spec["kd"], JOINTS)
        self.command_max = np.array(spec["command_max"], np.float64)
        self.command_slew = np.array(spec.get("command_slew", [0.0, 0.0, 0.0]), np.float64)
        self.limits = joint_limits(robot)
        self.clamp_targets = bool(robot.get("policy_target_clamp", True))
        self.params: dict[str, float] = dict(spec.get("command_params", {}))
        self.policy_dt = 0.0
        self.reset()

    def reset(self) -> None:
        self.last_action = np.zeros(JOINTS)
        self.history: np.ndarray | None = None
        self.phase = 0.0
        self.command = np.zeros(3)
        self.command_state = np.zeros(3, np.float32)
        self.steps = 0
        self.active_steps = 0
        self.collected = 0
        self.params = dict(self.spec.get("command_params", {}))
        self.clock = GaitClock(self.spec["gait_clock"]) if "gait_clock" in self.spec else None
        self.stand_mode = StandMode()
        self.gravity_z = -1.0

    def activate(self) -> None:
        """The runner becomes the active one (a gait switch): its start-up gains count from here."""
        self.active_steps = 0

    def on_exit(self, handover: dict[str, np.ndarray]) -> None:
        """rl_deploy's OnExit: the last action into the handover as leg positions (per leg) and wheel velocities."""
        act = self.spec["action"]
        a = self.last_action
        legs = a[LEGS_PER_LEG]
        if act["leg_mode"] != "absolute":
            legs = self.default[LEGS_PER_LEG] + act["leg_scale"][0] * legs
        handover["legs"] = legs.astype(np.float32)
        handover["wheels"] = (a[WHEELS] * act["wheel_scale"]).astype(np.float32)

    def on_enter(self, handover: dict[str, np.ndarray], per_leg: bool) -> None:
        """rl_deploy's OnEnter: the handover becomes the last action, laid out per leg (per_leg runners, entries) or
        by joint type; history, command slew, counters and the stand flags start over; the phase carries on."""
        act = self.spec["action"]
        legs = handover["legs"].astype(np.float64)
        if act["leg_mode"] != "absolute":
            legs = (legs - self.default[LEGS_PER_LEG]) / act["leg_scale"][0]
        wheels = handover["wheels"].astype(np.float64) / act["wheel_scale"]
        handover["legs"], handover["wheels"] = legs.astype(np.float32), wheels.astype(np.float32)
        a = np.zeros(JOINTS)
        if act["leg_mode"] == "absolute" or per_leg:
            a[LEGS_PER_LEG], a[WHEELS] = legs, wheels
        else:
            a[:12] = legs[[3 * (k % 4) + k // 4 for k in range(12)]]
            a[12:] = wheels
        self.last_action = a
        self.history = None
        self.command = np.zeros(3)
        self.command_state = np.zeros(3, np.float32)
        if self.clock is not None:
            self.clock.counter = np.float32(0.0)
        self.stand_mode.armed = False
        self.params = dict(self.spec.get("command_params", {}))
        self.steps = self.active_steps = self.collected = 0

    def gain_scale(self) -> float:
        """rl_deploy scales kp / kd for a runner's first steps after each switch to it (NaviCharge x1.3 for 29)."""
        if self.active_steps < self.spec.get("start_gain_steps", 0):
            return float(self.spec["start_gain_scale"])
        return 1.0

    def update_command(self, axes: np.ndarray) -> np.ndarray:
        """Normalized user axes [-1, 1] -> physical (vx, vy, wz) as rl_deploy's limit_cmd_vel does it, in f32: the
        normalized command moves toward the axes by at most command_slew per policy step, then x command_max."""
        axes = np.clip(axes, -1, 1)
        deadband = np.array(self.spec.get("axis_deadband", [0.0, 0.0, 0.0]))
        axes = np.where(np.abs(axes) < deadband, 0.0, axes).astype(np.float32)
        f32 = np.float32
        for i in range(3):
            limit = f32(self.command_slew[i])
            if limit > 0:
                d = f32(axes[i] - self.command_state[i])
                if limit < abs(d):
                    d = f32(np.copysign(limit, d))
                self.command_state[i] = f32(d + self.command_state[i])
            else:
                self.command_state[i] = axes[i]
        target = (self.command_state * self.command_max.astype(f32)).astype(np.float64)
        if "wz_speed_limit" in self.spec:
            w = self.spec["wz_speed_limit"]
            w_max = min(
                w["a"] / (target[0] + w["b"]) - w["c"], w["d"] / (1 + w["e"] * abs(target[1]))
            )
            target[2] = float(self.command_state[2]) * min(max(w_max, w["lo"]), w["hi"])
        if "wz_clamp" in self.spec and target[0] > self.spec["wz_clamp_above_vx"]:
            target[2] = np.clip(target[2], -self.spec["wz_clamp"], self.spec["wz_clamp"])
        self.command = target
        return self.command

    def frame(self, s: RobotState) -> np.ndarray:
        parts: list[np.ndarray] = []
        for block in self.spec["obs"]:
            kind = block["kind"]
            if kind == "gyro":
                v = s.gyro
            elif kind == "gravity":
                v = projected_gravity(s.quat)
            elif kind == "accel":
                v = s.accel / block["divisor"] if "divisor" in block else s.accel
            elif kind == "command":
                v = self.command
            elif kind == "constant":
                v = np.array(
                    [self.params.get(block["name"], block["values"][0])]
                    if "name" in block
                    else block["values"]
                )
            elif kind == "phase":
                v = np.array(
                    [math.sin(2 * math.pi * self.phase), math.cos(2 * math.pi * self.phase)]
                )
            elif kind == "joint_pos":
                idx = JOINT_SETS[block["joints"]]
                v = s.q[idx] - (self.default[idx] if block.get("relative", True) else 0.0)
            elif kind == "joint_vel":
                v = s.dq[JOINT_SETS[block["joints"]]]
            elif kind == "last_action":
                v = self.last_action
            elif kind == "height_map":
                n = block["rows"] * block["cols"]
                v = np.zeros(n) if s.height_map is None else np.asarray(s.height_map, np.float64)
            else:
                raise ValueError(f"unknown obs block {kind}")
            v = np.asarray(v, np.float64)
            if "scale" in block:
                v = v * tile(block["scale"], len(v))
            parts.append(v)
        frame: np.ndarray = np.concatenate(parts).astype(np.float32)
        if len(frame) != self.spec["frame"]:
            raise ValueError(
                f"{self.name}: frame is {len(frame)}, config says {self.spec['frame']}"
            )
        return frame

    def network_input(self, frame: np.ndarray) -> dict[str, np.ndarray]:
        n = self.spec["history"]
        if self.spec["kind"] == "frames":
            self.history = (
                np.vstack([np.zeros((n - 1, len(frame)), np.float32), frame[None]])
                if self.history is None
                else np.vstack([self.history[1:], frame])
            )
            return {self.input_names[0]: np.concatenate([frame, self.history.ravel()])[None]}
        self.history = (
            np.vstack([np.zeros((n - 1, len(frame)), np.float32), frame[None]])
            if self.history is None
            else np.vstack([self.history[1:], frame])
        )
        command = self.sequence_command(s_gravity_z=self.gravity_z)
        return {
            "state": self.history[None].astype(np.float32),
            "command": np.array([command], np.float32),
            "uncertain": np.zeros((1, 1), np.float32),
        }

    def sequence_command(self, s_gravity_z: float) -> list[float]:
        """The command input; the gait clock and stand mode advance (every step, warm-up included)."""
        p = self.params
        self.stand_mode.step(bool(p.get("mode_byte", 0.0)), int(p.get("gait", 0xF001)), s_gravity_z)
        command = [self.command_entry(e) for e in self.spec["command_vector"]]
        if self.clock is not None:
            self.clock.advance(self.policy_dt, self.command)
        return command

    def command_entry(self, e: Any) -> float:
        if not isinstance(e, dict):
            return float(e)
        src = e["src"]
        if src in ("vx", "vy", "wz"):
            return float(self.command[("vx", "vy", "wz").index(src)])
        if src == "param":
            k = math.radians(e["scale_deg"]) if "scale_deg" in e else e.get("scale", 1.0)
            return float(self.params.get(e["name"], 0.0) * k + e.get("offset", 0.0))
        if src == "stand_mode":
            return float(self.stand_mode.mode)
        if src == "gait":
            if self.clock is None:
                raise ValueError(f"{self.name}: a gait command entry needs a gait clock")
            return float(self.clock.fields(self.policy_dt)[e["field"]] * e.get("scale", 1.0))
        if src == "foot":
            f = self.spec["feet"]
            nx, ny = f["nominal"][e["leg"]]
            if self.params.get("feet_fixed", 0.0):
                return float(nx if e["axis"] == "x" else f["fixed_y"][e["leg"]])
            th = math.radians(f["yaw_deg"]) * self.params.get("yaw", 0.0)
            x = math.cos(th) * nx - math.sin(th) * ny + f["dx_scale"] * self.params.get("dx", 0.0)
            y = math.sin(th) * nx + math.cos(th) * ny + f["dy_scale"] * self.params.get("dy", 0.0)
            return float(x if e["axis"] == "x" else y)
        raise ValueError(f"unknown command source {src}")

    def warmup(self, s: RobotState) -> Targets:
        """A fresh sequence runner only collects frames for its first warmup_steps, holding the default pose."""
        self.last_frame = self.frame(s)
        n = self.spec["history"]
        f = self.last_frame[None]
        self.history = f if self.history is None else np.vstack([self.history, f])[-n:]
        self.sequence_command(s_gravity_z=self.gravity_z)
        act = self.spec["action"]
        a = np.zeros(JOINTS)
        if act["leg_mode"] == "absolute":
            idx = LEGS_PER_LEG if act["layout"] == "per_leg" else LEGS_BY_TYPE
            pos = np.arange(JOINTS)[idx] if act["layout"] == "per_leg" else np.arange(12)
            a[pos] = self.default[idx] / tile(act["leg_scale"], 12)
        self.last_action = a
        self.collected += 1
        q = self.default.copy()
        return Targets(q=q, wheel_velocity=np.zeros(4), action=a, velocity_estimate=None)

    def step(self, s: RobotState, axes: np.ndarray) -> Targets:
        self.update_command(np.asarray(axes, np.float64))
        self.gravity_z = float(projected_gravity(s.quat)[2])
        gate = "warmup_param" not in self.spec or self.params.get(self.spec["warmup_param"], 0.0)
        if gate and self.collected < self.spec.get("warmup_steps", 0):
            return self.warmup(s)
        # the gait phase advances before it is observed (rl_deploy: the first frame after a reset sees one step)
        for block in self.spec["obs"]:
            if block["kind"] == "phase":
                self.phase = (self.phase + self.policy_dt * block["frequency"]) % 1.0
        self.last_frame = self.frame(s)
        outputs = self.session.run(None, self.network_input(self.last_frame))
        act = self.spec["action"]
        a = np.clip(outputs[0][0].astype(np.float64), -act["clip"], act["clip"])
        self.steps += 1
        self.active_steps += 1
        self.collected += 1
        self.last_action = a
        if act["layout"] == "per_leg":
            leg_a, leg_idx, wheel_a = a[LEGS_PER_LEG], LEGS_PER_LEG, a[WHEELS]
        else:
            leg_a, leg_idx, wheel_a = a[:12], LEGS_BY_TYPE, a[12:16]
        q = self.default.copy()
        scale = tile(act["leg_scale"], 12)
        q[leg_idx] = scale * leg_a + (self.default[leg_idx] if act["leg_mode"] == "offset" else 0.0)
        wheel = act["wheel_scale"] * wheel_a
        if self.clamp_targets:
            q[leg_idx] = np.clip(q[leg_idx], self.limits[0][leg_idx], self.limits[1][leg_idx])
            wheel = np.clip(wheel, self.limits[0][WHEELS], self.limits[1][WHEELS])
        vel = outputs[1][0].astype(np.float64) if len(outputs) > 1 else None
        return Targets(q=q, wheel_velocity=wheel, action=a, velocity_estimate=vel)
