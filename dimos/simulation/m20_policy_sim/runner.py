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
    return rotation(quat).T @ np.array([0.0, 0.0, -1.0])


def joint_limits(robot: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """Policy target clamp: the joint range, hipx narrowed to mid +- fraction * half range."""
    lo = np.array(robot["joint_lower"], np.float64)
    hi = np.array(robot["joint_upper"], np.float64)
    mid, half = (lo + hi) / 2, (hi - lo) / 2
    frac = robot["hipx_limit_fraction"]
    lo = np.where(IS_HIPX, mid - frac * half, lo)
    hi = np.where(IS_HIPX, mid + frac * half, hi)
    return lo, hi


@dataclass
class Targets:
    """A policy step's output in robot order."""

    q: np.ndarray  # leg position targets (wheels: unused)
    wheel_velocity: np.ndarray  # 4 wheel velocity targets
    action: np.ndarray  # the clipped network output (next step's last action)
    velocity_estimate: np.ndarray | None


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
        self.wheel_limit = robot["wheel_velocity_limit"]
        self.params: dict[str, float] = dict(spec.get("command_params", {}))
        self.policy_dt = 0.0
        self.reset()

    def reset(self) -> None:
        self.last_action = np.zeros(JOINTS)
        self.history: np.ndarray | None = None
        self.phase = 0.0
        self.command = np.zeros(3)

    def update_command(self, axes: np.ndarray) -> np.ndarray:
        """Normalized user axes [-1, 1] -> physical (vx, vy, wz), slew-limited per policy step where configured."""
        target = np.clip(axes, -1, 1) * self.command_max
        if "wz_clamp" in self.spec and target[0] > self.spec["wz_clamp_above_vx"]:
            target[2] = np.clip(target[2], -self.spec["wz_clamp"], self.spec["wz_clamp"])
        slew = self.command_slew
        self.command = np.where(
            slew > 0, self.command + np.clip(target - self.command, -slew, slew), target
        )
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
                v = s.accel
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
        frame = np.concatenate(parts).astype(np.float32)
        if len(frame) != self.spec["frame"]:
            raise ValueError(
                f"{self.name}: frame is {len(frame)}, config says {self.spec['frame']}"
            )
        return frame

    def network_input(self, frame: np.ndarray) -> dict[str, np.ndarray]:
        n = self.spec["history"]
        if self.spec["kind"] == "frames":
            self.history = (
                np.tile(frame, (n, 1))
                if self.history is None
                else np.vstack([self.history[1:], frame])
            )
            return {self.input_names[0]: np.concatenate([frame, self.history.ravel()])[None]}
        self.history = (
            np.tile(frame, (n, 1)) if self.history is None else np.vstack([self.history[1:], frame])
        )
        command = []
        for entry in self.spec["command_vector"]:
            if isinstance(entry, str):
                command.append(
                    {"vx": self.command[0], "vy": self.command[1], "wz": self.command[2]}.get(
                        entry, self.params.get(entry, 0.0)
                    )
                )
            else:
                command.append(entry)
        return {
            "state": self.history[None].astype(np.float32),
            "command": np.array([command], np.float32),
            "uncertain": np.zeros((1, 1), np.float32),
        }

    def step(self, s: RobotState, axes: np.ndarray) -> Targets:
        self.update_command(np.asarray(axes, np.float64))
        outputs = self.session.run(None, self.network_input(self.frame(s)))
        act = self.spec["action"]
        a = np.clip(outputs[0][0].astype(np.float64), -act["clip"], act["clip"])
        self.last_action = a
        for block in self.spec["obs"]:
            if block["kind"] == "phase":
                self.phase = (self.phase + self.policy_dt * block["frequency"]) % 1.0
        if act["layout"] == "per_leg":
            leg_a, leg_idx, wheel_a = a[LEGS_PER_LEG], LEGS_PER_LEG, a[WHEELS]
        else:
            leg_a, leg_idx, wheel_a = a[:12], LEGS_BY_TYPE, a[12:16]
        q = self.default.copy()
        scale = tile(act["leg_scale"], 12)
        q[leg_idx] = scale * leg_a + (self.default[leg_idx] if act["leg_mode"] == "offset" else 0.0)
        q = np.clip(q, *self.limits)
        wheel = np.clip(act["wheel_scale"] * wheel_a, -self.wheel_limit, self.wheel_limit)
        vel = outputs[1][0].astype(np.float64) if len(outputs) > 1 else None
        return Targets(q=q, wheel_velocity=wheel, action=a, velocity_estimate=vel)
