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

"""OpenArm leaders with the seven-joint R1 Pro JoyLo control layout.

Layout reference: StanfordVL/BEHAVIOR-1K joylo at b544acbbb76b.
OpenArm is not a kinematic twin of R1 Pro: joint deltas are aligned at engagement.
Locks hold follower targets in software; they do not energize leader motors.
"""

from dataclasses import dataclass
import math

from pydantic import Field, model_validator

from dimos.core.module import ModuleConfig
from dimos.robot.galaxea.r1pro.config import R1PRO_GRIPPER_OPENING_RANGE
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS
from dimos.teleop.joycon.mapping import (
    GripperState,
    JoyConReading,
    JoyConSide,
    apply_deadzone,
    step_gripper,
)

SIDES = ("left", "right")
UPPER = tuple(f"r1pro/{name}" for name in UPPER_BODY_JOINTS)
ARMS = UPPER[4:]
TORSO = UPPER[:4]
JAWS = tuple(f"r1pro/{side}_gripper" for side in SIDES)
TORSO_LINK_M = 0.35


class JoyloConfig(ModuleConfig):
    enabled_sides: tuple[JoyConSide, ...] = ("left", "right")
    arm_only: bool = False
    input_timeout_s: float = Field(default=0.25, gt=0, allow_inf_nan=False)
    tick_period_s: float = Field(default=0.01, gt=0, le=0.05)
    arm_rate: float = Field(default=0.25, gt=0, le=0.5)
    torso_rate: float = Field(default=0.15, gt=0, le=0.3)
    height_rate: float = Field(default=0.03, gt=0, le=0.05)
    pitch_rate: float = Field(default=0.1, gt=0, le=0.2)
    base_speed: float = Field(default=0.1, gt=0, le=0.2)
    yaw_speed: float = Field(default=0.25, gt=0, le=0.5)
    deadzone: float = Field(default=0.15, ge=0, lt=1)
    max_leader_jump: float = Field(default=0.75, gt=0)

    @model_validator(mode="after")
    def _validate_sides(self) -> "JoyloConfig":
        if not self.enabled_sides or len(set(self.enabled_sides)) != len(self.enabled_sides):
            raise ValueError("enabled_sides must name one or two distinct sides")
        return self


def torso_step(
    q: list[float],
    lift: float,
    pitch: float,
    dt: float,
    limits: list[tuple[float, float]],
    config: JoyloConfig,
) -> list[float]:
    """Height at fixed fore/aft reach, upper pitch trim, and retained yaw.

    The pinned 2026 model has two +Y folding axes and a -Y third pitch axis.
    q1 + q2 - q3 is the upper pitch. Bounds apply to the coordinated step.
    """
    if not lift and not pitch:
        return q.copy()
    a, b, c, yaw = q
    x = TORSO_LINK_M * (math.sin(a) + math.sin(a + b))
    z = TORSO_LINK_M * (math.cos(a) + math.cos(a + b))
    ceiling = math.sqrt(max(0.0, (2 * TORSO_LINK_M) ** 2 - x * x))
    target_a, target_b = a, b
    if lift and 0.45 <= z <= ceiling + 1e-6:
        height = min(ceiling, max(0.45, z + lift * config.height_rate * dt))
        cosine = (x * x + height * height - 2 * TORSO_LINK_M**2) / (2 * TORSO_LINK_M**2)
        target_b = (1 if b > 0 else -1) * math.acos(max(-1, min(1, cosine)))
        target_a = math.atan2(x, height) - math.atan2(math.sin(target_b), 1 + math.cos(target_b))
    target_c = c + target_a + target_b - a - b - pitch * config.pitch_rate * dt
    target = [target_a, target_b, target_c, yaw]
    delta = [new - old for new, old in zip(target, q, strict=True)]
    scale = min(1.0, config.torso_rate * dt / max(1e-12, *map(abs, delta)))
    result = [old + scale * d for old, d in zip(q, delta, strict=True)]
    return (
        result
        if all(lo <= value <= hi for value, (lo, hi) in zip(result, limits, strict=True))
        else q.copy()
    )


@dataclass(frozen=True)
class JoyloOutput:
    positions: dict[str, float]
    openings: tuple[float, float]
    velocity: tuple[float, float, float]


class JoyloController:
    """Deterministic state machine; the module owns clocks, transport and locking."""

    def __init__(self, limits: dict[str, tuple[float, float]], config: JoyloConfig) -> None:
        if set(limits) != set(UPPER):
            raise ValueError("JoyLo needs limits for all 18 R1 Pro upper-body joints")
        self.limits = limits
        self.config = config
        self.arm_names = tuple(
            n
            for side in SIDES
            if side in config.enabled_sides
            for n in ARMS
            if n.startswith(f"r1pro/{side}_")
        )
        self.left_only = config.enabled_sides == ("left",)
        self.enable_button = "minus" if self.left_only else "X"
        self.pause_button = "capture" if self.left_only else "home"
        self.authority_index = 0 if self.left_only else 1
        self.active = False
        self.reason = f"Press {self.enable_button} to align and enable"
        self.leader: dict[str, float] = {}
        self.feedback: dict[str, float] = {}
        self.readings: tuple[JoyConReading | None, JoyConReading | None] = (None, None)
        self.receipts = {name: -math.inf for name in ("leader", "robot", "joycon")}
        self.target: dict[str, float] = {}
        self.anchor_leader: dict[str, float] = {}
        self.anchor_robot: dict[str, float] = {}
        self.grippers = [GripperState(opening=0), GripperState(opening=0)]
        self.gripper_touched = [False, False]
        self.arm_locked = [False, False]
        self.locked: set[str] = set()
        self.previous_buttons: tuple[frozenset[str], frozenset[str]] = (frozenset(), frozenset())
        self.enable_released = False
        self.capture_hold = True

    def pause(self, reason: str) -> None:
        self.capture_hold |= self.active
        self.active = False
        self.reason = reason
        self.enable_released = False

    def set_leader(self, positions: dict[str, float], now: float) -> None:
        if set(positions) != set(self.arm_names) or not all(map(math.isfinite, positions.values())):
            self.pause("Incomplete leader sample")
            self.receipts["leader"] = -math.inf
            return
        if self.active and now - self.receipts["leader"] > self.config.input_timeout_s:
            # A new sample can arrive before the watchdog tick notices the gap.
            # Recover the reading, but require a fresh enable press to align it.
            self.pause(
                f"Leader input resumed after timeout; release {self.enable_button} and re-align"
            )
        elif (
            self.leader
            and self.active
            and any(
                abs(value - self.leader[name]) > self.config.max_leader_jump
                for name, value in positions.items()
            )
        ):
            self.pause(f"Leader discontinuity; release {self.enable_button} and re-align")
        self.leader = positions.copy()
        self.receipts["leader"] = now

    def set_feedback(self, positions: dict[str, float], now: float) -> None:
        required = (*UPPER, *JAWS)
        if not all(name in positions and math.isfinite(positions[name]) for name in required):
            return
        self.feedback = positions.copy()
        self.receipts["robot"] = now

    def set_joycon(
        self, left: JoyConReading | None, right: JoyConReading | None, now: float
    ) -> None:
        self.readings = (left, right)
        self.receipts["joycon"] = now

    def _capture(self) -> None:
        self.target = {
            name: min(hi, max(lo, self.feedback[name])) for name, (lo, hi) in self.limits.items()
        }
        for i, joint in enumerate(JAWS):
            opening = max(0.0, min(1.0, self.feedback[joint] / R1PRO_GRIPPER_OPENING_RANGE[1]))
            self.grippers[i] = GripperState(opening=opening, closed=opening < 0.5)
        self.gripper_touched = [False, False]
        self.capture_hold = False

    def step(self, now: float, dt: float) -> JoyloOutput | None:
        dt = min(0.05, max(0.0, dt))
        stale = [
            name
            for name, receipt in self.receipts.items()
            if now - receipt > self.config.input_timeout_s
        ]
        left, right = self.readings
        healthy = not stale and all(
            self.readings[SIDES.index(side)] is not None for side in self.config.enabled_sides
        )
        if not healthy:
            self.pause(
                "Waiting for fresh "
                + (", ".join(stale) if stale else "/".join(self.config.enabled_sides) + " Joy-Con")
            )
            self.previous_buttons = (frozenset(), frozenset())
            if "robot" in stale:
                return None
        if self.capture_hold:
            self._capture()
        if healthy:
            left, right = left or JoyConReading(), right or JoyConReading()
            buttons = (left.pressed, right.pressed)
            authority = buttons[self.authority_index]
            x = self.enable_button in authority
            enable_edge = (
                x
                and self.enable_released
                and self.enable_button not in self.previous_buttons[self.authority_index]
            )
            if self.pause_button in authority or (self.left_only and self.active and enable_edge):
                self.pause(f"Paused; release {self.enable_button} before enabling")
                if self.capture_hold:
                    self._capture()
            elif enable_edge and not self.active:
                self._capture()
                self.anchor_leader = self.leader.copy()
                self.anchor_robot = self.target.copy()
                self.arm_locked = [False, False]
                self.locked.clear()
                self.active = True
                self.reason = "Following aligned OpenArm joint deltas"
                # A held trigger at engagement must be released before it toggles a jaw.
                for i, label in enumerate(("ZL", "ZR")):
                    self.grippers[i].trigger_was_down = label in buttons[i]
            if not x:
                self.enable_released = True
            if self.active:
                self._follow(left, right, buttons, dt)
            self.previous_buttons = buttons
        velocity = (0.0, 0.0, 0.0)
        if self.active and not self.config.arm_only and left is not None and right is not None:

            def axis(value: float) -> float:
                return apply_deadzone(value, self.config.deadzone)

            speed = self.config.base_speed * (2 if left.stick_pressed else 1)
            # Match the current R1 Pro reference: right click suppresses yaw and tilt.
            yaw = 0.0 if right.stick_pressed else -axis(right.stick[0]) * self.config.yaw_speed
            velocity = (-axis(left.stick[1]) * speed, -axis(left.stick[0]) * speed, yaw)
        return JoyloOutput(
            self.target.copy(), (self.grippers[0].opening, self.grippers[1].opening), velocity
        )

    def _follow(
        self,
        left: JoyConReading,
        right: JoyConReading,
        buttons: tuple[frozenset[str], frozenset[str]],
        dt: float,
    ) -> None:
        next_locked: set[str] = set()
        for i, (side, toggle, wrist) in enumerate(
            zip(SIDES, ("minus", "plus"), ("L", "R"), strict=True)
        ):
            if side not in self.config.enabled_sides:
                continue
            if (
                not self.left_only
                and toggle in buttons[i]
                and toggle not in self.previous_buttons[i]
            ):
                self.arm_locked[i] = not self.arm_locked[i]
            names = tuple(f"r1pro/{side}_arm_joint{j}" for j in range(1, 8))
            if self.arm_locked[i]:
                next_locked.update(names)
            if wrist in buttons[i]:
                next_locked.update(names[-3:])
            down = ("ZL", "ZR")[i] in buttons[i]
            self.gripper_touched[i] |= down and not self.grippers[i].trigger_was_down
            if self.gripper_touched[i]:
                step_gripper(self.grippers[i], down, mode="toggle", slew_per_s=3, dt=dt)
            else:
                self.grippers[i].trigger_was_down = down
        for name in self.arm_names:
            if name in self.locked and name not in next_locked:
                self.anchor_leader[name] = self.leader[name]
                self.anchor_robot[name] = self.target[name]
            if name in next_locked:
                continue
            lo, hi = self.limits[name]
            desired = min(
                hi, max(lo, self.anchor_robot[name] + self.leader[name] - self.anchor_leader[name])
            )
            delta = self.config.arm_rate * dt
            self.target[name] += max(-delta, min(delta, desired - self.target[name]))
        self.locked = next_locked
        if self.config.arm_only:
            return
        lift = float("up" in left.pressed) - float("down" in left.pressed)
        pitch = (
            0.0 if right.stick_pressed else -apply_deadzone(right.stick[1], self.config.deadzone)
        )
        torso = torso_step(
            [self.target[n] for n in TORSO],
            lift,
            pitch,
            dt,
            [self.limits[n] for n in TORSO],
            self.config,
        )
        self.target.update(zip(TORSO, torso, strict=True))
