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
Calibrated leader angles set fixed joint targets; enabling never changes their origin.
Locks hold follower targets in software; they do not energize leader motors.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
import math
from typing import TYPE_CHECKING, Literal

import numpy as np
from pydantic import Field, model_validator

from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.robot.galaxea.r1pro.config import R1PRO_GRIPPER_OPENING_RANGE, R1PRO_MODEL
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS
from dimos.teleop.joycon.mapping import (
    GripperState,
    JoyConReading,
    JoyConSide,
    apply_deadzone,
    step_gripper,
)

if TYPE_CHECKING:
    from numpy.typing import NDArray

SIDES = ("left", "right")
UPPER = tuple(f"r1pro/{name}" for name in UPPER_BODY_JOINTS)
ARMS = UPPER[4:]
TORSO = UPPER[:4]
JAWS = tuple(f"r1pro/{side}_gripper" for side in SIDES)


class JoyloConfig(ModuleConfig):
    model: RobotModel = R1PRO_MODEL
    torso_link_lengths: tuple[float, float] = (0.35, 0.35)
    enabled_sides: tuple[JoyConSide, ...] = ("left", "right")
    arm_only: bool = False
    # Release the impedance task to the simulated servo's measured-position hold on pause.
    release_arm_impedance_on_pause: bool = False
    input_timeout_s: float = Field(default=0.25, gt=0, allow_inf_nan=False)
    # Stop motion at input_timeout_s, but retain enable intent across a brief
    # transport gap. Fresh inputs are required before any motion resumes.
    input_recovery_timeout_s: float = Field(default=1.0, gt=0, le=2, allow_inf_nan=False)
    tick_period_s: float = Field(default=0.01, gt=0, le=0.05)
    height_rate: float = Field(default=0.03, gt=0, le=0.05)
    pitch_rate: float = Field(default=0.1, gt=0, le=0.2)
    base_speed: float = Field(default=0.1, ge=0, le=0.5)
    yaw_speed: float = Field(default=0.25, ge=0, le=2.0)
    deadzone: float = Field(default=0.15, ge=0, lt=1)
    max_leader_jump: float = Field(default=0.75, gt=0)
    # A tracked hand that moves this far between two samples was lost and found
    # again somewhere else; following stops, as for a leader discontinuity.
    max_hand_jump_m: float = Field(default=0.2, gt=0)
    # Feetech calibrated readings wrap at +/-pi. Resolve equivalent angles
    # against joint limits and use periodic deltas for the discontinuity guard.
    leader_angle_period: float | None = Field(default=None, gt=0, allow_inf_nan=False)

    @model_validator(mode="after")
    def _validate_sides(self) -> JoyloConfig:
        if self.input_recovery_timeout_s < self.input_timeout_s:
            raise ValueError("input_recovery_timeout_s must be at least input_timeout_s")
        if not self.enabled_sides or len(set(self.enabled_sides)) != len(self.enabled_sides):
            raise ValueError("enabled_sides must name one or two distinct sides")
        if not all(math.isfinite(v) and v > 0 for v in self.torso_link_lengths):
            raise ValueError("torso link lengths must be finite and positive")
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

    Both vendor variants have two +Y folding axes and a -Y third pitch axis.
    q1 + q2 - q3 is the upper pitch. Bounds apply to the coordinated step.
    """
    if not lift and not pitch:
        return q.copy()
    a, b, c, yaw = q
    l1, l2 = config.torso_link_lengths
    x = l1 * math.sin(a) + l2 * math.sin(a + b)
    z = l1 * math.cos(a) + l2 * math.cos(a + b)
    ceiling = math.sqrt(max(0.0, (l1 + l2) ** 2 - x * x))
    floor = math.sqrt(max(0.0, (l1 - l2) ** 2 - x * x))
    height = z
    if lift and floor - 1e-6 <= z <= ceiling + 1e-6:
        height = min(ceiling, max(floor, z + lift * config.height_rate * dt))

    def candidate(fraction: float) -> list[float]:
        target_a, target_b = a, b
        if height != z:
            target_height = z + fraction * (height - z)
            cosine = (x * x + target_height**2 - l1 * l1 - l2 * l2) / (2 * l1 * l2)
            target_b = (1 if b > 0 else -1) * math.acos(max(-1, min(1, cosine)))
            target_a = math.atan2(x, target_height) - math.atan2(
                l2 * math.sin(target_b), l1 + l2 * math.cos(target_b)
            )
        target_c = c + target_a + target_b - a - b - pitch * config.pitch_rate * dt * fraction
        return [target_a, target_b, target_c, yaw]

    def within_limits(target: list[float]) -> bool:
        return all(lo <= value <= hi for value, (lo, hi) in zip(target, limits, strict=True))

    target = candidate(1.0)
    if within_limits(target):
        return target
    if not within_limits(q):
        return q.copy()
    # Stop at the actual joint boundary, retaining reach and upper-body pitch.
    # Rejecting a whole step leaves unused travel and can make reversal sticky.
    low, high = 0.0, 1.0
    result = q.copy()
    for _ in range(24):
        fraction = (low + high) / 2
        target = candidate(fraction)
        if within_limits(target):
            low, result = fraction, target
        else:
            high = fraction
    return result


ArmSource = Literal["leader", "hands"]
# A pose: position (x, y, z) in metres and orientation (x, y, z, w). A hand's is in
# the frame the headset module gives its arm commands in: x forward, y left, z up
# as the headset's space lies, from that space's origin.
HandPose = tuple[tuple[float, float, float], tuple[float, float, float, float]]
# The operator's head in the frame of the hands: its position, and the heading the
# operator faces in radians about z.
Head = tuple[tuple[float, float, float], float]

# Targets are published in the IK model's base frame. Hand poses are first
# attached to the neutral upper body, then carried by its measured transform.
BASE_FRAME = "base_link"
# The robot's eyes in that frame: the point between the head cameras with the
# torso straight up, in metres from the model.
ROBOT_EYES = (-0.014, 0.0, 1.624)
# From a wrist as the headset module gives it to the gripper link. The link and a
# headset's wrist joint have the same axes (-z out along the fingers, y across the
# palm), but the module turns a pose a quarter turn about z for a controller's grip,
# one way per side, and the arms' natural roll differs by a half turn between the
# sides. The two add up to this one turn for both sides.
_WRIST_TO_GRIPPER = (0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5))


def _turned(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    """The orientation ``b`` in a frame that has the orientation ``a``: a * b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


@dataclass(frozen=True)
class HandsOutput:
    """What the headset arm task gets while the hands are the arm source."""

    # Whether the arms follow.
    engaged: bool
    # Where the left and the right gripper go, in BASE_FRAME: as far from the
    # robot's eyes as each hand is from where the operator's were at the enable
    # press. A locked arm keeps the target of the pose its hand was locked at, and
    # a held wrist keeps its orientation.
    poses: tuple[HandPose, HandPose] | None


@dataclass(frozen=True)
class JoyloOutput:
    positions: dict[str, float]
    openings: tuple[float, float]
    velocity: tuple[float, float, float]
    # None while the leader arms are the arm source.
    hands: HandsOutput | None = None


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
        self.recovery_deadline: float | None = None
        self.hold_count = 0
        self.pause_count = 0
        self.last_pause_reason: str | None = None
        self.reason = f"Press {self.enable_button} to enable tracking"
        self.leader: dict[str, float] = {}
        self.feedback: dict[str, float] = {}
        self.readings: tuple[JoyConReading | None, JoyConReading | None] = (None, None)
        self.receipts = {name: -math.inf for name in ("leader", "robot", "joycon")}
        # The arms follow the leader arms, or the operator's hands as a headset sees them.
        self.arm_source: ArmSource = "leader"
        self.hands: list[HandPose | None] = [None, None]
        self.hand_targets_in_base = [False, False]
        self.hand_receipts = [-math.inf, -math.inf]
        self.held_hands: list[HandPose | None] = [None, None]
        self.head: Head | None = None
        self.head_receipt = -math.inf
        # The operator's head at the enable press: the hands are measured from it.
        self.anchor: Head | None = None
        self.target: dict[str, float] = {}
        self.grippers = [GripperState(opening=0), GripperState(opening=0)]
        self.gripper_touched = [False, False]
        self.arm_locked = [False, False]
        self.locked: set[str] = set()
        self.previous_buttons: tuple[frozenset[str], frozenset[str]] = (frozenset(), frozenset())
        self.enable_released = False
        self.capture_hold = True

    def pause(self, reason: str) -> None:
        if self.active or self.recovery_deadline is not None:
            self.pause_count += 1
            self.last_pause_reason = reason
        self.capture_hold |= self.active
        self.active = False
        self.recovery_deadline = None
        self.reason = reason
        self.enable_released = False

    def _hold_for_input(self, now: float, reason: str) -> None:
        if self.active:
            self.recovery_deadline = (
                min(self._arm_receipt(), self.receipts["robot"], self.receipts["joycon"])
                + self.config.input_recovery_timeout_s
            )
            self.active = False
            self.capture_hold = True
            self.hold_count += 1
        if self.recovery_deadline is not None and now < self.recovery_deadline:
            self.reason = "Holding for fresh " + reason
        else:
            self.pause("Waiting for fresh " + reason)

    def set_arm_source(self, source: ArmSource) -> None:
        """Take the arms from the leader arms or from the headset's hands; then enable anew."""
        if source not in ("leader", "hands"):
            raise ValueError("arm source must be 'leader' or 'hands'")
        if source == "hands" and (self.left_only or self.config.arm_only):
            raise ValueError("the hands source needs both Joy-Cons and the full profile")
        if source == self.arm_source:
            return
        self.pause("Arm source changed")
        self.arm_source = source
        self.hands, self.hand_receipts = [None, None], [-math.inf, -math.inf]
        self.head, self.head_receipt, self.anchor = None, -math.inf, None
        self.reason = f"Press {self.enable_button} to enable tracking"

    def set_hand(self, side: JoyConSide, pose: HandPose, now: float) -> None:
        """One hand's pose from the headset. Both hands make the arm source fresh."""
        values = (*pose[0], *pose[1])
        i = SIDES.index(side)
        if not all(map(math.isfinite, values)) or abs(math.hypot(*pose[1]) - 1) > 0.01:
            self.pause("Malformed hand pose")
            self.hand_receipts[i] = -math.inf
            return
        previous = self.hands[i]
        if (
            previous is not None
            and (self.active or self.recovery_deadline is not None)
            and math.dist(previous[0], pose[0]) > self.config.max_hand_jump_m
        ):
            self.pause(
                f"{side.capitalize()} hand jumped {math.dist(previous[0], pose[0]):.2f} m; "
                f"release {self.enable_button} and re-enable"
            )
        self.hands[i] = pose
        self.hand_receipts[i] = now

    def set_head(self, position: tuple[float, float, float], heading: float, now: float) -> None:
        """The operator's head from the headset: where the hands are measured from."""
        if not all(map(math.isfinite, (*position, heading))):
            self.pause("Malformed head pose")
            self.head_receipt = -math.inf
            return
        self.head = (position, heading)
        self.head_receipt = now

    def _arm_receipt(self) -> float:
        """When the arm source last gave a complete sample."""
        if self.arm_source == "hands":
            return min(*self.hand_receipts, self.head_receipt)
        return self.receipts["leader"]

    def input_receipts(self) -> dict[str, float]:
        """When each input the profile needs was last complete, by its name."""
        arms = (
            {"hands": min(self.hand_receipts), "head": self.head_receipt}
            if self.arm_source == "hands"
            else {"leader": self.receipts["leader"]}
        )
        return {**arms, "robot": self.receipts["robot"], "joycon": self.receipts["joycon"]}

    def set_leader(self, positions: dict[str, float], now: float) -> None:
        if set(positions) != set(self.arm_names) or not all(map(math.isfinite, positions.values())):
            self.pause("Incomplete leader sample")
            self.receipts["leader"] = -math.inf
            return
        age = now - self.receipts["leader"]
        period = self.config.leader_angle_period
        if period is not None:
            resolved = {}
            for name, value in positions.items():
                previous = self.leader.get(name, value)
                value = previous + math.remainder(value - previous, period)
                lo, hi = self.limits[name]
                # Keep continuity when possible, but never clamp a remembered
                # full turn if the same physical angle fits the robot's range.
                # Otherwise returning a leader to neutral can pin its follower
                # at a limit, including after pausing and re-enabling.
                first_turn = math.ceil((lo - value) / period)
                last_turn = math.floor((hi - value) / period)
                if first_turn <= last_turn:
                    value += min(last_turn, max(first_turn, 0)) * period
                resolved[name] = value
            positions = resolved
        if self.active and age > self.config.input_timeout_s:
            # A new sample can arrive before the watchdog tick notices the gap.
            self._hold_for_input(now, "leader")
        if self.leader and (self.active or self.recovery_deadline is not None):
            deltas = {name: value - self.leader[name] for name, value in positions.items()}
            if period is not None:
                deltas = {name: math.remainder(delta, period) for name, delta in deltas.items()}
            joint = max(deltas, key=lambda name: abs(deltas[name]))
            delta = deltas[joint]
            if abs(delta) > self.config.max_leader_jump:
                self.pause(
                    f"Leader discontinuity at {joint}: {delta:+.3f} rad; "
                    f"release {self.enable_button} and re-enable"
                )
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
        hands = self.arm_source == "hands"
        stale = [
            name
            for name, receipt in self.input_receipts().items()
            if now - receipt > self.config.input_timeout_s
        ]
        left, right = self.readings
        present = all(
            self.readings[SIDES.index(side)] is not None for side in self.config.enabled_sides
        )
        healthy = not stale and present
        authority_reading = self.readings[self.authority_index]
        # An explicit second enable press stops even if hand/leader tracking is temporarily
        # missing. Otherwise recovery could silently resume after the operator toggled off.
        disable_edge = (
            authority_reading is not None
            and self.enable_button in authority_reading.pressed
            and self.enable_released
            and self.enable_button not in self.previous_buttons[self.authority_index]
            and (self.active or self.recovery_deadline is not None)
        )
        if (
            "joycon" not in stale
            and authority_reading is not None
            and (self.pause_button in authority_reading.pressed or disable_edge)
        ):
            self.pause(f"Paused; release {self.enable_button} before enabling")
        if self.recovery_deadline is not None and now >= self.recovery_deadline:
            self.pause("Input interruption exceeded recovery window; re-enable tracking")
        if not healthy:
            if not present:
                missing = [*stale, "/".join(self.config.enabled_sides) + " Joy-Con"]
                self.pause("Waiting for fresh " + ", ".join(missing))
            else:
                self._hold_for_input(now, ", ".join(stale))
            if "robot" in stale:
                return None
        if self.capture_hold:
            self._capture()
        if healthy:
            left, right = left or JoyConReading(), right or JoyConReading()
            buttons = (left.pressed, right.pressed)
            if self.recovery_deadline is not None:
                self.recovery_deadline = None
                self.active = True
                self.reason = self._following()
                # Buttons held across a gap are not new toggle presses.
                self.previous_buttons = buttons
                for i, label in enumerate(("ZL", "ZR")):
                    self.grippers[i].trigger_was_down = label in buttons[i]
            authority = buttons[self.authority_index]
            x = self.enable_button in authority
            enable_edge = (
                x
                and self.enable_released
                and self.enable_button not in self.previous_buttons[self.authority_index]
            )
            if self.pause_button in authority:
                self.pause(f"Paused; release {self.enable_button} before enabling")
                if self.capture_hold:
                    self._capture()
            elif enable_edge and not self.active:
                self._capture()
                self.arm_locked = [False, False]
                self.locked.clear()
                self.held_hands = [None, None]
                # Where the operator stands and faces now is what the hands are measured from.
                self.anchor = self.head
                self.active = True
                self.reason = self._following()
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

            speed = self.config.base_speed
            # Match the current R1 Pro reference: right click suppresses yaw and tilt.
            yaw = 0.0 if right.stick_pressed else -axis(right.stick[0]) * self.config.yaw_speed
            velocity = (-axis(left.stick[1]) * speed, -axis(left.stick[0]) * speed, yaw)
        return JoyloOutput(
            self.target.copy(),
            (self.grippers[0].opening, self.grippers[1].opening),
            velocity,
            self._hands_output() if hands else None,
        )

    def _following(self) -> str:
        if self.arm_source == "hands":
            return "Following the headset's hands"
        return "Following calibrated OpenArm joint positions"

    def _hands_output(self) -> HandsOutput:
        if not self.active or None in self.hands or self.anchor is None:
            return HandsOutput(False, None)
        targets = []
        for held, hand, in_base in zip(
            self.held_hands, self.hands, self.hand_targets_in_base, strict=True
        ):
            pose = held if held is not None else hand
            assert pose is not None
            targets.append(pose if in_base else self.gripper_target(pose))
        return HandsOutput(True, (targets[0], targets[1]))

    @cached_property
    def _torso_joints(self) -> tuple[JointDescription, ...]:
        joints = {joint.name: joint for joint in self.config.model.load().joints}
        return tuple(joints[name] for name in TORSO)

    @cached_property
    def _neutral_torso_inverse(self) -> NDArray[np.float64]:
        pose = np.eye(4)
        for joint in self._torso_joints:
            pose = pose @ joint.parent_from_child(0.0)
        return np.asarray(np.linalg.inv(pose), dtype=np.float64)

    def _torso_motion(self) -> NDArray[np.float64]:
        """Carry a neutral-body target with the measured upper body, including tilt/yaw."""
        pose = np.eye(4)
        for joint in self._torso_joints:
            pose = pose @ joint.parent_from_child(self.feedback[joint.name])
        result: NDArray[np.float64] = pose @ self._neutral_torso_inverse
        return result

    def gripper_target(self, hand: HandPose) -> HandPose:
        """Where a gripper goes for a hand, in BASE_FRAME.

        The gripper is put as far from the robot's eyes as the hand is from where
        the operator's eyes were at the enable press, in the directions the
        operator faced then, and is turned as the hand is. This reference rides
        with the measured torso, so lowering or tilting does not lift the arms
        to compensate in the chassis frame.
        """
        assert self.anchor is not None
        (origin, heading), (position, orientation) = self.anchor, hand
        cos, sin = math.cos(heading), math.sin(heading)
        dx, dy, dz = (value - start for value, start in zip(position, origin, strict=True))
        facing = (0.0, 0.0, -math.sin(heading / 2), math.cos(heading / 2))
        neutral = np.array(
            [
                ROBOT_EYES[0] + cos * dx + sin * dy,
                ROBOT_EYES[1] - sin * dx + cos * dy,
                ROBOT_EYES[2] + dz,
            ]
        )
        motion = self._torso_motion()
        carried = motion[:3, :3] @ neutral + motion[:3, 3]
        turn = Quaternion.from_rotation_matrix(motion[:3, :3])
        orientation = _turned(_turned(facing, orientation), _WRIST_TO_GRIPPER)
        return (
            (float(carried[0]), float(carried[1]), float(carried[2])),
            _turned((turn.x, turn.y, turn.z, turn.w), orientation),
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
                step_gripper(self.grippers[i], down, mode="toggle", slew_per_s=None, dt=dt)
            else:
                self.grippers[i].trigger_was_down = down
        if self.arm_source == "hands":
            self._follow_hands(next_locked)
        else:
            for name in self.arm_names:
                if name in next_locked:
                    if name not in self.locked:
                        self.target[name] = self.feedback[name]
                    continue
                lo, hi = self.limits[name]
                desired = min(hi, max(lo, self.leader[name]))
                self.target[name] = desired
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

    def _follow_hands(self, next_locked: set[str]) -> None:
        """The locks, as poses: the headset arm task follows what it is given.

        A locked arm is given the pose its hand had when it was locked, and a held
        wrist the orientation its hand had, so the arm stays or keeps its
        orientation. On release the task moves to where the hand is now, at its
        own rate, as an unlocked arm goes to its leader. An enabled arm does the
        same from where it stands: it goes to its hand, then stays with it.
        """
        for i, side in enumerate(SIDES):
            hand = self.hands[i]
            if hand is None:
                continue
            arm = {f"r1pro/{side}_arm_joint{j}" for j in range(1, 8)}
            wrist = {f"r1pro/{side}_arm_joint{j}" for j in (5, 6, 7)}
            held = self.held_hands[i]
            if arm <= next_locked:
                self.held_hands[i] = held if held is not None and arm <= self.locked else hand
            elif wrist <= next_locked:
                orientation = held[1] if held is not None and wrist <= self.locked else hand[1]
                self.held_hands[i] = (hand[0], orientation)
            else:
                self.held_hands[i] = None
        self.locked = next_locked
