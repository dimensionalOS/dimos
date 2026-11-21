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

"""Bounded single-trial excitation plans and a passive Coordinator task."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping
from dataclasses import dataclass
import json
import math
from threading import Lock
import time
from typing import TYPE_CHECKING, Any

from dimos.control.task import BaseControlTask, CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.std_msgs.String import String
from dimos.robot.galaxea.r1pro.config import R1PRO_BODY_AXES, R1PRO_PLANAR_BASE
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS, coordinator_name

if TYPE_CHECKING:
    from dimos.control.connection_source import ConnectionSource
    from dimos.control.contract.limits import Limits
    from dimos.control.coordinator import TaskConfig

HZ = 50
DT = 1 / HZ
UPPER = tuple(map(coordinator_name, UPPER_BODY_JOINTS))
BASE = R1PRO_BODY_AXES
SEGMENTS = {"/hdas/feedback_torso": 4, "/hdas/feedback_arm_left": 7, "/hdas/feedback_arm_right": 7}
CHASSIS = "/motion_control/chassis_speed"
JOINTS = {"torso-j2": 1, "shoulder": 4, "elbow": 7, "wrist": 9}
MAX_FEEDBACK_AGE = 0.15
MAX_TICK_GAP = 0.065
SETTLE_S = 0.5


@dataclass(frozen=True)
class Trial:
    kind: str
    duration: float
    axis: int = 0
    amplitude: float = 0.0
    pose: tuple[float, ...] = ()

    def velocity(self, t: float) -> tuple[float, float, float]:
        if self.kind == "ramp":
            return (0.01 + 0.09 * t / 2 if 0 <= t <= 2 else 0.0, 0.0, 0.0)
        if self.kind == "coupled":
            return (0.5, 0.0, 0.8) if 0 <= t < 1.5 else (0.0, 0.0, 0.0)
        speed = self.amplitude if self.kind == "base" and 0 <= t < 1.5 else 0.0
        return (
            speed if self.axis == 0 else 0.0,
            speed if self.axis == 1 else 0.0,
            speed if self.axis == 2 else 0.0,
        )

    def position(self, t: float, anchor: tuple[float, ...]) -> tuple[float, ...]:
        if len(anchor) != len(UPPER):
            raise ValueError("expected an 18-axis anchor")
        if self.kind == "pose":
            # Slow linear approach (1.5 s) followed by 3 s at the exact pose.
            u = min(1.0, max(0.0, t / 1.5))
            return tuple(a + u * (b - a) for a, b in zip(anchor, self.pose, strict=True))
        offset = 0.0
        if self.kind == "step" and 0 <= t < 1.5:
            offset = self.amplitude
        elif self.kind == "chirp" and 0 <= t < 6.0:
            frequencies = (0.5, 1.0, 2.0, 3.0)
            section = min(int(t / 2), 2)
            u = t - 2 * section
            cycles = sum(frequencies[i] + frequencies[i + 1] for i in range(section))
            start, end = frequencies[section : section + 2]
            cycles += start * u + (end - start) * u**2 / 4
            envelope = min(1.0, t / 0.35, (6.0 - t) / 0.35)
            amplitude = 0.15 - 0.05 * min(1.0, t / 4)
            offset = amplitude * envelope * math.sin(math.tau * cycles)
        return tuple(q + offset if i == self.axis else q for i, q in enumerate(anchor))


def _pose(joints: dict[int, float]) -> tuple[float, ...]:
    return tuple(joints.get(i, 0.0) for i in range(len(UPPER)))


TRIALS = {
    **{
        f"{axis}:{sign * magnitude:g}": Trial("base", 2.0, i, sign * magnitude)
        for i, (axis, magnitudes) in enumerate(
            (("vx", (0.2, 0.5, 0.8, 1.2)), ("vy", (0.2, 0.5, 0.8)), ("wz", (0.3, 0.8, 1.5)))
        )
        for magnitude in magnitudes
        for sign in (1, -1)
    },
    "deadband": Trial("ramp", 2.5),
    "coupled": Trial("coupled", 2.0),
    **{
        f"{joint}:step:{amplitude:g}": Trial("step", 2.0, index, amplitude)
        for joint, index in JOINTS.items()
        for amplitude in (0.10, -0.10, 0.25, -0.25)
    },
    **{f"{joint}:chirp": Trial("chirp", 6.0, index) for joint, index in JOINTS.items()},
    "zero": Trial("pose", 4.5, pose=_pose({})),
    "arms-forward": Trial("pose", 4.5, pose=_pose({4: -0.95, 11: -0.95})),
    "torso-bent": Trial("pose", 4.5, pose=_pose({1: 0.20})),
}


def chassis_allowed(target: tuple[float, float, float]) -> bool:
    vx, vy, wz = target
    return (
        all(
            math.isfinite(v) and abs(v) <= bound
            for v, bound in zip(target, R1PRO_PLANAR_BASE.velocity_limits, strict=True)
        )
        and math.hypot(vx, vy) <= 1.0
        and abs(wz) <= 1.5
    )


class R1ProSysIdTask(BaseControlTask):
    """One explicitly requested trial. Idle/preview never admit a write."""

    def __init__(self, name: str, priority: int, limits: Mapping[str, Limits]) -> None:
        self._name = name
        self._priority = priority
        self._limits = limits
        self._feedback: dict[str, tuple[float, tuple[float, ...]]] = {}
        self._faults: set[str] = set()
        self._gate = Lock()
        self._write_enabled = False
        self._estopped = False
        self._trial: Trial | None = None
        self._case = ""
        self._phase = "idle"
        self._anchor: tuple[float, ...] | None = None
        self._last_good: tuple[float, ...] | None = None
        self._hold: tuple[float, ...] | None = None
        self._started = self._last_tick = self._settle_until = self._requested_at = 0.0
        self._reason = ""

    def on_diagnostics(self, msg: String) -> bool:
        # Called under the Coordinator task lock, like compute and task RPCs.
        topic: str | None = None
        try:
            event = json.loads(msg.data)
            if event["schema"] != 1 or event["direction"] != "rx":
                return False
            topic = event["topic"]
            if topic in (
                "/hdas/feedback_status_torso",
                "/hdas/feedback_status_arm_left",
                "/hdas/feedback_status_arm_right",
            ):
                if any(code or text for _, code, text in event["values"]["errors"]):
                    self._faults.add(topic)
                    self.abort("vendor upper-body fault")
                else:
                    self._faults.discard(topic)
                return True
            if topic not in SEGMENTS and topic != CHASSIS:
                return False
            rx = event["monotonic_ns"] / 1e9
            age = (event["unix_ns"] - event["header_ns"]) / 1e9
            source_at = rx - max(0.0, age)
            if not -0.01 <= age <= MAX_FEEDBACK_AGE:
                raise ValueError("stale or future header")
            fields = event["values"]
            values = (
                (fields["linear"][0], fields["linear"][1], fields["angular"][2])
                if topic == CHASSIS
                else tuple(fields["position"])
            )
            if len(values) != SEGMENTS.get(topic, 3) or not all(map(math.isfinite, values)):
                raise ValueError("incomplete or non-finite feedback")
            previous = self._feedback.get(topic)
            if previous is not None and source_at <= previous[0]:
                return False  # repeated/reordered source timestamp never refreshes it
            self._feedback[topic] = (source_at, values)
            return True
        except (KeyError, IndexError, TypeError, ValueError, AttributeError):
            if isinstance(topic, str):
                self._feedback.pop(topic, None)
                if self._phase in ("running", "pending"):
                    self.abort("malformed or stale ROS feedback")
            return False

    def _fresh(self, topic: str, now: float) -> tuple[float, ...] | None:
        data = self._feedback.get(topic)
        return data[1] if data is not None and -0.01 <= now - data[0] <= MAX_FEEDBACK_AGE else None

    def _bounded_position(self, q: tuple[float, ...], margin: float = 0.0) -> bool:
        if len(q) != len(UPPER):
            return False
        for joint, value in zip(UPPER, q, strict=True):
            limit = self._limits[joint]
            if (
                limit.lo is None
                or limit.hi is None
                or not limit.lo + margin <= value <= limit.hi - margin
            ):
                return False
        return True

    def _measured(self, now: float) -> tuple[float, ...] | None:
        segments = [self._fresh(topic, now) for topic in SEGMENTS]
        if any(q is None for q in segments):
            return None
        q = tuple(x for part in segments if part is not None for x in part)
        return q if self._bounded_position(q) else None

    def run_trial(self, case: str, live: bool = False) -> dict[str, Any]:
        trial = TRIALS[case]
        caps = all(chassis_allowed(trial.velocity(i / HZ)) for i in range(int(trial.duration * HZ)))
        if live is not True:
            return {
                "case": case,
                "duration_s": trial.duration,
                "rate_hz": HZ,
                "chassis_within_limits": caps,
                "dry_run": True,
            }
        if self._estopped or self.is_active() or self._faults:
            raise ValueError("E-stop, active trial or vendor fault")
        if not caps:
            raise ValueError("trial exceeds the deployed chassis envelope; preview only")
        self._trial, self._case = trial, case
        self._anchor = self._last_good = self._hold = None
        self._phase, self._reason = "pending", ""
        self._requested_at = time.perf_counter()
        return self.status()

    def status(self) -> dict[str, Any]:
        return {"case": self._case, "phase": self._phase, "reason": self._reason}

    def abort(self, reason: str = "operator abort") -> bool:
        if self._phase == "settling" and self._reason:
            return False
        with self._gate:
            self._write_enabled = False  # fence any moving frame not yet submitted
        if self._phase in ("pending", "running", "settling"):
            self._phase, self._reason = "stopping", reason
            return True
        return False

    def set_estop(self, estopped: bool) -> None:
        self._estopped = estopped
        if estopped:
            self._enable_write(False)
            self._phase, self._reason = "aborted", "coordinator E-stop"

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        return self.abort("runtime reset")

    def admit_write(self, publish: Callable[[], None]) -> None:
        with self._gate:
            if self._write_enabled:
                publish()

    def _enable_write(self, enabled: bool) -> None:
        with self._gate:
            self._write_enabled = enabled

    def is_active(self) -> bool:
        return self._phase in ("pending", "running", "stopping", "settling")

    def claim(self) -> ResourceClaim:
        # The Coordinator's arbiter uses each output's mode and joint names.
        moving_base = (
            self._phase in ("running", "pending")
            and self._trial is not None
            and self._trial.kind in ("base", "coupled", "ramp")
        )
        return ResourceClaim(
            frozenset(BASE if moving_base else UPPER),
            self._priority,
            ControlMode.VELOCITY if moving_base else ControlMode.POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        self.abort(f"preempted by {by_task}")

    def _position(self, values: tuple[float, ...]) -> JointCommandOutput:
        return JointCommandOutput(list(UPPER), positions=list(values))

    def _settle(self, now: float, measured: tuple[float, ...] | None) -> JointCommandOutput | None:
        self._hold = measured if measured is not None else self._last_good
        if self._hold is None:
            self._phase = "aborted"
            self._enable_write(False)
            return None  # Never synthesize a hold or re-enable motion without feedback.
        self._settle_until = now + SETTLE_S
        self._phase = "settling"
        self._enable_write(True)
        # No chassis winner: the dedicated Coordinator fills native axes with zero.
        return self._position(self._hold)

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self.is_active():
            return None
        now = state.t_now
        measured = self._measured(now)
        base = self._fresh(CHASSIS, now)
        assert self._trial is not None
        trial = self._trial
        chassis = trial.kind in ("base", "coupled", "ramp")
        if self._phase == "stopping":
            return self._settle(now, measured if self._anchor is not None else None)
        if self._phase == "settling":
            if now >= self._settle_until or self._hold is None:
                self._enable_write(False)
                self._phase = "aborted" if self._reason else "complete"
                return None
            return self._position(self._hold)
        if measured is None or base is None or self._faults:
            self.abort("missing/stale ROS feedback or vendor fault")
            return self._settle(now, measured if self._anchor is not None else None)
        stationary = math.hypot(*base[:2]) <= 0.05 and abs(base[2]) <= 0.10
        priming = self._phase == "pending"
        if priming:
            if now - self._requested_at > MAX_FEEDBACK_AGE or not stationary:
                self.abort("expired start request or chassis not stationary")
                return self._settle(now, None)
            self._anchor = measured
            self._started = now + DT if chassis else now
            self._last_tick = now
            self._phase = "running"
            for i in range(int(trial.duration * HZ) + 1):
                target = trial.position(i / HZ, measured)
                delta = max(abs(a - b) for a, b in zip(target, measured, strict=True))
                envelope = 0.35 if trial.kind == "pose" else 0.25
                if delta > envelope + 1e-9 or not self._bounded_position(target, 0.03):
                    self._anchor = None
                    self.abort("trial outside position/entry envelope")
                    return self._settle(now, None)
        assert self._anchor is not None
        elapsed = now - self._started
        last = self._last_tick - self._started
        self._last_tick = now
        self._last_good = measured
        previous_target = trial.position(last, self._anchor)
        if (
            elapsed - last > MAX_TICK_GAP
            or not chassis_allowed(base)
            or (not chassis and not stationary)
            or max(abs(a - b) for a, b in zip(previous_target, measured, strict=True)) > 0.40
        ):
            self.abort("tick overrun, unexpected chassis motion or tracking error")
            return self._settle(now, measured)
        if elapsed >= trial.duration:
            return self._settle(now, measured)
        self._enable_write(True)
        if chassis and priming:
            return self._position(self._anchor)  # seed a fresh arm hold before moving the chassis
        if chassis:
            return JointCommandOutput(
                list(BASE), velocities=list(trial.velocity(elapsed)), mode=ControlMode.VELOCITY
            )
        return self._position(trial.position(elapsed, self._anchor))


def create_task(
    cfg: TaskConfig, hardware: Any, connections: Mapping[str, ConnectionSource]
) -> R1ProSysIdTask:
    source = connections["r1pro"]
    return R1ProSysIdTask(
        cfg.name, cfg.priority, {joint: source.position_limit(joint) for joint in UPPER}
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Offline, synthetic zero-anchor R1 Pro SysID waveform preview"
    )
    parser.add_argument("--dry-run", action="store_true", required=True)
    parser.add_argument("--case", choices=TRIALS, required=True)
    args = parser.parse_args()
    trial = TRIALS[args.case]
    for i in range(int(trial.duration * HZ) + 1):
        t = i / HZ
        velocity = trial.velocity(t)
        print(
            json.dumps(
                {
                    "case": args.case,
                    "t_s": t,
                    "body_velocity": dict(zip(BASE, velocity, strict=True)),
                    "upper_position": dict(
                        zip(UPPER, trial.position(t, (0.0,) * len(UPPER)), strict=True)
                    ),
                    "chassis_within_limits": chassis_allowed(velocity),
                }
            )
        )


if __name__ == "__main__":
    main()
