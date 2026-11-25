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

"""Stock OpenYAM I/O with an independent, connection-owned reference tracker."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import replace
from functools import partial
from pathlib import Path
import time
from typing import Any, Protocol
import uuid

import numpy as np
from reactivex.disposable import Disposable

from dimos.control.connection.connection_module import ConnectionConfig, Values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.damiao.arm import ArmReading, DamiaoArm
from dimos.robot.manipulators.openyam.connection import OpenYamConnection, OpenYamConnectionConfig
from dimos.robot.manipulators.openyam.joints import OPENYAM_ARM_JOINTS
from dimos.robot.manipulators.openyam.trimode_tracker import (
    REFERENCE_TIMEOUT,
    Actor,
    ArmSample,
    Bounds,
    Goal,
    JointGoal,
    PoseGoal,
    PositionGoal,
    Tracker,
    Vector,
    load_actor,
    measured_bounded,
)
from dimos.spec.utils import Spec

PORTS = {"tcp_pose": PoseGoal, "tcp_position": PositionGoal, "joint_reference": JointGoal}
JAW = "joint7"
# Frozen 13.5k browser plant (policy.js): q_des, zero dq_des, no gravity feedforward.
_KP = (50, 60, 50, 25, 15, 10)
_KD = (3, 4, 3, 2, 1, 0.8)


def declaration(model: RobotModel | None) -> ConnectionConfig:
    return ConnectionConfig(
        source="arm",
        model=model,
        joint_commands={JAW: (Interface.POSITION,)},
        group_commands=dict.fromkeys(PORTS, frozenset(OPENYAM_ARM_JOINTS)),
    )


def arm_bounds(model: RobotModel) -> Bounds:
    joints = {j.name: j for j in movable_joints(model)}
    bounds = []
    for name in OPENYAM_ARM_JOINTS:
        joint = joints[name]
        if joint.lower is None or joint.upper is None:
            raise ValueError("tri-mode requires six position bounds")
        bounds.append((max(joint.lower, -12.5), min(joint.upper, 12.5)))
    return tuple(bounds)


class TrimodeLifecycle(Spec, Protocol):
    def open_reference(self, port: str, until: float) -> str | None: ...
    def close_reference(self, session: str, idle_hold: bool = False) -> None: ...
    def stop_references(self, stopped: bool) -> None: ...


class TrimodeConnectionConfig(OpenYamConnectionConfig):
    model_path: Path


class TrimodeArm(DamiaoArm):
    """Keep stock connect, enable/calibration, read and disable; split native writes."""

    def _send(self) -> None:
        # Stock commands still update the calibrated jaw cache and resend
        # it via the same SDK call. It can never resend cached arm targets.
        native = self._native
        if native is None or not self.active:
            raise RuntimeError("Damiao robot is inactive")
        (jaw,) = self.robot.grippers
        closed, opened = self._travel[jaw.joint]
        opening = (self._targets[jaw.joint] - closed) / (opened - closed)
        if not 0 <= opening <= 1:
            raise ValueError("gripper target is outside its URDF travel")
        self._grippers[jaw.name].set_opening(opening)
        native.tick(1_000)

    def hold_jaw(self, position: float) -> None:
        self.command({JAW: position})

    def hold_arm(self, q: Vector) -> None:
        # Preserve the stock measured-hold law, without touching the jaw cache.
        (bus,) = self.robot.buses
        assert self._physical is not None
        gravity = self._physical.gravity(np.asarray(q, dtype=np.float64))
        self._mit(bus.kp, bus.kd, q, tuple(gravity.tolist()))

    def submit_arm(self, q: Vector) -> None:
        self._mit(_KP, _KD, q, (0.0,) * 6)

    def _mit(self, kp: Sequence[float], kd: Sequence[float], q: Vector, tau: Vector) -> None:
        if not self.active:
            raise RuntimeError("Damiao robot is inactive")
        self._arms["arm"].mit_control(np.column_stack((kp, kd, q, (0.0,) * 6, tau)))


class TrimodeConnection(OpenYamConnection):
    config: TrimodeConnectionConfig
    tcp_pose: In[PoseGoal]
    tcp_position: In[PositionGoal]
    joint_reference: In[JointGoal]
    arm_sample: Out[ArmSample]

    def __init__(self, **kwargs: Any) -> None:
        for key, value in dict(declaration(self.robot.model)).items():
            if key not in {"source", "model", "joint_commands", "group_commands"}:
                continue
            kwargs.setdefault(key, value)
        super().__init__(**kwargs)
        self._tracker: Tracker | None = None
        self._sample: ArmSample | None = None
        self._token: str | None = None
        self._port: str | None = None
        self._last_issued = 0.0
        self._deadline = 0.0
        self._write_until = 0.0
        self._last_write: float | None = None
        self._last_goal: Goal | None = None
        self._idle_goal: Goal | None = None
        self._terminal_at: float | None = None
        self._reference_fault: str | None = None
        self._inhibited = False

    def _make_arm(self) -> TrimodeArm:
        return TrimodeArm(self.robot, self.can_ports())

    def write(self, values: Values) -> None:
        if not self._inhibited and not self._stopping:
            super().write(values)

    def halt(self) -> None:
        # The inherited deadman, write-failure and stop paths call this with
        # the device lock held. Revoke before I/O so in-flight inference cannot
        # publish after a halt, even if that halt cannot be sent.
        self._revoke_locked("stock connection halted")
        self._hold_locked(hold_jaw=True)

    def _actor(self) -> Actor:
        return load_actor(self.config.model_path.expanduser())

    def connect(self) -> None:
        model = self.config.model
        if model is None or {j.name for j in movable_joints(model)} != set(self.joint_names):
            raise ValueError("connection model joints must match vendor topology")
        if tuple(self.robot.arm_joint_names) != tuple(OPENYAM_ARM_JOINTS):
            raise ValueError("OpenYAM arm order differs from the actor ABI")
        self._state_frame = self.frame(model.load().root_link)
        self._arm = arm = self._make_arm()
        arm.connect()
        self._read_state()

    @rpc
    def start(self) -> None:
        expected = declaration(self.config.model)
        if (
            self.config.g.simulation
            or self.config.g.replay
            or self.config.teach
            or self.config.source != "arm"
            or self.config.joint_commands != expected.joint_commands
            or self.config.group_commands != expected.group_commands
            or self.config.model is None
        ):
            raise ValueError("tri-mode requires one known arm and the stock jaw route")
        if self._tracker is not None or self._stopping:
            raise RuntimeError("construct a fresh tri-mode connection to restart")
        self._bounds = arm_bounds(self.config.model)
        self._tracker = Tracker(
            self._actor(), self._bounds, self._prepare, self._submit_policy, self._failed
        )
        try:
            super().start()
            for port in PORTS:
                self.register_disposable(
                    Disposable(self.inputs[port].subscribe(partial(self._offer, port)))
                )
        except Exception:
            self.stop()
            raise

    @rpc
    def stop(self) -> None:
        try:
            self.stop_references(True)
        finally:
            try:
                super().stop()
            finally:
                tracker, self._tracker = self._tracker, None
                if tracker is not None:
                    tracker.close()

    @rpc
    def open_reference(self, port: str, until: float) -> str | None:
        # An RPC delayed across cancellation/startup is not permission to move.
        with self._hardware_lock:
            now = time.monotonic()
            if (
                port not in PORTS
                or not now < until <= now + REFERENCE_TIMEOUT
                or self._inhibited
                or self._stopping
                or not self._connected
                or self._fault is not None
                or not self._sample_live(now)
            ):
                return None
            self._expire_locked(now)
            if self._idle_goal is not None:
                # Fence pending idle work, but keep its write watchdog armed
                # until the new reference actually submits or times out.
                self._idle_goal = None
            else:
                self._end_locked()
            if self._fault is not None:
                return None
            self._token, self._port = uuid.uuid4().hex, port
            self._last_issued, self._deadline = now, until
            return self._token

    @rpc
    def close_reference(self, session: str, idle_hold: bool = False) -> None:
        with self._hardware_lock:
            if session != self._token or self._token is None:
                return
            if not idle_hold:
                self._end_locked()
                return
            now = time.monotonic()
            self._expire_locked(now)
            if self._token is None:
                return
            goal, tracker, sample = self._last_goal, self._tracker, self._sample
            if goal is None or tracker is None or sample is None or not self._policy_live(now):
                self._end_locked()
                return
            last_write = self._last_write
            assert last_write is not None
            self._revoke_locked(None)
            self._sample = replace(sample, session=None, terminal_at=None, fault=None)
            self._last_write = last_write
            self._write_until = last_write + self.config.deadman_timeout_s
            # A distinct object revokes every external goal, but reusing its
            # session inside Tracker preserves history and the previous action.
            self._idle_goal = (
                replace(goal, terminal=False) if isinstance(goal, JointGoal) else replace(goal)
            )
            tracker.offer(self._idle_goal)

    @rpc
    def stop_references(self, stopped: bool) -> None:
        with self._hardware_lock:
            self._inhibited = stopped
            if stopped:
                self._end_locked()

    def _sample_live(self, now: float) -> bool:
        return self._sample is not None and 0 <= now - self._sample.at < REFERENCE_TIMEOUT

    def _policy_live(self, now: float) -> bool:
        return (
            self._connected
            and not self._inhibited
            and not self._stopping
            and self._fault is None
            and isinstance(self._arm, TrimodeArm)
            and self._arm.active
            and self._sample_live(now)
        )

    def _offer(self, port: str, goal: Goal) -> None:
        with self._hardware_lock:
            now = time.monotonic()
            self._expire_locked(now)
            if (
                type(goal) is not PORTS[port]
                or goal.session != self._token
                or port != self._port
                or not self._last_issued < goal.issued_at <= now < goal.deadline
                or not goal.deadline <= goal.issued_at + REFERENCE_TIMEOUT
            ):
                return
            self._last_issued, self._deadline = goal.issued_at, goal.deadline
            if self._tracker is not None:
                self._tracker.offer(goal)

    def _prepare(self, goal: Goal) -> ArmSample | None:
        with self._hardware_lock:
            now = time.monotonic()
            self._expire_locked(now)
            if goal is self._idle_goal or (
                goal.session == self._token and now < goal.deadline and self._sample_live(now)
            ):
                return self._sample
            return None

    def _submit_policy(self, goal: Goal, sample: ArmSample, q_des: Vector) -> bool:
        with self._hardware_lock:
            idle = goal is self._idle_goal
            now = time.monotonic()
            self._expire_locked(now)
            arm = self._arm
            if idle:
                if goal is not self._idle_goal:
                    return False
                if not 0 <= now - sample.at < REFERENCE_TIMEOUT:
                    self._end_locked("idle policy feedback expired")
                    return False
            elif (
                goal.session != self._token
                or self._inhibited
                or not self._connected
                or self._stopping
                or self._fault is not None
                or not isinstance(arm, TrimodeArm)
                or now >= min(goal.deadline, sample.at + REFERENCE_TIMEOUT)
            ):
                return False
            assert isinstance(arm, TrimodeArm)
            # Native SDK call performs synchronous host submission, not execution.
            # A failed batch may have partially reached the device. Arm the
            # safety hold before the call, not only on a successful return.
            self._last_write = now
            arm.submit_arm(q_des)
            self._write_until = now + self.config.deadman_timeout_s
            if not idle:
                self._write_until = min(goal.deadline, self._write_until)
                self._last_goal = goal
                if isinstance(goal, JointGoal) and goal.terminal and self._terminal_at is None:
                    self._terminal_at = time.monotonic()
            return True

    def _failed(self, goal: Goal) -> None:
        with self._hardware_lock:
            if goal is self._idle_goal or goal.session == self._token:
                self._end_locked("actor or arm submission failed")

    def _expire_locked(self, now: float) -> None:
        if self._token is not None and (
            now >= self._deadline or (self._last_write is not None and now >= self._write_until)
        ):
            self._end_locked("arm reference expired")
        elif self._idle_goal is not None and (
            not self._policy_live(now) or now >= self._write_until
        ):
            self._end_locked("idle policy hold expired")

    def _end_locked(self, fault: str | None = None) -> None:
        had_write = self._last_write is not None
        self._revoke_locked(fault)
        if had_write:
            self._hold_locked(hold_jaw=False)

    def _revoke_locked(self, fault: str | None) -> None:
        self._token = self._port = None
        self._reference_fault = fault
        self._last_write = None
        self._terminal_at = None
        self._last_goal = self._idle_goal = None

    def _feedback_failed(self, error: Exception) -> None:
        self._revoke_locked("arm feedback failed")
        super()._feedback_failed(error)

    def _hold_locked(self, *, hold_jaw: bool) -> None:
        arm = self._arm
        if self._fault is not None or not isinstance(arm, TrimodeArm) or not arm.active:
            return
        # Explicit stock-style measured hold, never a cached policy action.
        # On bad feedback/send failure fail closed via the stock disable path.
        try:
            reading = arm.read()
            q = tuple(reading.position[:6])
            if not measured_bounded(q, self._bounds):
                raise ValueError("cannot safely hold invalid arm position")
            # The measured-state tolerance is shared with the tracker; hold commands stay in bounds.
            arm.hold_arm(
                tuple(
                    min(max(value, lo), hi) for value, (lo, hi) in zip(q, self._bounds, strict=True)
                )
            )
            if hold_jaw:
                arm.hold_jaw(reading.position[6])
        except Exception as error:
            self._feedback_failed(error)

    def _read_state(self) -> ArmReading:
        # Stock read loop calls this with the same lock as device writes. Check
        # timeout here even if the inference thread is stalled and jaw is busy.
        self._expire_locked(time.monotonic())
        at = time.monotonic()
        reading = super()._read_state()
        self._sample = ArmSample(
            at,
            tuple(reading.position[:6]),
            tuple(reading.velocity[:6]),
            self._token,
            self._terminal_at,
            self._reference_fault,
        )
        if self._idle_goal is not None and self._tracker is not None:
            self._tracker.offer(self._idle_goal)
        return reading

    def _publish_reading(self, reading: ArmReading) -> None:
        super()._publish_reading(reading)
        if self._sample is not None:
            self.arm_sample.publish(self._sample)
