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

"""Operator hold: freeze every joint where it is and wait for a person.

A hold is not an e-stop. An e-stop cuts motion; a hold keeps commanding
each joint's current position (zero velocity for a wheeled base) from a
priority above every other task, so nothing moves until someone
acknowledges the hold.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import threading
import time
from typing import TYPE_CHECKING, Any

from pydantic import Field

from dimos.control.components import TWIST_SUFFIX_MAP
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.control.coordinator import TaskConfig

logger = setup_logger()

OPERATOR_HOLD_TASK_NAME = "operator_hold"
OPERATOR_HOLD_PRIORITY = 100


class HoldRoute(str, Enum):
    """Who asked for the hold."""

    MANUAL = "manual"  # a person, e.g. a cockpit button
    FAILSAFE = "failsafe"  # the robot itself, e.g. a module entering FAULT
    AGENT = "agent"  # the autonomy agent asking for help


@dataclass(frozen=True)
class OperatorHoldStatus:
    """What the hold is doing right now.

    Attributes:
        on: True while the hold is requested.
        route: The HoldRoute value that asked for the hold; "" when off.
        reason: Free text from the requester; "" when off.
        started_at: Unix time in seconds the hold was requested; 0.0 when off.
        stamp: Unix time in seconds this status was produced.
        unheld: Joints with no valid position reading, not held yet.
    """

    on: bool
    route: str = ""
    reason: str = ""
    started_at: float = 0.0
    stamp: float = 0.0
    unheld: tuple[str, ...] = ()


def operator_hold_task(priority: int = OPERATOR_HOLD_PRIORITY) -> TaskConfig:
    """Blueprint entry for the coordinator's one operator-hold task.

    Args:
        priority: Arbitration priority. Keep it above every other task
            (teleop is 50, autonomy 10-20) or the hold cannot take the joints.
    """
    # The coordinator imports task modules, so import it lazily here.
    from dimos.control.coordinator import TaskConfig

    return TaskConfig(name=OPERATOR_HOLD_TASK_NAME, type="operator_hold", priority=priority)


def _hold_value(joint_name: str, measured_position: float) -> float:
    # A wheeled base's "joints" are velocities (base/vx, base/vy, base/wz), so
    # freezing one means commanding zero, not repeating its odometry reading.
    suffix = joint_name.rsplit("/", 1)[-1]
    return 0.0 if suffix in TWIST_SUFFIX_MAP else measured_position


class OperatorHoldTask(BaseControlTask):
    """Hold every joint at the position it had when the hold was requested.

    Idle until ``request()``. Then, each tick, it learns the joints from the
    coordinator's joint-state snapshot: a joint seen for the first time is
    frozen at its measured position (zero for a base's velocity joints), and
    the whole set is commanded every tick until ``acknowledge()``. A joint
    that appears later, e.g. hardware added during a hold, is frozen when
    first seen. A joint with no valid reading waits in ``unheld``.

    Status goes to the publisher given to ``set_status_publisher`` on the
    first tick after a request, once per ``status_interval`` after that, and
    once more on acknowledge. ``get_status`` answers the same over task_invoke.
    """

    def __init__(
        self,
        name: str = OPERATOR_HOLD_TASK_NAME,
        priority: int = OPERATOR_HOLD_PRIORITY,
        status_interval: float = 1.0,
    ) -> None:
        """
        Args:
            name: Task name; callers address request/acknowledge to it.
            priority: Arbitration priority. Must exceed every other task's.
            status_interval: Seconds between repeated status messages while on.
        """
        if not math.isfinite(status_interval) or status_interval <= 0.0:
            raise ValueError(f"status_interval must be positive seconds, got {status_interval}")
        self._name = name
        self._priority = priority
        self._status_interval = status_interval
        self._publish: Callable[[OperatorHoldStatus], None] | None = None

        self._lock = threading.Lock()
        self._on = False
        self._route = ""
        self._reason = ""
        self._started_at = 0.0
        self._held: dict[str, float] = {}  # joint -> value commanded while on
        self._unheld: set[str] = set()  # joints seen without a finite reading
        self._last_status_t: float | None = None  # coordinator time of the last status

    def set_status_publisher(self, publish: Callable[[OperatorHoldStatus], None] | None) -> None:
        """Where to send OperatorHoldStatus messages; None sends them nowhere."""
        self._publish = publish

    def claim(self) -> ResourceClaim:
        """Every joint frozen so far; empty until the first tick after a request."""
        with self._lock:
            return ResourceClaim(
                joints=frozenset(self._held),
                priority=self._priority,
                mode=ControlMode.SERVO_POSITION,
            )

    def is_active(self) -> bool:
        return self._on

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Command the frozen value of every joint; freeze joints seen for the first time."""
        status: OperatorHoldStatus | None = None
        newly_unheld: list[str] = []
        with self._lock:
            if not self._on:
                return None
            for name, position in state.joints.joint_positions.items():
                if name in self._held:
                    continue
                if math.isfinite(position):
                    self._held[name] = _hold_value(name, position)
                    self._unheld.discard(name)
                elif name not in self._unheld:
                    self._unheld.add(name)
                    newly_unheld.append(name)
            if (
                self._last_status_t is None
                or state.t_now - self._last_status_t >= self._status_interval
            ):
                self._last_status_t = state.t_now
                status = self._status_locked()
            names = list(self._held)
            values = [self._held[name] for name in names]
        if newly_unheld:
            logger.warning(
                "Operator hold cannot hold joints without a valid position reading",
                joints=newly_unheld,
            )
        if status is not None:
            self._emit(status)
        if not names:
            return None
        return JointCommandOutput(
            joint_names=names, positions=values, mode=ControlMode.SERVO_POSITION
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Nothing should outrank a hold; losing joints means a priority misconfiguration."""
        logger.error(
            "Operator hold lost joints to a higher-priority task",
            by_task=by_task,
            joints=sorted(joints),
        )

    def request(self, route: str, reason: str = "") -> OperatorHoldStatus:
        """Freeze every joint where it is until acknowledge().

        Args:
            route: Who is asking: "manual", "failsafe" or "agent". Anything
                else raises ValueError and nothing changes.
            reason: Free text shown to the operator.

        A repeated request while on refreshes route and reason only; the
        joints stay where they were first frozen and started_at is kept.
        """
        route_value = HoldRoute(route).value
        with self._lock:
            if not self._on:
                self._on = True
                self._held = {}
                self._unheld = set()
                # Wall clock, for people reading the status; never used for control.
                self._started_at = time.time()
                self._last_status_t = None
            self._route = route_value
            self._reason = reason
            status = self._status_locked()
        logger.warning("Operator hold requested", route=route_value, reason=reason)
        return status

    def acknowledge(self) -> OperatorHoldStatus:
        """Release the hold. Nothing resumes by itself; the next task to command a joint takes it.

        Operator only: never expose this as an agent skill.
        """
        with self._lock:
            was_on = self._on
            self._on = False
            self._held = {}
            self._unheld = set()
            self._route = ""
            self._reason = ""
            self._started_at = 0.0
            self._last_status_t = None
            status = self._status_locked()
        if was_on:
            logger.warning("Operator hold acknowledged")
            self._emit(status)
        return status

    def get_status(self) -> OperatorHoldStatus:
        """The current hold state, without changing anything."""
        with self._lock:
            return self._status_locked()

    def _status_locked(self) -> OperatorHoldStatus:
        return OperatorHoldStatus(
            on=self._on,
            route=self._route,
            reason=self._reason,
            started_at=self._started_at,
            stamp=time.time(),
            unheld=tuple(sorted(self._unheld)),
        )

    def _emit(self, status: OperatorHoldStatus) -> None:
        if self._publish is None:
            return
        try:
            self._publish(status)
        except Exception:
            logger.exception("Publishing operator hold status failed")


class OperatorHoldTaskParams(BaseConfig):
    """Task-owned parameters from ``TaskConfig.params``."""

    status_interval: float = Field(default=1.0, gt=0.0, allow_inf_nan=False)


def create_task(cfg: Any, hardware: Any) -> OperatorHoldTask:
    if cfg.name != OPERATOR_HOLD_TASK_NAME:
        raise ValueError(
            f"operator hold task must be named {OPERATOR_HOLD_TASK_NAME!r}, got {cfg.name!r}"
        )
    if cfg.joint_names:
        raise ValueError(
            "operator hold claims every joint the coordinator reads; leave joint_names empty"
        )
    params = OperatorHoldTaskParams.model_validate(cfg.params)
    return OperatorHoldTask(priority=cfg.priority, status_interval=params.status_interval)
