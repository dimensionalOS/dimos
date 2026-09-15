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

"""Hold joints at a latched pose while the base drives.

A mobile base accelerating under an arm that nothing is commanding leaves the
arm to swing on its own inertia; on a bimanual robot the two can reach each
other. This task watches the base's own joints on the coordinator and, while
the base is moving, holds its arm joints at the pose they were in when the
motion started.

The base is read from CoordinatorState rather than from a twist stream: for BASE
hardware the coordinator fills the virtual joints' velocity from the commanded
twist, and every writer - teleop, a route follower, the base half of a whole-body
plan - is already aggregated there. Watching one command stream would see only
one of them.

It claims SERVO_POSITION like the trajectory task but at a lower priority, so a
real plan preempts it: hold is what happens when nothing else wants the arm.
On preemption the latch is dropped, and when the task next takes the joints
back it latches wherever the arm now is rather than snapping to a stale pose.

    joint_hold_task(alfred_arm_joints())
"""

from __future__ import annotations

from dataclasses import dataclass, field
import threading
from typing import TYPE_CHECKING, Any

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
    from dimos.control.coordinator import TaskConfig

logger = setup_logger()

# Below the trajectory task's 10, so a plan always wins the joints.
DEFAULT_HOLD_PRIORITY = 5


@dataclass
class JointHoldTaskConfig:
    """Configuration for the joint hold task.

    Attributes:
        joint_names: Joints to hold. Usually the arms; leave the lift out, it
            carries its own load and does not swing.
        base_joint_names: The base's virtual twist joints (vx, vy, wz) to read
            motion from. Empty disables the automatic trigger, leaving only the
            explicit hold() rpc.
        priority: Arbitration priority. Must stay below the trajectory task or
            hold would fight every plan.
        linear_threshold: Base speed (m/s) above which the base counts as moving.
        angular_threshold: Base yaw rate (rad/s) above which it counts as moving.
        release_after_s: Keep holding this long after the base goes quiet, so a
            stop-start does not drop and re-latch on every gap in the stream.
        drift_warn_rad: Warn once per joint if the measured pose departs this far
            from the latched target while holding.
    """

    joint_names: list[str]
    base_joint_names: list[str] = field(default_factory=list)
    priority: int = DEFAULT_HOLD_PRIORITY
    linear_threshold: float = 0.02
    angular_threshold: float = 0.05
    release_after_s: float = 0.5
    drift_warn_rad: float = 0.15


class JointHoldTask(BaseControlTask):
    """Hold joints where they were when the base started moving.

    The twist stream drives the task rather than the joint state: a base that is
    commanded to move is about to accelerate the arm, and waiting to see the arm
    swing before holding it is already too late.

    Example:
        >>> task = JointHoldTask(
        ...     "hold_arms",
        ...     JointHoldTaskConfig(
        ...         joint_names=["left/joint1", "left/joint2"],
        ...         base_joint_names=["base/vx", "base/vy", "base/wz"],
        ...     ),
        ... )
        >>> task.claim().priority < 10  # a plan always outranks the hold
        True
    """

    def __init__(self, name: str, config: JointHoldTaskConfig) -> None:
        if not config.joint_names:
            raise ValueError(f"JointHoldTask '{name}' requires at least one joint")
        if config.release_after_s < 0.0:
            raise ValueError(f"JointHoldTask '{name}' needs release_after_s >= 0")

        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._base_joint_names = list(config.base_joint_names)

        self._lock = threading.Lock()
        self._enabled = True
        # Deadline, on the coordinator clock, until which the base counts as moving.
        self._hold_until: float | None = None
        self._latched: list[float] | None = None
        self._warned_missing = False
        self._warned_drift: set[str] = set()

        logger.info(f"JointHoldTask {name} initialized for joints: {config.joint_names}")

    @property
    def config(self) -> JointHoldTaskConfig:
        return self._config

    def claim(self) -> ResourceClaim:
        """Declare resource requirements."""
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        """Participate whenever enabled; compute() decides if there is a hold.

        The base's motion is only legible from CoordinatorState, which is_active
        does not receive, so the decision moves into compute(). This task sits at
        the bottom of the stack, so claiming the joints and then declining to
        command them blocks nothing underneath.
        """
        with self._lock:
            return self._enabled

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Hold the latched pose, latching it on the first tick of a hold."""
        moving = self._base_is_moving(state)
        with self._lock:
            if not self._enabled:
                return None

            if moving:
                self._hold_until = state.t_now + self._config.release_after_s

            if self._hold_until is None:
                return None

            # The release window runs on the coordinator clock, so a base that
            # stops being commanded still releases the joints rather than
            # holding them until something says so.
            if state.t_now >= self._hold_until:
                self._reset_locked()
                return None

            if self._latched is None:
                latched = self._latch_locked(state)
                if latched is None:
                    return None
                self._latched = latched

            self._warn_on_drift_locked(state)
            positions = list(self._latched)

        return JointCommandOutput(
            joint_names=list(self._joint_names_list),
            positions=positions,
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Drop the latch so the next hold starts from wherever the arm ends up."""
        if not (joints & self._joint_names):
            return
        with self._lock:
            self._latched = None
            self._warned_drift.clear()
        logger.debug(f"JointHoldTask {self._name} preempted by {by_task}, latch dropped")

    def _base_is_moving(self, state: CoordinatorState) -> bool:
        """True when any base axis is being commanded above its threshold.

        For BASE hardware the coordinator reports the commanded twist as the
        virtual joints' velocity, so this reads the aggregate of every writer.
        """
        if not self._base_joint_names:
            return False
        linear_sq = 0.0
        angular = 0.0
        for index, joint in enumerate(self._base_joint_names):
            value = state.joints.get_velocity(joint)
            if value is None:
                continue
            # vx, vy, wz by position: the first two are linear, the third angular.
            if index < 2:
                linear_sq += float(value) ** 2
            else:
                angular = max(angular, abs(float(value)))
        return (
            linear_sq > self._config.linear_threshold**2 or angular > self._config.angular_threshold
        )

    def start(self) -> None:
        """Enable the task. ``auto_start`` calls this; holding still waits for the base."""
        self.set_enabled(True)

    def stop(self) -> None:
        """Disable the task and let the joints go."""
        self.set_enabled(False)

    def hold(self, t_now: float) -> bool:
        """Hold now, without waiting for a twist. Returns False when disabled."""
        with self._lock:
            if not self._enabled:
                return False
            self._hold_until = t_now + self._config.release_after_s
            return True

    def release(self) -> None:
        """Drop the hold and the latch."""
        with self._lock:
            self._reset_locked()

    def set_enabled(self, enabled: bool) -> None:
        """Turn the task on or off; disabling releases immediately."""
        with self._lock:
            self._enabled = bool(enabled)
            if not self._enabled:
                self._reset_locked()

    def get_status(self) -> dict[str, Any]:
        """Current enable, hold and latch state."""
        with self._lock:
            return {
                "enabled": self._enabled,
                "holding": self._hold_until is not None,
                "latched": self._latched is not None,
                "joints": list(self._joint_names_list),
                "priority": self._config.priority,
            }

    def _reset_locked(self) -> None:
        self._hold_until = None
        self._latched = None
        self._warned_drift.clear()

    def _latch_locked(self, state: CoordinatorState) -> list[float] | None:
        """Capture the current pose, or None if any joint is missing this tick."""
        positions: list[float] = []
        missing: list[str] = []
        for joint in self._joint_names_list:
            value = state.joints.get_position(joint)
            if value is None:
                missing.append(joint)
                continue
            positions.append(float(value))

        if missing:
            # Holding a subset would pin some joints and leave the rest to swing
            # into them, which is the failure this task exists to prevent.
            if not self._warned_missing:
                self._warned_missing = True
                logger.warning(
                    "JointHoldTask cannot latch, joints missing from coordinator state",
                    task=self._name,
                    missing_joints=missing,
                )
            return None

        self._warned_missing = False
        logger.debug(f"JointHoldTask {self._name} latched {len(positions)} joints")
        return positions

    def _warn_on_drift_locked(self, state: CoordinatorState) -> None:
        """A held joint that keeps moving is being back-driven; say so once."""
        if self._latched is None:
            return
        for joint, target in zip(self._joint_names_list, self._latched, strict=True):
            if joint in self._warned_drift:
                continue
            measured = state.joints.get_position(joint)
            if measured is None:
                continue
            if abs(float(measured) - target) > self._config.drift_warn_rad:
                self._warned_drift.add(joint)
                logger.warning(
                    "JointHoldTask joint drifting away from its hold target",
                    task=self._name,
                    joint=joint,
                    target=target,
                    measured=float(measured),
                )


class JointHoldTaskParams(BaseConfig):
    """Validated ``TaskConfig.params`` for the joint hold task."""

    base_joint_names: list[str] = []
    linear_threshold: float = 0.02
    angular_threshold: float = 0.05
    release_after_s: float = 0.5
    drift_warn_rad: float = 0.15


def joint_hold_task(
    joint_names: list[str],
    base_joint_names: list[str],
    *,
    name: str = "joint_hold",
    priority: int = DEFAULT_HOLD_PRIORITY,
    release_after_s: float = 0.5,
) -> TaskConfig:
    """A ``TaskConfig`` for holding ``joint_names`` while the base drives.

    ``base_joint_names`` are the base's virtual twist joints on the same
    coordinator; their commanded velocity is what triggers the hold.
    """
    from dimos.control.coordinator import TaskConfig

    return TaskConfig(
        name=name,
        type="joint_hold",
        joint_names=list(joint_names),
        priority=priority,
        auto_start=True,
        params={
            "release_after_s": release_after_s,
            "base_joint_names": list(base_joint_names),
        },
    )


def create_task(cfg: Any, hardware: Any) -> JointHoldTask:
    params = JointHoldTaskParams.model_validate(cfg.params)
    return JointHoldTask(
        cfg.name,
        JointHoldTaskConfig(
            joint_names=cfg.joint_names,
            base_joint_names=params.base_joint_names,
            priority=cfg.priority,
            linear_threshold=params.linear_threshold,
            angular_threshold=params.angular_threshold,
            release_after_s=params.release_after_s,
            drift_warn_rad=params.drift_warn_rad,
        ),
    )
