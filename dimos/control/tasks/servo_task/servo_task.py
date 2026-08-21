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

"""Streaming policy wrapper around joint trajectory execution."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import TYPE_CHECKING, Any

from pydantic import Field

from dimos.control.joint_limits import resolve_velocity_limits
from dimos.control.task import (
    BaseControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.JointState import JointState

logger = setup_logger()


@dataclass
class JointServoTaskConfig:
    """Configuration for servo task.

    Attributes:
        joint_names: List of joint names this task controls
        priority: Priority for arbitration (higher wins)
        timeout: If no command received for this many seconds, go inactive (0 = never timeout)
        default_positions: Optional initial target held until/unless a
            new target arrives via set_target(). Must match joint_names
            length if provided. Useful for "hold at this pose" tasks
            (e.g. arms during whole-body locomotion). Pair with
            timeout=0.0 to hold indefinitely.
        hold_measured_on_start: Latch the hold target from the first
            measured state instead of a configured pose. Startup is then
            bumpless: a configured pose would snap the joints there from
            wherever the robot actually is, at full stiffness.
        velocity_limits: Effective physical limits resolved by the task factory.
    """

    joint_names: list[str]
    priority: int = 10
    timeout: float = 0.5  # 500ms default timeout
    default_positions: list[float] | None = None
    hold_measured_on_start: bool = False
    velocity_limits: dict[str, float] | None = None


class JointServoTask(BaseControlTask):
    """Translate streamed position targets into replacement JTT trajectories."""

    def __init__(self, name: str, config: JointServoTaskConfig) -> None:
        if not config.joint_names:
            raise ValueError(f"JointServoTask '{name}' requires at least one joint")
        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._num_joints = len(config.joint_names)
        self._lock = threading.Lock()
        self._target: list[float] | None = None
        self._last_update_time: float = 0.0
        self._active = False
        self._name_to_index = {name: i for i, name in enumerate(self._joint_names_list)}
        self._preempted_joints: set[str] = set()
        self._logged_preemption: frozenset[str] = frozenset()
        self._trajectory_task = JointTrajectoryTask(
            JointTrajectoryTaskConfig(
                joint_names=tuple(config.joint_names),
                priority=config.priority,
                velocity_limits=config.velocity_limits,
                hold_final=True,
            )
        )
        if config.default_positions is not None:
            if len(config.default_positions) != self._num_joints:
                raise ValueError(
                    f"JointServoTask '{name}': default_positions length "
                    f"{len(config.default_positions)} does not match "
                    f"joint_names length {self._num_joints}"
                )
            if any(not math.isfinite(position) for position in config.default_positions):
                raise ValueError(f"JointServoTask '{name}': default_positions must be finite")
            self._target = list(config.default_positions)
            self._trajectory_task.replace(self._target_trajectory())
        logger.info(f"JointServoTask {name} initialized for joints: {config.joint_names}")

    def claim(self) -> ResourceClaim:
        return self._trajectory_task.claim()

    def is_active(self) -> bool:
        with self._lock:
            waiting_for_state = self._config.hold_measured_on_start and self._target is None
            return self._active and (waiting_for_state or self._target is not None)

    def _latch_measured(self, state: CoordinatorState) -> bool:
        measured: dict[str, float] = {}
        for name in self._joint_names_list:
            position = state.joints.joint_positions.get(name)
            if position is None:
                return False
            measured[name] = float(position)
        self._target = [measured[name] for name in self._joint_names_list]
        self._trajectory_task.replace(self._target_trajectory(), measured)
        logger.info(f"JointServoTask {self._name} holding measured start pose")
        return True

    def _target_trajectory(self) -> JointTrajectory:
        assert self._target is not None
        return JointTrajectory(
            joint_names=self._joint_names_list,
            points=[TrajectoryPoint(positions=list(self._target))],
        )

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._lock:
            if not self._active:
                return None
            if self._target is None:
                if not self._config.hold_measured_on_start or not self._latch_measured(state):
                    return None
            assert self._target is not None
            if self._preempted_joints:
                still_pending: set[str] = set()
                handoff: dict[str, float] = {}
                for name in self._preempted_joints:
                    position = state.joints.joint_positions.get(name)
                    if position is None:
                        still_pending.add(name)
                        continue
                    self._target[self._name_to_index[name]] = position
                    handoff[name] = position
                self._preempted_joints = still_pending
                self._trajectory_task.replace(self._target_trajectory(), handoff)
                self._last_update_time = state.t_now
            elif self._logged_preemption:
                self._logged_preemption = frozenset()
                logger.info(f"JointServoTask {self._name} resumed hold at handed-off positions")
            elapsed = state.t_now - self._last_update_time
            if self._config.timeout > 0 and elapsed > self._config.timeout:
                logger.warning(f"JointServoTask {self._name} timed out after {elapsed:.3f}s")
                self._active = False
                self._trajectory_task.cancel()
                return None
            return self._trajectory_task.compute(state)

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        overlap = joints & self._joint_names
        if not overlap:
            return
        with self._lock:
            self._preempted_joints |= overlap
            should_log = overlap != self._logged_preemption
            if should_log:
                self._logged_preemption = frozenset(overlap)
        if should_log:
            logger.warning(
                f"JointServoTask {self._name} preempted by {by_task} on joints "
                f"{sorted(overlap)}; hold target tracks measured positions until released"
            )

    def set_target(self, positions: list[float], t_now: float) -> bool:
        if len(positions) != self._num_joints:
            logger.warning(
                f"JointServoTask {self._name}: expected {self._num_joints} "
                f"positions, got {len(positions)}"
            )
            return False
        if any(not math.isfinite(position) for position in positions):
            logger.warning(f"JointServoTask {self._name}: positions must be finite")
            return False
        with self._lock:
            self._target = list(positions)
            self._trajectory_task.replace(self._target_trajectory())
            self._last_update_time = t_now
            self._active = True
        return True

    def set_target_by_name(self, positions: dict[str, float], t_now: float) -> bool:
        ordered: list[float] = []
        for name in self._joint_names_list:
            if name not in positions:
                logger.warning(
                    f"JointServoTask {self._name}: dropping command missing '{name}' "
                    f"(partial sets are ignored; name all {len(self._joint_names_list)} "
                    f"claimed joints)"
                )
                return False
            ordered.append(positions[name])

        return self.set_target(ordered, t_now)

    def on_joint_command(self, msg: JointState, t_now: float) -> bool:
        if not msg.position:
            return False
        return self.set_target_by_name(dict(zip(msg.name, msg.position, strict=True)), t_now)

    def start(self) -> None:
        with self._lock:
            self._active = True
            if self._target is not None:
                self._trajectory_task.replace(self._target_trajectory())
            self._last_update_time = time.perf_counter()
        logger.info(f"JointServoTask {self._name} started")

    def stop(self) -> None:
        with self._lock:
            self._active = False
            self._trajectory_task.cancel()
        logger.info(f"JointServoTask {self._name} stopped")

    def clear(self) -> None:
        with self._lock:
            self._target = None
            self._active = False
            self._trajectory_task.cancel()
        logger.info(f"JointServoTask {self._name} cleared")

    def is_streaming(self) -> bool:
        with self._lock:
            return self._active and self._target is not None


class JointServoTaskParams(BaseConfig):
    timeout: float | None = None
    default_positions: list[float] | None = None
    hold_measured_on_start: bool = False
    speed_scale: float = Field(default=1.0, gt=0.0, le=1.0, allow_inf_nan=False)


def create_task(cfg: Any, hardware: Any) -> JointServoTask:
    params = JointServoTaskParams.model_validate(cfg.params)
    velocity_limits = resolve_velocity_limits(
        cfg.joint_names,
        hardware,
        speed_scale=params.speed_scale,
    )
    kwargs: dict[str, object] = {
        "joint_names": cfg.joint_names,
        "priority": cfg.priority,
        "velocity_limits": velocity_limits,
    }
    if params.timeout is not None:
        kwargs["timeout"] = params.timeout
    if params.default_positions is not None:
        kwargs["default_positions"] = params.default_positions
        # Zero timeout pairs naturally with default-hold.
        kwargs.setdefault("timeout", 0.0)
    kwargs["hold_measured_on_start"] = params.hold_measured_on_start
    if params.hold_measured_on_start:
        kwargs.setdefault("timeout", 0.0)
    return JointServoTask(cfg.name, JointServoTaskConfig(**kwargs))  # type: ignore[arg-type]
