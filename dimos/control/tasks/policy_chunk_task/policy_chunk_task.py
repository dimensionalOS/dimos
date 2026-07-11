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

"""Control task for live robot policy action chunks."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import threading
from typing import Any, Literal

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.robot_learning.policy_rollout.models import RobotPolicyActionChunk
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

ActionMapping = Literal["delta", "target"]


@dataclass
class PolicyChunkControlTaskConfig:
    joint_names: list[str]
    accepted_action_space_id: str
    priority: int = 10
    ticks_per_action: int = 1
    execute_first_n: int | None = None
    stale_timeout_ticks: int = 10
    action_scale: float = 1.0
    action_mapping: ActionMapping = "delta"
    gripper_joint_name: str | None = None
    gripper_action_index: int | None = None
    gripper_min: float = 0.0
    gripper_max: float = 1.0


class PolicyChunkControlTask(BaseControlTask):
    """Execute normalized robot policy action chunks as joint commands."""

    def __init__(self, name: str, config: PolicyChunkControlTaskConfig) -> None:
        if not config.joint_names:
            raise ValueError(f"PolicyChunkControlTask '{name}' requires at least one joint")
        if config.ticks_per_action < 1:
            raise ValueError("ticks_per_action must be >= 1")
        if config.stale_timeout_ticks < 0:
            raise ValueError("stale_timeout_ticks must be >= 0")
        self._name = name
        self._config = config
        self._joint_names_list = list(config.joint_names)
        self._claimed_joints = set(config.joint_names)
        if config.gripper_joint_name is not None:
            self._claimed_joints.add(config.gripper_joint_name)
        self._lock = threading.Lock()
        self._actions: list[tuple[float, ...]] = []
        self._ticks_since_chunk = 0
        self._enabled = False
        self._active = False
        self._trigger: Callable[[], object] | None = None
        self._trigger_in_flight = False
        self._accepted_chunk_count = 0
        self._trigger_count = 0
        self._trigger_status_counts: dict[str, int] = {}
        self._consumed_action_count = 0
        self._stale_deactivation_count = 0

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=frozenset(self._claimed_joints),
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        with self._lock:
            return self._enabled or self._active

    def start(self) -> None:
        with self._lock:
            self._enabled = True

    def stop(self) -> None:
        with self._lock:
            self._enabled = False
            self._active = False

    def on_robot_policy_action_chunk(self, chunk: object, t_now: float) -> bool:
        del t_now
        if not isinstance(chunk, RobotPolicyActionChunk):
            return False
        if chunk.space_id != self._config.accepted_action_space_id:
            logger.warning(
                f"PolicyChunkControlTask {self._name}: rejected action space {chunk.space_id!r}"
            )
            return False
        if not self._validate_values(chunk.values):
            return False
        action_count = chunk.horizon
        if self._config.execute_first_n is not None:
            action_count = min(action_count, self._config.execute_first_n)
        if action_count <= 0:
            return False
        with self._lock:
            self._actions = list(chunk.values[:action_count])
            self._ticks_since_chunk = 0
            self._active = True
            self._trigger_in_flight = False
            self._accepted_chunk_count += 1
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        del state.t_now
        with self._lock:
            if not self._active:
                self._maybe_trigger_locked()
                return None
            if self._ticks_since_chunk > self._config.stale_timeout_ticks:
                self._active = False
                self._stale_deactivation_count += 1
                return None
            action_index = self._ticks_since_chunk // self._config.ticks_per_action
            if action_index >= len(self._actions):
                self._active = False
                self._maybe_trigger_locked()
                return None
            row = self._actions[action_index]
            self._ticks_since_chunk += 1
            self._consumed_action_count += 1

        positions = self._positions_from_row(row, state)
        if positions is None:
            return None
        return JointCommandOutput(
            joint_names=self._output_joint_names(),
            positions=positions,
            mode=ControlMode.SERVO_POSITION,
        )

    def set_policy_trigger(self, trigger: Callable[[], object]) -> None:
        self._trigger = trigger

    def diagnostics(self) -> dict[str, int | dict[str, int]]:
        with self._lock:
            return {
                "accepted_chunks": self._accepted_chunk_count,
                "refill_triggers": self._trigger_count,
                "inference_status_counts": dict(self._trigger_status_counts),
                "consumed_actions": self._consumed_action_count,
                "stale_deactivations": self._stale_deactivation_count,
            }

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._claimed_joints:
            logger.warning(
                f"PolicyChunkControlTask {self._name} preempted by {by_task} on {joints}"
            )

    def _maybe_trigger_locked(self) -> None:
        if self._trigger is None or self._trigger_in_flight:
            return
        self._trigger_in_flight = True
        try:
            result = self._trigger()
            status = str(getattr(result, "status", "unknown"))
            self._trigger_status_counts[status] = self._trigger_status_counts.get(status, 0) + 1
            accepted = getattr(result, "accepted", True)
            if accepted:
                self._trigger_count += 1
            else:
                self._trigger_in_flight = False
        except Exception:
            self._trigger_in_flight = False
            logger.exception(f"PolicyChunkControlTask {self._name}: policy trigger failed")

    def _validate_values(self, values: tuple[tuple[float, ...], ...]) -> bool:
        required_dim = len(self._joint_names_list)
        if self._config.gripper_joint_name is not None:
            required_dim = max(
                required_dim, (self._config.gripper_action_index or required_dim) + 1
            )
        action_dim = len(values[0])
        if action_dim < required_dim:
            logger.warning(
                f"PolicyChunkControlTask {self._name}: action_dim {action_dim} < required {required_dim}"
            )
            return False
        for row in values:
            if len(row) != action_dim:
                return False
            for value in row:
                if not math.isfinite(value) or value < -1.0 or value > 1.0:
                    logger.warning(
                        f"PolicyChunkControlTask {self._name}: invalid action value {value}"
                    )
                    return False
        return True

    def _positions_from_row(
        self, row: tuple[float, ...], state: CoordinatorState
    ) -> list[float] | None:
        positions: list[float] = []
        for index, joint_name in enumerate(self._joint_names_list):
            value = row[index] * self._config.action_scale
            if self._config.action_mapping == "delta":
                current = state.joints.get_position(joint_name)
                if current is None:
                    return None
                positions.append(current + value)
            else:
                positions.append(value)
        if self._config.gripper_joint_name is not None:
            gripper_index = self._config.gripper_action_index
            if gripper_index is None:
                gripper_index = len(self._joint_names_list)
            normalized = (row[gripper_index] + 1.0) / 2.0
            positions.append(
                self._config.gripper_min
                + normalized * (self._config.gripper_max - self._config.gripper_min)
            )
        return positions

    def _output_joint_names(self) -> list[str]:
        names = list(self._joint_names_list)
        if self._config.gripper_joint_name is not None:
            names.append(self._config.gripper_joint_name)
        return names


class PolicyChunkControlTaskParams(BaseConfig):
    accepted_action_space_id: str
    ticks_per_action: int = 1
    execute_first_n: int | None = None
    stale_timeout_ticks: int = 10
    policy_trigger_method: str | None = None
    policy_trigger_config_name: str | None = None
    action_scale: float = 1.0
    action_mapping: ActionMapping = "delta"
    gripper_joint_name: str | None = None
    gripper_action_index: int | None = None
    gripper_min: float = 0.0
    gripper_max: float = 1.0


def create_task(cfg: Any, hardware: Any) -> PolicyChunkControlTask:
    del hardware
    params = PolicyChunkControlTaskParams.model_validate(cfg.params)
    return PolicyChunkControlTask(
        cfg.name,
        PolicyChunkControlTaskConfig(
            joint_names=cfg.joint_names,
            accepted_action_space_id=params.accepted_action_space_id,
            priority=cfg.priority,
            ticks_per_action=params.ticks_per_action,
            execute_first_n=params.execute_first_n,
            stale_timeout_ticks=params.stale_timeout_ticks,
            action_scale=params.action_scale,
            action_mapping=params.action_mapping,
            gripper_joint_name=params.gripper_joint_name,
            gripper_action_index=params.gripper_action_index,
            gripper_min=params.gripper_min,
            gripper_max=params.gripper_max,
        ),
    )
