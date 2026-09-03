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

"""Measured-position passthrough for gravity-compensated teaching."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)


@dataclass(frozen=True)
class TeachControlTaskConfig:
    """Configuration for a gravity-compensated teach task."""

    joint_names: tuple[str, ...]
    priority: int = 10


class TeachControlTask(BaseControlTask):
    """Continuously command each joint's measured position.

    On zero-stiffness whole-body hardware this keeps gravity compensation and
    damping active while allowing an operator to move the mechanism by hand.
    """

    def __init__(self, name: str, config: TeachControlTaskConfig) -> None:
        self._name = name
        self._config = config

    def claim(self) -> ResourceClaim:
        """Claim the taught joints at the configured priority."""
        return ResourceClaim(
            joints=frozenset(self._config.joint_names),
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        """Keep the hardware control loop active for the entire run."""
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Mirror a complete, finite measured-position snapshot."""
        positions: list[float] = []
        for joint_name in self._config.joint_names:
            position = state.joints.get_position(joint_name)
            if position is None or not math.isfinite(position):
                return None
            positions.append(position)
        return JointCommandOutput(
            joint_names=list(self._config.joint_names),
            positions=positions,
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Allow higher-priority tasks to override individual joints."""


def _validate_hardware(cfg: Any, hardware: Any) -> None:
    where = f"teach task {cfg.name!r}"
    joint_names = list(cfg.joint_names)
    if not joint_names:
        raise ValueError(f"{where}: requires at least one joint")
    if len(set(joint_names)) != len(joint_names):
        raise ValueError(f"{where}: joint_names must not contain duplicates")

    owners: list[ConnectedWholeBody] = []
    for joint_name in joint_names:
        matches = [
            connected
            for connected in (hardware or {}).values()
            if joint_name in connected.component.joints
        ]
        if not matches:
            raise ValueError(f"{where}: joint {joint_name!r} is not owned by coordinator hardware")
        if len(matches) > 1:
            raise ValueError(f"{where}: joint {joint_name!r} is owned by multiple components")
        owner = matches[0]
        if not isinstance(owner, ConnectedWholeBody):
            raise ValueError(f"{where}: requires whole-body hardware")
        owners.append(owner)

    owner = owners[0]
    if any(candidate is not owner for candidate in owners[1:]):
        raise ValueError(f"{where}: all joints must belong to one whole-body component")
    component = owner.component
    wb_config = component.wb_config
    if wb_config is None or wb_config.kp is None or wb_config.kd is None:
        raise ValueError(f"{where}: whole-body hardware requires explicit kp and kd")
    if len(wb_config.kp) != len(component.joints) or len(wb_config.kd) != len(component.joints):
        raise ValueError(
            f"{where}: kp and kd must match the component's {len(component.joints)} joints"
        )

    indices = [component.joints.index(name) for name in joint_names]
    stiffness = [wb_config.kp[index] for index in indices]
    damping = [wb_config.kd[index] for index in indices]
    if any(not math.isfinite(value) for value in [*stiffness, *damping]):
        raise ValueError(f"{where}: kp and kd must be finite")
    if any(value != 0.0 for value in stiffness):
        raise ValueError(f"{where}: requires zero stiffness (kp=0) for every taught joint")
    if any(value < 0.0 for value in damping):
        raise ValueError(f"{where}: damping (kd) must be non-negative")


def create_task(cfg: Any, hardware: Any) -> TeachControlTask:
    """Build and validate a teach task from coordinator configuration."""
    _validate_hardware(cfg, hardware)
    return TeachControlTask(
        cfg.name,
        TeachControlTaskConfig(
            joint_names=tuple(cfg.joint_names),
            priority=cfg.priority,
        ),
    )
