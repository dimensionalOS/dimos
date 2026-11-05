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


"""Keep zero-stiffness whole-body commands flowing during hand guiding."""

from collections.abc import Mapping
import math
from typing import TYPE_CHECKING

from dimos.control.hardware_interface import ConnectedHardware, ConnectedWholeBody
from dimos.control.task import BaseControlTask, CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig


class HandGuidingTask(BaseControlTask):
    """Follow measured positions while the adapter supplies damping and gravity torque.

    The hardware must have zero position gains. This task supplies the periodic
    writes needed by torque-compensating adapters; it does not compute gravity.
    """

    def __init__(self, name: str, joint_names: list[str], priority: int) -> None:
        if not joint_names or len(set(joint_names)) != len(joint_names):
            raise ValueError("Hand guiding requires nonempty, unique joint names")
        self._name = name
        self._joint_names = list(joint_names)
        self._claim = ResourceClaim(frozenset(joint_names), priority=priority)
        self._active = False
        self._estopped = False

    def claim(self) -> ResourceClaim:
        return self._claim

    def start(self) -> bool:
        self._active = not self._estopped
        return self._active

    def stop(self) -> bool:
        self._active = False
        return True

    def set_estop(self, estopped: bool) -> None:
        self._estopped = estopped
        if estopped:
            self.stop()

    def is_active(self) -> bool:
        return self._active

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self._active:
            return None
        positions = [state.joints.get_position(name) for name in self._joint_names]
        if any(position is None or not math.isfinite(position) for position in positions):
            return None
        return JointCommandOutput(
            joint_names=list(self._joint_names),
            positions=[float(position) for position in positions if position is not None],
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._claim.joints:
            self.stop()


def create_task(
    cfg: "TaskConfig",
    hardware: Mapping[str, ConnectedHardware | ConnectedWholeBody],
) -> HandGuidingTask:
    BaseConfig.model_validate(cfg.params)
    remaining = set(cfg.joint_names)
    for connected in hardware.values():
        if not remaining.intersection(connected.joint_names):
            continue
        gains = connected.component.wb_config
        if (
            not isinstance(connected, ConnectedWholeBody)
            or gains is None
            or gains.kp is None
            or any(gain != 0.0 for gain in gains.kp)
        ):
            raise ValueError("Hand guiding requires whole-body hardware with explicit zero kp")
        remaining.difference_update(connected.joint_names)
    if remaining:
        raise ValueError(f"Hand guiding joints have no connected hardware: {sorted(remaining)}")
    return HandGuidingTask(cfg.name, cfg.joint_names, cfg.priority)
