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
from typing import TYPE_CHECKING, Protocol

from dimos.control.task import BaseControlTask, CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig


class ZeroStiffnessSource(Protocol):
    """A connection robot's joints and the ones it commands with zero stiffness."""

    @property
    def joint_names(self) -> list[str]: ...

    @property
    def zero_stiffness_joints(self) -> frozenset[str]: ...


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
    cfg: "TaskConfig", connections: Mapping[str, ZeroStiffnessSource]
) -> HandGuidingTask:
    """Build the task only for joints whose connection declares zero stiffness.

    A robot that declares nothing is refused, so a position-holding stack
    cannot fight the operator.
    """
    BaseConfig.model_validate(cfg.params)
    remaining = set(cfg.joint_names)
    for source in connections.values():
        claimed = remaining.intersection(source.joint_names)
        if not claimed:
            continue
        if not claimed <= source.zero_stiffness_joints:
            raise ValueError("Hand guiding requires a connection that declares zero stiffness")
        remaining.difference_update(source.joint_names)
    if remaining:
        raise ValueError(f"Hand guiding joints belong to no connected robot: {sorted(remaining)}")
    return HandGuidingTask(cfg.name, cfg.joint_names, cfg.priority)
