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

"""Control task implemented by an external package."""

from __future__ import annotations

from collections.abc import Mapping
import logging

from dimos.control.coordinator import TaskConfig
from dimos.control.hardware_interface import ConnectedHardware, ConnectedWholeBody
from dimos.control.task import BaseControlTask, CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.hardware.manipulators.spec import ControlMode

LOGGER = logging.getLogger("dimos_external_control_extension")


class ExternalTestDriveTask(BaseControlTask):
    """Always-active demo task that commands a tiny base velocity."""

    def __init__(
        self,
        cfg: TaskConfig,
        hardware: Mapping[str, ConnectedHardware | ConnectedWholeBody],
    ) -> None:
        self._name = cfg.name
        self._joint_names = list(cfg.joint_names)
        self._priority = cfg.priority
        velocity = cfg.params.get("velocity", 0.1)
        self._velocity = float(velocity)
        self._tick_count = 0
        self._hardware_ids = sorted(hardware.keys())
        LOGGER.info(
            "[external_test_robot] ExternalTestDriveTask created for task=%s hardware=%s",
            cfg.name,
            self._hardware_ids,
        )

    @property
    def name(self) -> str:
        return self._name

    @property
    def tick_count(self) -> int:
        return self._tick_count

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=frozenset(self._joint_names),
            priority=self._priority,
            mode=ControlMode.VELOCITY,
        )

    def is_active(self) -> bool:
        return True

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        LOGGER.info(
            "[external_test_robot] ExternalTestDriveTask.on_preempted(by_task=%s, joints=%s)",
            by_task,
            sorted(joints),
        )

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        self._tick_count += 1
        velocities = [self._velocity, 0.0, 0.0][: len(self._joint_names)]
        LOGGER.info(
            "[external_test_robot] ExternalTestDriveTask.compute(tick=%s, dt=%.4f)",
            self._tick_count,
            state.dt,
        )
        return JointCommandOutput(
            joint_names=self._joint_names,
            velocities=velocities,
            mode=ControlMode.VELOCITY,
        )


def create_task(
    cfg: TaskConfig,
    hardware: Mapping[str, ConnectedHardware | ConnectedWholeBody],
) -> ExternalTestDriveTask:
    return ExternalTestDriveTask(cfg=cfg, hardware=hardware)


__all__ = ["ExternalTestDriveTask", "create_task"]
