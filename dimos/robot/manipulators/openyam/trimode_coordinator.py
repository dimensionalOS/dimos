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

"""Quest/trajectory producers; arm arbitration and routing are ordinary Layer A."""

from __future__ import annotations

import threading
import time
from typing import Literal

from reactivex.disposable import Disposable

from dimos.control.coordinator import ControlCoordinatorConfig, TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openyam.joints import OPENYAM_GRIPPER_JOINT
from dimos.robot.manipulators.openyam.trimode_connection import (
    TrimodeLifecycle,
    arm_bounds,
    declaration,
)
from dimos.robot.manipulators.openyam.trimode_tasks import (
    POSE,
    XYZ,
    PlannedReference,
    QuestReference,
)
from dimos.robot.manipulators.openyam.trimode_tracker import (
    REFERENCE_TIMEOUT,
    ArmSample,
    JointGoal,
    PoseGoal,
    PositionGoal,
)


class TrimodeCoordinatorConfig(ControlCoordinatorConfig):
    quest_mode: Literal["pose", "position"] = "pose"


class TrimodeCoordinator(TeleopControlCoordinator):
    """No arm writer or inference here; RPC is only for the reference lifecycle."""

    config: TrimodeCoordinatorConfig  # type: ignore[assignment]
    _lifecycle: TrimodeLifecycle
    arm_sample: In[ArmSample]
    arm_joint_state: In[JointState]
    arm_position_command: Out[JointState]  # stock jaw only
    arm_joint_reference: Out[JointGoal]
    arm_tcp_pose: Out[PoseGoal]
    arm_tcp_position: Out[PositionGoal]
    _sample: ArmSample | None = None

    def _on_sample(self, sample: ArmSample) -> None:
        if isinstance(sample, ArmSample) and (self._sample is None or sample.at > self._sample.at):
            self._sample = sample

    def _read_sample(self) -> ArmSample | None:
        sample = self._sample
        if sample is not None and 0 <= time.monotonic() - sample.at < REFERENCE_TIMEOUT:
            return sample
        return None

    def _connect_robots(self) -> None:
        if (
            len(self.config.connections) != 1
            or self.config.hardware
            or self.config.tasks
            or self.config.zero_stiffness_joints
        ):
            raise ValueError("tri-mode requires its canonical connection and built-in tasks only")
        contract = self.config.connections[0]
        model = contract.model
        expected = declaration(model)
        if (
            model is None
            or contract.source != expected.source
            or contract.joint_commands != expected.joint_commands
            or contract.group_commands != expected.group_commands
        ):
            raise ValueError("tri-mode forbids alternate arm writers")
        self._remote_estop_lock = threading.Lock()
        super()._connect_robots()
        self.register_disposable(Disposable(self.arm_sample.subscribe(self._on_sample)))

        def takeover() -> None:
            plan.cancel()

        quest = QuestReference(
            POSE if self.config.quest_mode == "pose" else XYZ,
            self._read_sample,
            takeover,
            self._lifecycle,
        )
        plan = PlannedReference(
            arm_bounds(model), self._read_sample, lambda: quest.blocks_plan, self._lifecycle
        )
        self.add_task(quest, task_type="teleop_ik")
        self.add_task(plan, task_type="trajectory")
        gripper = TaskConfig(
            name="arm_gripper",
            type="gripper",
            joint_names=[OPENYAM_GRIPPER_JOINT],
            priority=20,
            params={"hold_duration": 0.1, "open_at": "lower"},
            stream_bind={"gripper_command": "right_gripper_command"},
        )
        self.add_task(
            self._create_task_from_config(gripper),
            task_type=gripper.type,
            stream_bind=gripper.stream_bind,
        )

    @rpc
    def set_estop(self, estopped: bool) -> bool:
        with self._remote_estop_lock:
            if not estopped:
                self._lifecycle.stop_references(False)
                return super().set_estop(False)
            try:
                self._lifecycle.stop_references(True)
            finally:
                result = super().set_estop(True)
            return result

    @rpc
    def reset_runtime_state(self, reactivate: bool | None = None) -> dict[str, bool]:
        results = super().reset_runtime_state(reactivate)
        with self._task_lock:
            self._invalidate_group_results()
            for task in self._tasks.values():
                if isinstance(task, (QuestReference, PlannedReference)):
                    results[task.name] = task.reset_runtime_state(reactivate)
        return results

    @rpc
    def stop(self) -> None:
        try:
            super().stop()
        finally:
            with self._task_lock:
                for task in self._tasks.values():
                    if isinstance(task, (QuestReference, PlannedReference)):
                        task.stop()
