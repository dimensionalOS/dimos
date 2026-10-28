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

"""Dual OpenYAM coordinator and planning blueprints."""

import math

from dimos.control.coordinator import ControlCoordinatorConfig, TaskConfig
from dimos.control.task import BaseControlTask, CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.hardware.manipulators.spec import ControlMode
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.dual_openyam.config import (
    dual_openyam_arm_joints,
    dual_openyam_hardware,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_ARM_JOINTS,
)
from dimos.robot.manipulators.dual_openyam.model import DUAL_OPENYAM_MODEL


def dual_openyam_trajectory_task(*, priority: int = 20) -> TaskConfig:
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(DUAL_OPENYAM_ARM_JOINTS),
        priority=priority,
        params={"start_position_tolerance": 0.05},
    )


class DualOpenYamCoordinatorConfig(ControlCoordinatorConfig):
    """Dual OpenYAM deployment configuration."""

    left_can_port: str | None = None
    right_can_port: str | None = None


class JointOffsetHandoffHold(BaseControlTask):
    """Keep one arm at the measured handoff pose until another task takes over."""

    def __init__(self, side: str) -> None:
        self._name = f"joint_offset_handoff_{side}"
        self._joints = dual_openyam_arm_joints(side)
        self._positions: list[float] | None = None

    def hold(self, positions: list[float]) -> None:
        if len(positions) != len(self._joints) or not all(map(math.isfinite, positions)):
            raise ValueError("Handoff hold needs six finite arm positions")
        self._positions = positions.copy()

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=frozenset(self._joints) if self._positions is not None else frozenset(),
            priority=5,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        return self._positions is not None

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if self._positions is None:
            return None
        return JointCommandOutput(
            joint_names=self._joints.copy(),
            positions=self._positions.copy(),
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints:
            self._positions = None


class DualOpenYamCoordinator(TeleopControlCoordinator):
    """Select mock or explicit dual-CAN hardware during coordinator setup."""

    config: DualOpenYamCoordinatorConfig

    @rpc
    def pause_for_joint_offset_test(self) -> bool:
        """Stop teleop commands while keeping the motor adapters connected and enabled."""
        loop = self._tick_loop
        if loop is None or not loop.is_running:
            return False
        loop.stop()
        return loop._tick_thread is None or not loop._tick_thread.is_alive()

    @rpc
    def resume_after_joint_offset_test(self) -> bool:
        """Hold the current arm poses and resume teleop; each grip takes its arm back."""
        loop = self._tick_loop
        if loop is None or loop.is_running:
            return False
        if loop._tick_thread is not None and loop._tick_thread.is_alive():
            return False
        positions = self.get_joint_positions()
        for side in ("left", "right"):
            name = f"joint_offset_handoff_{side}"
            task = self._tasks.get(name)
            if task is None:
                task = JointOffsetHandoffHold(side)
                self.add_task(task)
            if not isinstance(task, JointOffsetHandoffHold):
                raise RuntimeError(f"Unexpected handoff task {name}")
            task.hold([positions[joint] for joint in dual_openyam_arm_joints(side)])
        loop.start()
        return loop.is_running

    def _setup_from_config(self) -> None:
        self.config.hardware = [
            dual_openyam_hardware(
                left_can_port=self.config.left_can_port,
                right_can_port=self.config.right_can_port,
            )
        ]
        # Resolve assets at startup, using the same bounds as the planning model.
        component = self.config.hardware[0]
        assert component.limits is not None
        lower = list(component.limits.position_lower)
        upper = list(component.limits.position_upper)
        model = DUAL_OPENYAM_MODEL.load()
        for name in DUAL_OPENYAM_ARM_JOINTS:
            joint = model.get_joint(name)
            if (
                joint is None
                or joint.lower is None
                or joint.upper is None
                or not math.isfinite(joint.lower)
                or not math.isfinite(joint.upper)
                or joint.lower >= joint.upper
            ):
                raise ValueError(f"Dual OpenYAM model has invalid position limits for {name!r}")
            index = component.joints.index(name)
            lower[index] = joint.lower
            upper[index] = joint.upper
        component.limits.position_lower = lower
        component.limits.position_upper = upper
        super()._setup_from_config()


coordinator_dual_openyam = DualOpenYamCoordinator.blueprint(
    tasks=[dual_openyam_trajectory_task()],
)

dual_openyam_planner_coordinator = autoconnect(
    planner(model=dual_openyam_model_config()),
    DualOpenYamCoordinator.blueprint(
        tasks=[dual_openyam_trajectory_task()],
    ),
)
