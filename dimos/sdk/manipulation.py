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

"""Client-only conveniences for sequential arm motion over existing RPCs."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import ArrayLike, NDArray

from dimos.manipulation.manipulation_spec import (
    CommandResult,
    ExecutionResult,
    ExecutionStatus,
    ManipulationSpec,
    MoveResult,
    PlanningGroupInfo,
    PlanningGroupState,
    PlanResult,
)
from dimos.manipulation.planning.groups.utils import joint_state_to_ordered_positions
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from dimos.porcelain.dimos import Dimos


class MotionError(RuntimeError):
    """An SDK action failed; the original RPC result is available as ``result``.

    A timeout or uncertain outcome does not imply that motion has stopped.
    """

    def __init__(
        self,
        operation: str,
        result: PlanResult | ExecutionResult | MoveResult | CommandResult,
    ) -> None:
        self.operation = operation
        self.result = result
        super().__init__(f"{operation}: {result}")


def _vector(values: ArrayLike, size: int, name: str) -> NDArray[np.float64]:
    if np.iscomplexobj(values):
        raise ValueError(f"{name} must contain real numbers")
    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain {size} finite values in a one-dimensional sequence")
    return vector


class Arm:
    """A selected arm, borrowing a motion RPC proxy without owning its lifecycle.

    Use ``from_app`` for discovery. Moves block until physical completion and
    raise ``MotionError`` on failure. Explicit planning, preview, nonblocking
    execution, and cancellation remain on ``rpc``. The runtime has one shared
    pending plan and execution, so use one motion-commanding client per module.
    """

    def __init__(self, rpc: ManipulationSpec, info: PlanningGroupInfo) -> None:
        self.rpc = rpc
        self.info = info

    @classmethod
    def from_app(
        cls,
        app: Dimos,
        *,
        group: PlanningGroupID | None = None,
        instance_name: str | None = None,
    ) -> Arm:
        """Select the unique pose-capable group, or an explicit group and module.

        Discovery does not start a runtime or command motion. Gripperless arms
        are eligible. Missing or ambiguous selections raise ``ValueError``.
        """
        rpc = app.get_module(ManipulationSpec, instance_name=instance_name)
        groups = rpc.list_planning_groups()
        matches = [
            info
            for info in groups
            if info.tip_frame is not None and (group is None or info.id == group)
        ]
        if len(matches) != 1:
            available = [info.id for info in groups if info.tip_frame is not None]
            raise ValueError(
                f"Expected one pose-capable group for {group!r}; "
                f"available arms: {available}; all group IDs: {[info.id for info in groups]}"
            )
        return cls(rpc, matches[0])

    def state(self) -> PlanningGroupState:
        """Read the selected group's state, including available joint presets."""
        state = self.rpc.get_state().groups.get(self.info.id)
        if state is None:
            raise RuntimeError(f"State is unavailable for arm {self.info.id!r}")
        return state

    def joints(self) -> NDArray[np.float64]:
        """Read a fresh position array in ``info.joint_names`` order."""
        joints = self.state().joints
        if joints is None:
            raise RuntimeError(f"Joint state is unavailable for arm {self.info.id!r}")
        return joint_state_to_ordered_positions(joints, joint_names=self.info.joint_names).copy()

    def pose(self) -> PoseStamped:
        """Read the current end-effector pose; raise if telemetry is unavailable."""
        pose = self.state().end_effector_pose
        if pose is None:
            raise RuntimeError(f"End-effector pose is unavailable for arm {self.info.id!r}")
        return pose

    def move_joints(
        self,
        positions: ArrayLike,
        *,
        speed_scale: float | None = None,
        timeout: float | None = None,
    ) -> ExecutionResult:
        """Plan and move to positions in group joint order; angular units are radians."""
        values = _vector(positions, len(self.info.joint_names), "positions")
        target = JointState(name=list(self.info.joint_names), position=values.tolist())
        return self._move_joint_target("move_joints", target, speed_scale, timeout)

    def _move_joint_target(
        self,
        operation: str,
        target: JointState,
        speed_scale: float | None,
        timeout: float | None,
    ) -> ExecutionResult:
        plan = self.rpc.plan_to_joints({self.info.id: target}, speed_scale=speed_scale)
        return self._execute(operation, plan, timeout)

    def move_pose(
        self,
        position: ArrayLike,
        *,
        orientation: ArrayLike | None = None,
        speed_scale: float | None = None,
        timeout: float | None = None,
    ) -> ExecutionResult:
        """Plan and move to world XYZ in metres, with optional XYZW orientation.

        Omitting orientation preserves the current end-effector orientation.
        This targets an endpoint; use ``move_linear`` for a straight translation.
        """
        xyz = _vector(position, 3, "position")
        rotation = (
            self.pose().orientation
            if orientation is None
            else Quaternion(_vector(orientation, 4, "orientation"))
        )
        target = PoseStamped(frame_id="world", position=xyz, orientation=rotation)
        plan = self.rpc.plan_to_poses({self.info.id: target}, speed_scale=speed_scale)
        return self._execute("move_pose", plan, timeout)

    def _execute(self, operation: str, plan: PlanResult, timeout: float | None) -> ExecutionResult:
        if not plan.succeeded:
            raise MotionError(operation, plan)
        result = self.rpc.execute(blocking=True, timeout=timeout)
        if result.status is not ExecutionStatus.COMPLETED:
            raise MotionError(operation, result)
        return result

    def move_linear(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        *,
        check_collision: bool = False,
        speed_scale: float | None = None,
        timeout: float | None = None,
    ) -> MoveResult:
        """Execute the existing world-frame translation primitive, in metres.

        Collision checking remains opt-in, matching the RPC's default.
        """
        delta = _vector((dx, dy, dz), 3, "delta")
        result = self.rpc.move_linear(
            dx=float(delta[0]),
            dy=float(delta[1]),
            dz=float(delta[2]),
            planning_group=self.info.id,
            check_collision=check_collision,
            speed_scale=speed_scale,
            blocking=True,
            timeout=timeout,
        )
        if (
            not result.plan.succeeded
            or result.execution is None
            or result.execution.status is not ExecutionStatus.COMPLETED
        ):
            raise MotionError("move_linear", result)
        return result

    def set_gripper_position(self, position: float) -> CommandResult:
        """Set normalized gripper travel: 0 closed, 1 open."""
        if not math.isfinite(position) or not 0.0 <= position <= 1.0:
            raise ValueError("Gripper position must be finite and between 0 and 1")
        if not self.info.has_gripper:
            raise ValueError(f"Arm {self.info.id!r} has no gripper")
        result = self.rpc.set_gripper_position(position, planning_group=self.info.id)
        if not result.succeeded:
            raise MotionError("set_gripper_position", result)
        return result

    def open_gripper(self) -> CommandResult:
        """Open the gripper."""
        return self.set_gripper_position(1.0)

    def close_gripper(self) -> CommandResult:
        """Close the gripper."""
        return self.set_gripper_position(0.0)

    def home(
        self, *, speed_scale: float | None = None, timeout: float | None = None
    ) -> ExecutionResult:
        """Move to the configured home preset, shared with Viser."""
        return self.move_to_preset("home", speed_scale=speed_scale, timeout=timeout)

    def move_to_preset(
        self,
        name: str,
        *,
        speed_scale: float | None = None,
        timeout: float | None = None,
    ) -> ExecutionResult:
        """Move to an existing preset; fetch fresh values on every call."""
        presets = self.state().joint_presets
        if name not in presets:
            raise ValueError(f"Unknown joint preset {name!r}; available: {sorted(presets)}")
        return self._move_joint_target(
            f"move_to_preset({name!r})", presets[name], speed_scale, timeout
        )
