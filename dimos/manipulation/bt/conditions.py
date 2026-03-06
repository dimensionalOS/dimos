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

"""BT condition leaf nodes for pick-and-place orchestration.

Condition nodes are instant checks (never return RUNNING).  They test
predicates on blackboard state or robot hardware and return SUCCESS/FAILURE.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import py_trees
from py_trees.common import Status

from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.bt.pick_place_module import PickPlaceModule
    from dimos.msgs.geometry_msgs import Pose

logger = setup_logger()

class ManipulationCondition(py_trees.behaviour.Behaviour):
    """Base class for all BT condition nodes."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name=name)
        self.module = module
        self.bb = self.attach_blackboard_client(name=self.name)

class HasDetections(ManipulationCondition):
    """Check whether the blackboard has a non-empty detection list."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="detections", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        try:
            detections = self.bb.detections
        except KeyError:
            return Status.FAILURE

        if detections:
            return Status.SUCCESS
        return Status.FAILURE

class GripperHasObject(ManipulationCondition):
    """Verify grasp by checking gripper position against threshold.

    Position above threshold = object present (fingers stopped on object).
    Sets ``bb.has_object`` accordingly.
    """

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="error_message", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="has_object", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        try:
            position = self.module.get_rpc_calls("PickAndPlaceModule.get_gripper")()
            pos = float(position)
        except Exception as e:
            logger.error(f"[GripperHasObject] get_gripper RPC failed: {e}")
            self.bb.error_message = f"Error: Gripper query failed — {e}"
            self.bb.has_object = False
            return Status.FAILURE

        threshold = self.module.config.gripper_grasp_threshold
        # No upper bound needed — called right after CloseGripper,
        # unlike ProbeGripperState which guards against open gripper.
        has_object = pos > threshold

        if has_object:
            logger.info(
                f"[GripperHasObject] Object detected (position={pos:.4f}m > "
                f"threshold={threshold:.4f}m)"
            )
            self.bb.has_object = True
            return Status.SUCCESS

        logger.warning(
            f"[GripperHasObject] No object (position={pos:.4f}m <= "
            f"threshold={threshold:.4f}m)"
        )
        self.bb.has_object = False
        self.bb.error_message = "Error: Grasp verification failed — gripper empty"
        return Status.FAILURE

class RobotIsHealthy(ManipulationCondition):
    """Check that the robot is ready for new commands.

    FAIL if trajectory state is EXECUTING, ABORTED, or FAULT.
    FAIL if ``get_robot_info()`` returns None.
    """

    def update(self) -> Status:
        try:
            status = self.module.get_rpc_calls("PickAndPlaceModule.get_trajectory_status")()
        except Exception as e:
            logger.error(f"[RobotIsHealthy] get_trajectory_status failed: {e}")
            return Status.FAILURE

        if status is not None:
            state_val = int(status.get("state", -1)) if isinstance(status, dict) else int(status)
            if state_val in (TrajectoryState.EXECUTING, TrajectoryState.ABORTED, TrajectoryState.FAULT):
                logger.warning(f"[RobotIsHealthy] Bad state: {TrajectoryState(state_val).name}")
                return Status.FAILURE

        try:
            info = self.module.get_rpc_calls("PickAndPlaceModule.get_robot_info")()
        except Exception as e:
            logger.error(f"[RobotIsHealthy] get_robot_info failed: {e}")
            return Status.FAILURE

        if info is None:
            logger.warning("[RobotIsHealthy] get_robot_info returned None")
            return Status.FAILURE

        return Status.SUCCESS

class HasObject(ManipulationCondition):
    """Check ``bb.has_object`` — SUCCESS if True, FAILURE otherwise."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="has_object", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        try:
            if self.bb.has_object:
                return Status.SUCCESS
        except (KeyError, AttributeError):
            pass
        return Status.FAILURE

class VerifyReachedPose(ManipulationCondition):
    """Verify the EE has reached a target pose within tolerance."""

    def __init__(
        self,
        name: str,
        module: PickPlaceModule,
        pose_key: str,
        pos_tol: float | None = None,
        rot_tol: float | None = None,
    ) -> None:
        super().__init__(name, module)
        self.pose_key = pose_key
        self.pos_tol = pos_tol
        self.rot_tol = rot_tol
        self.bb.register_key(key=pose_key, access=py_trees.common.Access.READ)
        self.bb.register_key(key="error_message", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        pos_tol = self.pos_tol if self.pos_tol is not None else self.module.config.pose_position_tolerance
        rot_tol = self.rot_tol if self.rot_tol is not None else self.module.config.pose_orientation_tolerance

        try:
            target: Pose = getattr(self.bb, self.pose_key)
        except (KeyError, AttributeError):
            self.bb.error_message = f"Error: No target pose '{self.pose_key}' on blackboard"
            return Status.FAILURE

        try:
            actual: Pose | None = self.module.get_rpc_calls("PickAndPlaceModule.get_ee_pose")()
        except Exception as e:
            self.bb.error_message = f"Error: get_ee_pose failed — {e}"
            return Status.FAILURE

        if actual is None:
            self.bb.error_message = "Error: get_ee_pose returned None"
            return Status.FAILURE

        pos_err = target.position.distance(actual.position)
        if pos_err > pos_tol:
            self.bb.error_message = (
                f"Error: Position error {pos_err:.4f}m > tolerance {pos_tol:.4f}m"
            )
            logger.warning(f"[VerifyReachedPose:{self.pose_key}] {self.bb.error_message}")
            return Status.FAILURE

        angle_err = target.orientation.angular_distance(actual.orientation)
        if angle_err > rot_tol:
            self.bb.error_message = (
                f"Error: Orientation error {angle_err:.4f}rad > tolerance {rot_tol:.4f}rad"
            )
            logger.warning(f"[VerifyReachedPose:{self.pose_key}] {self.bb.error_message}")
            return Status.FAILURE

        logger.info(
            f"[VerifyReachedPose:{self.pose_key}] OK — "
            f"pos_err={pos_err:.4f}m, angle_err={angle_err:.4f}rad"
        )
        return Status.SUCCESS

class VerifyHoldAfterLift(ManipulationCondition):
    """Re-check gripper after lift to confirm object wasn't dropped."""

    def __init__(self, name: str, module: PickPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="error_message", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="has_object", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        try:
            position = self.module.get_rpc_calls("PickAndPlaceModule.get_gripper")()
            pos = float(position)
        except Exception as e:
            logger.error(f"[VerifyHoldAfterLift] get_gripper failed: {e}")
            self.bb.error_message = f"Error: Post-lift gripper check failed — {e}"
            self.bb.has_object = False
            return Status.FAILURE

        threshold = self.module.config.gripper_grasp_threshold
        if pos > threshold:
            logger.info("[VerifyHoldAfterLift] Object still held after lift")
            return Status.SUCCESS

        logger.warning("[VerifyHoldAfterLift] Object dropped during lift")
        self.bb.has_object = False
        self.bb.error_message = "Error: Object dropped during lift"
        return Status.FAILURE
