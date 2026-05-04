"""Behavior-tree condition nodes for current PickAndPlaceModule internals."""

from __future__ import annotations

from typing import TYPE_CHECKING

import py_trees
from py_trees.common import Status

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.pick_and_place_module import PickAndPlaceModule

logger = setup_logger()


class ManipulationCondition(py_trees.behaviour.Behaviour):
    """Base condition node."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name=name)
        self.module = module
        self.bb = self.attach_blackboard_client(name=self.name)


class HasDetections(ManipulationCondition):
    """Check whether the module has snapshotted detections."""

    def update(self) -> Status:
        return Status.SUCCESS if self.module._detection_snapshot else Status.FAILURE


class HasTargetDetection(ManipulationCondition):
    """Check whether the requested target is already in the detection snapshot."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="object_name", access=py_trees.common.Access.READ)
        self.bb.register_key(key="object_id", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        object_name = getattr(self.bb, "object_name", "")
        object_id = getattr(self.bb, "object_id", None)
        if not object_name and not object_id:
            return Status.FAILURE
        return (
            Status.SUCCESS
            if self.module._find_object_in_detections(object_name, object_id)
            else Status.FAILURE
        )


class RobotIsHealthy(ManipulationCondition):
    """Check basic module and robot readiness."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="robot_name", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        robot_name = getattr(self.bb, "robot_name", None)
        if self.module.get_robot_info(robot_name) is None:
            return Status.FAILURE
        if self.module.get_state() in {"EXECUTING", "FAULT"}:
            return Status.FAILURE
        return Status.SUCCESS


class GripperHasObject(ManipulationCondition):
    """Verify grasp by checking for a partially closed gripper."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="robot_name", access=py_trees.common.Access.READ)
        self.bb.register_key(key="has_object", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="error_message", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        pos = self.module.get_gripper(getattr(self.bb, "robot_name", None))
        if pos is None:
            self.bb.error_message = "Error: Gripper state unavailable"
            self.bb.has_object = False
            return Status.FAILURE
        opening = float(pos)
        min_opening = self.module.config.bt_gripper_grasp_threshold
        max_opening = self.module.config.bt_gripper_grasp_max_open_position
        if max_opening is None:
            max_opening = (
                self.module.config.bt_gripper_open_position
                * self.module.config.bt_gripper_grasp_max_open_fraction
            )
        min_closure = self.module.config.bt_gripper_grasp_min_closure
        if min_closure is not None:
            open_reference = self.module.config.bt_gripper_grasp_open_reference
            if open_reference is None:
                open_reference = self.module.config.bt_gripper_open_position
            max_opening = min(max_opening, open_reference - min_closure)

        logger.info(
            f"[{self.name}] gripper opening={opening:.4f}m "
            f"expected range=({min_opening:.4f}, {max_opening:.4f})m"
        )

        if min_opening < opening < max_opening:
            self.bb.has_object = True
            return Status.SUCCESS
        self.bb.has_object = False
        if opening >= max_opening:
            self.bb.error_message = (
                "Error: Grasp verification failed - gripper still open "
                f"({opening:.4f}m >= {max_opening:.4f}m)"
            )
        else:
            self.bb.error_message = (
                "Error: Grasp verification failed - gripper empty "
                f"({opening:.4f}m <= {min_opening:.4f}m)"
            )
        return Status.FAILURE


class VerifyHoldAfterLift(GripperHasObject):
    """Re-check gripper after lift."""
