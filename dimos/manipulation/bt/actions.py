"""Behavior-tree action nodes for current PickAndPlaceModule internals."""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

import py_trees
from py_trees.common import Status

from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.pick_and_place_module import PickAndPlaceModule

logger = setup_logger()


class ManipulationAction(py_trees.behaviour.Behaviour):
    """Base action node with shared blackboard keys."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name=name)
        self.module = module
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key="error_message", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="result_message", access=py_trees.common.Access.WRITE)


class ClearGraspState(ManipulationAction):
    """Clear per-attempt grasp state."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        for key in (
            "target_object",
            "object_pointcloud",
            "scene_pointcloud",
            "grasp_candidates",
            "grasp_index",
            "current_grasp",
            "pre_grasp_pose",
            "lift_pose",
            "has_object",
            "grasp_source",
        ):
            self.bb.register_key(key=key, access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        self.bb.target_object = None
        self.bb.object_pointcloud = None
        self.bb.scene_pointcloud = None
        self.bb.grasp_candidates = []
        self.bb.grasp_index = 0
        self.bb.current_grasp = None
        self.bb.pre_grasp_pose = None
        self.bb.lift_pose = None
        self.bb.has_object = False
        self.bb.grasp_source = ""
        return Status.SUCCESS


class ScanObjects(ManipulationAction):
    """Refresh perception detections."""

    def __init__(self, name: str, module: PickAndPlaceModule, min_duration: float) -> None:
        super().__init__(name, module)
        self.min_duration = min_duration
        self.bb.register_key(key="detections", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        try:
            self.module.refresh_obstacles(self.min_duration)
            self.bb.detections = list(self.module._detection_snapshot)
        except Exception as e:
            self.bb.error_message = f"Error: Scan failed - {e}"
            logger.error(f"[ScanObjects] {e}")
            return Status.FAILURE
        return Status.SUCCESS if self.bb.detections else Status.FAILURE


class ClearPerceptionObstacles(ManipulationAction):
    """Clear perception obstacles from the planning world."""

    def update(self) -> Status:
        try:
            result = self.module.clear_perception_obstacles()
            logger.info(f"[ClearPerceptionObstacles] {result}")
        except Exception as e:
            self.bb.error_message = f"Error: Clear perception obstacles failed - {e}"
            logger.error(f"[ClearPerceptionObstacles] {e}")
            return Status.FAILURE
        return Status.SUCCESS


class ResetRobot(ManipulationAction):
    """Cancel active motion if possible and reset the module state."""

    def update(self) -> Status:
        try:
            self.module.cancel()
        except Exception:
            logger.warning("[ResetRobot] cancel failed", exc_info=True)
        result = self.module.reset()
        if isinstance(result, str) and result.startswith("Error:"):
            self.bb.error_message = result
            return Status.FAILURE
        return Status.SUCCESS


class CancelMotion(ManipulationAction):
    """Best-effort cancellation of active motion."""

    def update(self) -> Status:
        try:
            self.module.cancel()
        except Exception:
            logger.warning("[CancelMotion] cancel failed", exc_info=True)
        return Status.SUCCESS


class FindObject(ManipulationAction):
    """Find the target object in the module's detection snapshot."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="object_name", access=py_trees.common.Access.READ)
        self.bb.register_key(key="object_id", access=py_trees.common.Access.READ)
        self.bb.register_key(key="target_object", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        det = self.module._find_object_in_detections(self.bb.object_name, self.bb.object_id)
        if det is None:
            self.bb.error_message = f"Error: Object '{self.bb.object_name}' not found"
            return Status.FAILURE
        self.bb.target_object = det
        return Status.SUCCESS


class GetObjectPointcloud(ManipulationAction):
    """Best-effort object point-cloud fetch through OSR RPCs."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="target_object", access=py_trees.common.Access.READ)
        self.bb.register_key(key="object_pointcloud", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        target = self.bb.target_object
        obj_id = getattr(target, "object_id", None)
        obj_name = getattr(target, "name", "")
        try:
            if obj_id:
                pc = self.module.get_rpc_calls(
                    "ObjectSceneRegistrationModule.get_object_pointcloud_by_object_id"
                )(obj_id)
            else:
                pc = self.module.get_rpc_calls(
                    "ObjectSceneRegistrationModule.get_object_pointcloud_by_name"
                )(obj_name)
        except Exception as e:
            logger.warning(f"[GetObjectPointcloud] unavailable: {e}")
            self.bb.error_message = f"Error: Object pointcloud unavailable - {e}"
            return Status.FAILURE

        if pc is None:
            self.bb.error_message = f"Error: No pointcloud for '{obj_name}'"
            return Status.FAILURE
        self.bb.object_pointcloud = pc
        return Status.SUCCESS


class GetScenePointcloud(ManipulationAction):
    """Best-effort scene point-cloud fetch. Missing scene cloud is non-fatal."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="target_object", access=py_trees.common.Access.READ)
        self.bb.register_key(key="scene_pointcloud", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        try:
            obj_id = getattr(self.bb.target_object, "object_id", None)
            self.bb.scene_pointcloud = self.module.get_rpc_calls(
                "ObjectSceneRegistrationModule.get_full_scene_pointcloud"
            )(exclude_object_id=obj_id)
        except Exception as e:
            logger.warning(f"[GetScenePointcloud] unavailable: {e}")
            self.bb.scene_pointcloud = None
        return Status.SUCCESS


class GenerateGrasps(ManipulationAction):
    """Generate GraspGen candidates from object and scene point clouds."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="object_pointcloud", access=py_trees.common.Access.READ)
        self.bb.register_key(key="scene_pointcloud", access=py_trees.common.Access.READ)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="grasp_index", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="grasp_source", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        result = self.module.generate_grasps(
            self.bb.object_pointcloud,
            getattr(self.bb, "scene_pointcloud", None),
        )
        poses = list(getattr(result, "poses", []) or [])
        if not poses:
            self.bb.error_message = "Error: GraspGen returned no grasps"
            return Status.FAILURE
        self.bb.grasp_candidates = poses
        self.bb.grasp_index = 0
        self.bb.grasp_source = "graspgen"
        logger.info(f"[GenerateGrasps] Generated {len(poses)} GraspGen candidates")
        return Status.SUCCESS


class GenerateHeuristicGrasps(ManipulationAction):
    """Fallback top-down grasps above the detection center."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="target_object", access=py_trees.common.Access.READ)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="grasp_index", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="grasp_source", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        target = self.bb.target_object
        center = getattr(target, "center", None)
        if center is None:
            self.bb.error_message = "Error: No detection center for heuristic grasp"
            return Status.FAILURE
        z_offsets = getattr(
            self.module.config,
            "bt_heuristic_grasp_z_offsets",
            (0.08, 0.12, 0.16),
        )
        min_heuristic_z = getattr(
            self.module.config,
            "bt_min_heuristic_grasp_z",
            self.module.config.bt_min_grasp_z,
        )
        candidates: list[Pose] = []
        seen_z: set[float] = set()
        for z_offset in z_offsets:
            z = max(
                center.z + z_offset,
                self.module.config.bt_min_grasp_z,
                min_heuristic_z,
            )
            z_key = round(z, 2)
            if z_key in seen_z:
                continue
            seen_z.add(z_key)
            candidates.append(
                Pose(
                    Vector3(center.x, center.y, z),
                    Quaternion.from_euler(Vector3(0.0, math.pi, 0.0)),
                )
            )
        self.bb.grasp_candidates = candidates
        self.bb.grasp_index = 0
        self.bb.grasp_source = "heuristic"
        zs = ", ".join(f"{pose.position.z:.3f}" for pose in candidates)
        logger.info(
            f"[GenerateHeuristicGrasps] {len(candidates)} top-down grasps above "
            f"({center.x:.3f}, {center.y:.3f}, {center.z:.3f}); z=[{zs}]"
        )
        return Status.SUCCESS


class FilterGraspWorkspace(ManipulationAction):
    """Filter candidates by simple workspace constraints."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.READ)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="grasp_index", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        cfg = self.module.config
        candidates = list(self.bb.grasp_candidates)
        filtered: list[Pose] = []
        cos_threshold = math.cos(cfg.bt_max_approach_angle)

        for pose in candidates:
            if pose.position.z < cfg.bt_min_grasp_z:
                logger.info(
                    "[FilterGraspWorkspace] rejected by z: "
                    f"z={pose.position.z:.3f} min={cfg.bt_min_grasp_z:.3f}"
                )
                continue
            distance = pose.position.magnitude()
            if distance > cfg.bt_max_grasp_distance:
                logger.info(
                    "[FilterGraspWorkspace] rejected by distance: "
                    f"distance={distance:.3f} max={cfg.bt_max_grasp_distance:.3f}"
                )
                continue
            qx, qy = pose.orientation.x, pose.orientation.y
            approach_z = 1.0 - 2.0 * (qx * qx + qy * qy)
            if approach_z > -cos_threshold:
                logger.info(
                    "[FilterGraspWorkspace] rejected by approach angle: "
                    f"approach_z={approach_z:.3f} threshold={-cos_threshold:.3f}"
                )
                continue
            filtered.append(pose)

        if not filtered:
            self.bb.error_message = "Error: No grasp candidates passed workspace filtering"
            return Status.FAILURE
        self.bb.grasp_candidates = filtered
        self.bb.grasp_index = 0
        logger.info(f"[FilterGraspWorkspace] {len(filtered)}/{len(candidates)} passed")
        return Status.SUCCESS


class SelectNextGrasp(ManipulationAction):
    """Select the next candidate."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="grasp_candidates", access=py_trees.common.Access.READ)
        self.bb.register_key(key="grasp_index", access=py_trees.common.Access.READ)
        self.bb.register_key(key="grasp_index", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="current_grasp", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        candidates: list[Pose] = self.bb.grasp_candidates
        idx: int = self.bb.grasp_index
        if idx >= len(candidates):
            self.bb.error_message = "Error: All grasp candidates exhausted"
            return Status.FAILURE
        self.bb.current_grasp = candidates[idx]
        self.bb.grasp_index = idx + 1
        logger.info(f"[SelectNextGrasp] Candidate {idx + 1}/{len(candidates)}")
        return Status.SUCCESS


class ComputePreGrasp(ManipulationAction):
    """Compute the pre-grasp pose."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="current_grasp", access=py_trees.common.Access.READ)
        self.bb.register_key(key="pre_grasp_pose", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="robot_name", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        robot = self.module._get_robot(getattr(self.bb, "robot_name", None))
        robot_config = robot[2] if robot is not None else None
        offset = getattr(robot_config, "pre_grasp_offset", 0.10)
        self.bb.pre_grasp_pose = self.module._compute_pre_grasp_pose(
            self.bb.current_grasp, offset
        )
        p = self.bb.pre_grasp_pose.position
        logger.info(
            f"[ComputePreGrasp] offset={offset:.3f} pose=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
        )
        return Status.SUCCESS


class PlanToPose(ManipulationAction):
    """Plan to a pose stored on the blackboard."""

    def __init__(self, name: str, module: PickAndPlaceModule, pose_key: str) -> None:
        super().__init__(name, module)
        self.pose_key = pose_key
        self.bb.register_key(key=pose_key, access=py_trees.common.Access.READ)
        self.bb.register_key(key="robot_name", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        pose = getattr(self.bb, self.pose_key)
        robot_name = getattr(self.bb, "robot_name", None)
        p = pose.position
        logger.info(
            f"[PlanToPose] Planning to {self.pose_key}: "
            f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
        )
        if self.module.get_state() == "FAULT":
            self.module.reset()
        if self.module.plan_to_pose(pose, robot_name):
            logger.info(f"[PlanToPose] Planning to {self.pose_key} succeeded")
            return Status.SUCCESS
        self.bb.error_message = f"Error: Planning to {self.pose_key} failed"
        logger.info(f"[PlanToPose] Planning to {self.pose_key} failed")
        return Status.FAILURE


class ExecuteTrajectory(ManipulationAction):
    """Preview, execute, and wait for completion using the current module helper."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="robot_name", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        err = self.module._preview_execute_wait(getattr(self.bb, "robot_name", None))
        if err:
            self.bb.error_message = err
            return Status.FAILURE
        return Status.SUCCESS


class SetGripper(ManipulationAction):
    """Set gripper and wait for settle."""

    def __init__(
        self,
        name: str,
        module: PickAndPlaceModule,
        position: float,
        settle_time: float,
    ) -> None:
        super().__init__(name, module)
        self.position = position
        self.settle_time = settle_time
        self.bb.register_key(key="robot_name", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        if not self.module._set_gripper_position(self.position, getattr(self.bb, "robot_name", None)):
            self.bb.error_message = "Error: Gripper command failed"
            return Status.FAILURE
        time.sleep(self.settle_time)
        return Status.SUCCESS


class ComputeLiftPose(ManipulationAction):
    """Compute a straight-up lift from the intended grasp pose."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="current_grasp", access=py_trees.common.Access.READ)
        self.bb.register_key(key="lift_pose", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="robot_name", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        base = self.bb.current_grasp
        self.bb.lift_pose = Pose(
            Vector3(base.position.x, base.position.y, base.position.z + self.module.config.bt_lift_height),
            base.orientation,
        )
        p = self.bb.lift_pose.position
        logger.info(f"[ComputeLiftPose] pose=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
        return Status.SUCCESS


class StorePickPosition(ManipulationAction):
    """Remember successful pick position for place_back."""

    def __init__(self, name: str, module: PickAndPlaceModule) -> None:
        super().__init__(name, module)
        self.bb.register_key(key="current_grasp", access=py_trees.common.Access.READ)

    def update(self) -> Status:
        self.module._last_pick_position = self.bb.current_grasp.position
        return Status.SUCCESS


class ComputePlacePose(ManipulationAction):
    """Compute place and pre-place poses."""

    def __init__(self, name: str, module: PickAndPlaceModule, x: float, y: float, z: float) -> None:
        super().__init__(name, module)
        self.x = x
        self.y = y
        self.z = z
        self.bb.register_key(key="place_pose", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="pre_place_pose", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        place_pose = Pose(
            Vector3(self.x, self.y, self.z),
            Quaternion.from_euler(Vector3(0.0, math.pi, 0.0)),
        )
        self.bb.place_pose = place_pose
        offset = getattr(self.module.config, "pre_grasp_offset", 0.10)
        self.bb.pre_place_pose = self.module._compute_pre_grasp_pose(
            place_pose, offset
        )
        return Status.SUCCESS


class SetResultMessage(ManipulationAction):
    """Write final result message."""

    def __init__(self, name: str, module: PickAndPlaceModule, message: str) -> None:
        super().__init__(name, module)
        self.message = message

    def update(self) -> Status:
        source = ""
        try:
            if self.bb.grasp_source:
                source = f" using {self.bb.grasp_source} grasp"
        except (AttributeError, KeyError):
            pass
        self.bb.result_message = self.message.format(grasp_source=source)
        return Status.SUCCESS
