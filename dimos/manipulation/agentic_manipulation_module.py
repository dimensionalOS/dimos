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

"""Agent-facing manipulation primitive facades."""

from __future__ import annotations

import numpy as np

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.manipulation.agentic_manipulation_spec import (
    AgenticGraspGenSpec,
    ManipulationControlSpec,
)
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.grasping_msgs.GraspDebugMarkers import GraspDebugMarkers
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.experimental.object_scene_registration_spec import (
    ObjectSceneRegistrationSpec,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

SUPPORTED_RELATIVE_FRAME = "world"
SUPPORTED_GRASP_FRAME = "world"
DEFAULT_PREGRASP_OFFSET_M = 0.10
DEFAULT_LIFT_DISTANCE_M = 0.10


class AgenticManipulationModule(Module):
    """Expose stable manipulation primitives for agent/tool callers."""

    _manipulation: ManipulationControlSpec

    @skill
    def get_robot_state(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Get current robot state for manipulation.

        Args:
            robot_name: Robot to query (only needed for multi-arm setups).
        """
        return self._manipulation.get_robot_state(robot_name)

    @skill
    def move_to_joints(
        self, joints: str, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Move the robot to a target joint configuration.

        Args:
            joints: Comma-separated joint positions in radians, e.g. "0.1, -0.5, 1.2, 0.0, 0.3, -0.1".
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        return self._manipulation.move_to_joints(joints, robot_name)

    @skill
    def set_motion_speed(self, speed_scale: float) -> SkillResult[ManipulationSkillError]:
        """Set runtime manipulation motion speed for future motions.

        Args:
            speed_scale: Speed multiplier in the range `(0, 1]`. Use values below
                1.0 for slower, gentler motion.
        """
        if not self._manipulation.set_motion_speed(speed_scale):
            return SkillResult[ManipulationSkillError].fail(
                "INVALID_INPUT",
                "Motion speed scale must be greater than 0 and less than or equal to 1.",
            )
        return SkillResult[ManipulationSkillError].ok(
            f"Motion speed scale set to {speed_scale:.2f}x. Re-plan to apply it."
        )

    @skill
    def get_motion_speed(self) -> SkillResult[ManipulationSkillError]:
        """Get the current runtime manipulation motion speed scale."""
        speed_scale = self._manipulation.get_motion_speed()
        return SkillResult[ManipulationSkillError].ok(
            f"Current motion speed scale is {speed_scale:.2f}x.", speed_scale=speed_scale
        )

    @skill
    def open_gripper(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Open the robot gripper fully.

        Args:
            robot_name: Robot to control (only needed for multi-arm setups).
        """
        return self._manipulation.open_gripper(robot_name)

    @skill
    def close_gripper(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Close the robot gripper fully.

        Args:
            robot_name: Robot to control (only needed for multi-arm setups).
        """
        return self._manipulation.close_gripper(robot_name)


class AgenticGraspManipulationModule(AgenticManipulationModule):
    """Expose grasp-capable manipulation primitives for manual agent-facing demos."""

    _scene_registration: ObjectSceneRegistrationSpec
    _grasp_gen: AgenticGraspGenSpec
    grasp_debug_markers: Out[GraspDebugMarkers]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._cached_grasps: PoseArray | None = None
        self._cached_grasp_target: str | None = None
        self._cached_bbox_center: Vector3 | None = None
        self._cached_bbox_size: Vector3 | None = None

    @skill
    def scan_objects(self, target_name: str = "object") -> SkillResult[ManipulationSkillError]:
        """Register and summarize scene objects matching a target name.

        Args:
            target_name: Object prompt/name to register and summarize, e.g. "sphere".
        """
        self._scene_registration.set_prompts(text=[target_name])
        objects = self._scene_registration.get_registered_objects()
        matches = [obj for obj in objects if obj.name.lower() == target_name.lower()]
        if not matches:
            return SkillResult[ManipulationSkillError].fail(
                "OBJECT_NOT_DETECTED",
                f"No registered Grasp target named '{target_name}'.",
            )

        summary = "; ".join(
            f"{obj.name}:{obj.object_id} center=({obj.center.x:.3f}, "
            f"{obj.center.y:.3f}, {obj.center.z:.3f}) size=({obj.size.x:.3f}, "
            f"{obj.size.y:.3f}, {obj.size.z:.3f}) frame={obj.frame_id}"
            for obj in matches
        )
        return SkillResult[ManipulationSkillError].ok(
            f"Registered {len(matches)} Grasp target(s) for '{target_name}': {summary}",
            target_count=len(matches),
            object_ids=[obj.object_id for obj in matches],
        )

    @skill
    def generate_grasps(
        self,
        target_name: str = "object",
        object_id: str | None = None,
        filter_collisions: bool = False,
    ) -> SkillResult[ManipulationSkillError]:
        """Generate and cache grasp candidates for a registered object.

        Args:
            target_name: Registered object name to grasp, e.g. "sphere".
            object_id: Optional specific registered object id to use instead of target_name.
            filter_collisions: Include the scene pointcloud as context when available.
        """
        registered_center: Vector3 | None = None
        registered_size: Vector3 | None = None
        registered_frame: str | None = None
        if object_id is not None:
            pointcloud = self._scene_registration.get_object_pointcloud_by_object_id(object_id)
            target_label = object_id
            obj = self._scene_registration.get_object_by_object_id(object_id)
            if obj is not None:
                registered_center = obj.center
                registered_size = obj.size
                registered_frame = obj.frame_id
        else:
            objects = self._scene_registration.get_registered_objects()
            matches = [obj for obj in objects if obj.name.lower() == target_name.lower()]
            matched_object = matches[0] if matches else None
            resolved_object_id = matched_object.object_id if matched_object is not None else None
            if matched_object is not None:
                registered_center = matched_object.center
                registered_size = matched_object.size
                registered_frame = matched_object.frame_id
            pointcloud = (
                self._scene_registration.get_object_pointcloud_by_object_id(resolved_object_id)
                if resolved_object_id is not None
                else self._scene_registration.get_object_pointcloud_by_name(target_name)
            )
            target_label = resolved_object_id or target_name
            object_id = resolved_object_id
        if pointcloud is None:
            self._cached_grasps = None
            self._cached_grasp_target = None
            self._cached_bbox_center = None
            self._cached_bbox_size = None
            return SkillResult[ManipulationSkillError].fail(
                "OBJECT_NOT_DETECTED",
                f"No pointcloud found for Grasp target '{target_label}'. Run scan_objects first.",
            )

        scene_pointcloud = None
        if filter_collisions:
            scene_pointcloud = self._scene_registration.get_full_scene_pointcloud(
                exclude_object_id=object_id
            )

        pointcloud_centroid = _pointcloud_centroid(pointcloud)
        logger.info(
            "[GRASP-FRAME] target=%s object_id=%s bbox_frame=%s bbox_center=%s "
            "bbox_size=%s bbox_min=%s bbox_max=%s pointcloud_frame=%s "
            "pointcloud_centroid=%s point_count=%d",
            target_label,
            object_id,
            registered_frame,
            _format_vector(registered_center),
            _format_vector(registered_size),
            _format_bbox_min(registered_center, registered_size),
            _format_bbox_max(registered_center, registered_size),
            pointcloud.frame_id,
            _format_vector(pointcloud_centroid),
            len(pointcloud.points_f32()),
        )

        grasps = self._grasp_gen.generate_grasps(pointcloud, scene_pointcloud)
        if grasps is None or len(grasps) == 0:
            self._cached_grasps = None
            self._cached_grasp_target = None
            self._cached_bbox_center = None
            self._cached_bbox_size = None
            return SkillResult[ManipulationSkillError].fail(
                "GRASP_GENERATION_FAILED",
                f"No Grasp candidates generated for '{target_label}'.",
            )

        if grasps.header.frame_id != SUPPORTED_GRASP_FRAME:
            self._cached_grasps = None
            self._cached_grasp_target = None
            self._cached_bbox_center = None
            self._cached_bbox_size = None
            return SkillResult[ManipulationSkillError].fail(
                "INVALID_INPUT",
                f"Grasp candidates are in frame '{grasps.header.frame_id}', but execute_grasp "
                f"requires '{SUPPORTED_GRASP_FRAME}' candidates.",
            )

        first_candidate = grasps[0]
        logger.info(
            "[GRASP-FRAME] grasps_frame=%s candidate_count=%d first_candidate_pos=%s "
            "first_candidate_quat=(%.3f, %.3f, %.3f, %.3f)",
            grasps.header.frame_id,
            len(grasps),
            _format_vector(first_candidate.position),
            first_candidate.orientation.x,
            first_candidate.orientation.y,
            first_candidate.orientation.z,
            first_candidate.orientation.w,
        )
        logger.info(
            "[GRASP-FRAME] candidate_positions_world=%s",
            _format_candidate_positions(grasps),
        )

        self._cached_grasps = grasps
        self._cached_grasp_target = target_label
        self._cached_bbox_center = registered_center
        self._cached_bbox_size = registered_size
        self._publish_grasp_debug(
            GraspDebugMarkers(
                frame_id=grasps.header.frame_id,
                bbox_center=registered_center,
                bbox_size=registered_size,
                candidate_poses=list(grasps),
                label=f"{target_label} grasp candidates",
            )
        )
        return SkillResult[ManipulationSkillError].ok(
            f"Generated and cached {len(grasps)} Grasp candidate(s) for '{target_label}'.",
            candidate_count=len(grasps),
            target=target_label,
            pointcloud_frame=pointcloud.frame_id,
            pointcloud_centroid=_format_vector(pointcloud_centroid),
            first_candidate_position=_format_vector(first_candidate.position),
        )

    @skill
    def execute_grasp(
        self, candidate_index: int = 0, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Execute a cached grasp candidate without rescanning or regenerating grasps.

        Args:
            candidate_index: Index into the candidates cached by generate_grasps().
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        if self._cached_grasps is None or len(self._cached_grasps) == 0:
            return SkillResult[ManipulationSkillError].fail(
                "INVALID_STATE",
                "No cached Grasp candidates. Call generate_grasps(...) before execute_grasp(...).",
            )
        if candidate_index < 0 or candidate_index >= len(self._cached_grasps):
            return SkillResult[ManipulationSkillError].fail(
                "INVALID_INPUT",
                f"candidate_index {candidate_index} is outside cached range 0..{len(self._cached_grasps) - 1}.",
            )
        if self._cached_grasps.header.frame_id != SUPPORTED_GRASP_FRAME:
            return SkillResult[ManipulationSkillError].fail(
                "INVALID_INPUT",
                f"Cached Grasp candidates are in frame '{self._cached_grasps.header.frame_id}', "
                f"but execute_grasp requires '{SUPPORTED_GRASP_FRAME}' candidates.",
            )

        selected_index, pose = self._select_feasible_grasp(candidate_index, robot_name)
        if pose is None:
            return SkillResult[ManipulationSkillError].fail(
                "PLANNING_FAILED",
                f"No cached Grasp candidates from index {candidate_index} are reachable and collision-free.",
            )
        target = self._cached_grasp_target or "unknown target"
        self._publish_grasp_debug(
            GraspDebugMarkers(
                frame_id=self._cached_grasps.header.frame_id,
                bbox_center=self._cached_bbox_center,
                bbox_size=self._cached_bbox_size,
                candidate_poses=list(self._cached_grasps),
                selected_candidate_index=selected_index,
                pregrasp_pose=_offset_pose_for_approach(pose, DEFAULT_PREGRASP_OFFSET_M),
                final_pose=pose,
                label=f"selected grasp {selected_index} for {target}",
            )
        )
        sequence = self._execute_grasp_sequence(pose, robot_name)
        if not sequence.is_success():
            return sequence
        return SkillResult[ManipulationSkillError].ok(
            f"Executed Grasp candidate {selected_index} for '{target}'.",
            candidate_index=candidate_index,
            selected_candidate_index=selected_index,
            target=target,
        )

    @skill
    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Move the robot end-effector to a target pose.

        Args:
            x: Target X position in meters.
            y: Target Y position in meters.
            z: Target Z position in meters.
            roll: Target roll in radians (omit to preserve current orientation).
            pitch: Target pitch in radians (omit to preserve current orientation).
            yaw: Target yaw in radians (omit to preserve current orientation).
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        return self._manipulation.move_to_pose(x, y, z, roll, pitch, yaw, robot_name)

    @skill
    def move_relative(
        self,
        dx: float,
        dy: float,
        dz: float,
        frame: str = SUPPORTED_RELATIVE_FRAME,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Move the robot end-effector by a world-frame Cartesian delta.

        Args:
            dx: World-frame X translation in meters.
            dy: World-frame Y translation in meters.
            dz: World-frame Z translation in meters.
            frame: Relative motion frame. Round 1 supports only "world".
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        if frame != SUPPORTED_RELATIVE_FRAME:
            return SkillResult[ManipulationSkillError].fail(
                "INVALID_INPUT",
                f"Unsupported relative motion frame '{frame}'. Only 'world' is supported.",
            )
        current_pose = self._manipulation.get_ee_pose(robot_name)
        if current_pose is None:
            return SkillResult[ManipulationSkillError].fail(
                "NO_PRIOR_POSE",
                "Current end-effector pose is unavailable; cannot move relatively.",
            )
        return self.move_to_pose(
            current_pose.position.x + dx,
            current_pose.position.y + dy,
            current_pose.position.z + dz,
            robot_name=robot_name,
        )

    @skill
    def move_along_axis(
        self,
        axis: str,
        distance: float,
        frame: str = SUPPORTED_RELATIVE_FRAME,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Move the robot end-effector along one world-frame axis.

        Args:
            axis: Axis name: "x", "y", or "z".
            distance: Signed translation distance in meters.
            frame: Relative motion frame. Round 1 supports only "world".
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        if axis not in {"x", "y", "z"}:
            return SkillResult[ManipulationSkillError].fail(
                "INVALID_INPUT",
                f"Unsupported axis '{axis}'. Expected one of: x, y, z.",
            )
        dx = distance if axis == "x" else 0.0
        dy = distance if axis == "y" else 0.0
        dz = distance if axis == "z" else 0.0
        return self.move_relative(dx, dy, dz, frame=frame, robot_name=robot_name)

    @skill
    def go_home(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Move the robot to its configured home/observe pose.

        Args:
            robot_name: Robot to move (only needed for multi-arm setups).
        """
        return self._manipulation.go_home(robot_name)

    @skill
    def set_gripper(
        self, position: float, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]:
        """Set the robot gripper opening in meters.

        Args:
            position: Gripper opening in meters; 0.0 is closed.
            robot_name: Robot to control (only needed for multi-arm setups).
        """
        return self._manipulation.set_gripper(position, robot_name)

    def _execute_grasp_sequence(
        self, pose: Pose, robot_name: str | None
    ) -> SkillResult[ManipulationSkillError]:
        pregrasp_pose = _offset_pose_for_approach(pose, DEFAULT_PREGRASP_OFFSET_M)
        rpy = pose.orientation.to_euler()
        logger.info(
            "[GRASP-FRAME] execute_targets_world pregrasp=%s final=%s final_rpy=(%.3f, %.3f, %.3f)",
            _format_vector(pregrasp_pose.position),
            _format_vector(pose.position),
            rpy.x,
            rpy.y,
            rpy.z,
        )

        step = self.open_gripper(robot_name)
        if not step.is_success():
            return step
        step = self.move_to_pose(
            pregrasp_pose.position.x,
            pregrasp_pose.position.y,
            pregrasp_pose.position.z,
            rpy.x,
            rpy.y,
            rpy.z,
            robot_name,
        )
        if not step.is_success():
            return step
        step = self.move_to_pose(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            rpy.x,
            rpy.y,
            rpy.z,
            robot_name,
        )
        if not step.is_success():
            return step
        actual_pose = self._manipulation.get_ee_pose(robot_name)
        logger.info(
            "[GRASP-FRAME] final_target=%s actual_ee_after_final=%s",
            _format_vector(pose.position),
            _format_vector(actual_pose.position if actual_pose is not None else None),
        )
        step = self.close_gripper(robot_name)
        if not step.is_success():
            return step
        step = self.move_to_pose(
            pregrasp_pose.position.x,
            pregrasp_pose.position.y,
            pregrasp_pose.position.z,
            rpy.x,
            rpy.y,
            rpy.z,
            robot_name,
        )
        if not step.is_success():
            return step
        step = self.move_relative(0.0, 0.0, DEFAULT_LIFT_DISTANCE_M, robot_name=robot_name)
        if not step.is_success():
            return step
        return SkillResult[ManipulationSkillError].ok("Grasp execution sequence completed.")

    def _select_feasible_grasp(
        self, start_index: int, robot_name: str | None
    ) -> tuple[int, Pose | None]:
        assert self._cached_grasps is not None
        for candidate_index in range(start_index, len(self._cached_grasps)):
            pose = self._cached_grasps[candidate_index]
            if self._is_grasp_candidate_feasible(candidate_index, pose, robot_name):
                return candidate_index, pose
        return start_index, None

    def _is_grasp_candidate_feasible(
        self, candidate_index: int, pose: Pose, robot_name: str | None
    ) -> bool:
        pregrasp_pose = _offset_pose_for_approach(pose, DEFAULT_PREGRASP_OFFSET_M)
        logger.info(
            "[GRASP-FRAME] feasibility_targets_world candidate_index=%d pregrasp=%s final=%s",
            candidate_index,
            _format_vector(pregrasp_pose.position),
            _format_vector(pose.position),
        )
        if not self._manipulation.plan_to_pose(pregrasp_pose, robot_name):
            logger.info(
                "[GRASP-FRAME] candidate_index=%d rejected: pregrasp is not plan-feasible",
                candidate_index,
            )
            self._manipulation.reset()
            return False
        if not self._manipulation.plan_to_pose(pose, robot_name):
            logger.info(
                "[GRASP-FRAME] candidate_index=%d rejected: grasp pose is not plan-feasible",
                candidate_index,
            )
            self._manipulation.reset()
            return False
        logger.info("[GRASP-FRAME] candidate_index=%d selected as plan-feasible", candidate_index)
        return True

    def _publish_grasp_debug(self, markers: GraspDebugMarkers) -> None:
        debug_out = getattr(self, "grasp_debug_markers", None)
        if debug_out is not None:
            debug_out.publish(markers)


def _offset_pose_for_approach(pose: Pose, distance: float) -> Pose:
    """Offset away from the GPD candidate along candidate local -X."""
    approach_offset = pose.orientation.rotate_vector(Vector3(-distance, 0.0, 0.0))
    return Pose((pose.position + approach_offset, pose.orientation))


def _pointcloud_centroid(pointcloud: PointCloud2) -> Vector3 | None:
    points = pointcloud.points_f32()
    if len(points) == 0:
        return None
    centroid = np.mean(points, axis=0)
    return Vector3(float(centroid[0]), float(centroid[1]), float(centroid[2]))


def _format_vector(vector: Vector3 | None) -> str:
    if vector is None:
        return "None"
    return f"({vector.x:.3f}, {vector.y:.3f}, {vector.z:.3f})"


def _format_bbox_min(center: Vector3 | None, size: Vector3 | None) -> str:
    if center is None or size is None:
        return "None"
    return _format_vector(center - size * 0.5)


def _format_bbox_max(center: Vector3 | None, size: Vector3 | None) -> str:
    if center is None or size is None:
        return "None"
    return _format_vector(center + size * 0.5)


def _format_candidate_positions(grasps: PoseArray) -> str:
    return ", ".join(
        f"{index}:{_format_vector(pose.position)}" for index, pose in enumerate(grasps)
    )


agentic_manipulation = AgenticManipulationModule.blueprint
agentic_grasp_manipulation = AgenticGraspManipulationModule.blueprint
