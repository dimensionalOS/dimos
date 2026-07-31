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

"""Pick-and-place manipulation module.

Extends ManipulationModule with perception integration and long-horizon skills:
- Perception: objects port, obstacle monitor, scan_objects, get_scene_info
- @rpc: generate_grasps (GraspGen Docker), refresh_obstacles, perception status
- @skill: pick, place, place_back, pick_and_place, scan_objects, get_scene_info
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from enum import Enum
import math
import threading
import time
from typing import TYPE_CHECKING, Any, Literal

import numpy as np
from pydantic import Field, FiniteFloat, model_validator

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
)
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.perception.detection.type.detection3d.object import (
    Object as DetObject,
)
from dimos.perception.object_scene_registration_spec import ObjectSceneRegistrationSpec
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import offset_distance

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseArray import PoseArray
    from dimos.msgs.sensor_msgs.JointState import JointState
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

logger = setup_logger()

# Beyond this XY distance from the base, the arm cannot reach both high and far,
# so pre-grasp/pre-place offsets are reduced.
_FAR_REACH_XY_THRESHOLD = 0.7

# Beyond this XY distance, the occlusion inset is increased so the grasp
# targets closer to the true center rather than the front surface.
_FAR_OCCLUSION_XY_THRESHOLD = 0.8

# Objects taller than this are grasped in the upper third to avoid
# plunging deep and colliding with the object body.
_TALL_OBJECT_MIN_HEIGHT = 0.06


class GraspVerificationConfig(BaseConfig):
    """Robot-specific gripper closure verification settings."""

    enabled: bool = False
    open_position: FiniteFloat = 0.85
    closed_position: FiniteFloat = 0.0
    held_threshold: FiniteFloat = 0.02
    timeout: FiniteFloat = Field(default=2.0, gt=0.0)
    poll_interval: FiniteFloat = Field(default=0.05, gt=0.0)

    @model_validator(mode="after")
    def _validate_threshold(self) -> GraspVerificationConfig:
        low = min(self.open_position, self.closed_position)
        high = max(self.open_position, self.closed_position)
        if self.open_position == self.closed_position:
            raise ValueError("gripper open_position and closed_position must differ")
        if not low < self.held_threshold < high:
            raise ValueError("held_threshold must lie between open_position and closed_position")
        if self.poll_interval > self.timeout:
            raise ValueError("poll_interval must not exceed timeout")
        return self


class PickAndPlaceModuleConfig(ManipulationModuleConfig):
    """Configuration for PickAndPlaceModule."""

    heuristic_grasp_fallback: bool = False
    planning_frame: str = "world"
    max_object_pointcloud_age: FiniteFloat = Field(default=10.0, gt=0.0)
    max_grasp_candidates_to_check: int = Field(default=5, gt=0)
    grasp_pre_grasp_offset: FiniteFloat | None = Field(default=None, gt=0.0)
    grasp_retreat_offset: FiniteFloat | None = Field(default=None, gt=0.0)
    grasp_approach_vector: tuple[FiniteFloat, FiniteFloat, FiniteFloat] = (0.0, 0.0, -1.0)
    pick_suppress_all_object_obstacles: bool = False
    grasp_verification: GraspVerificationConfig = Field(default_factory=GraspVerificationConfig)

    @model_validator(mode="after")
    def _validate_grasp_pipeline(self) -> PickAndPlaceModuleConfig:
        if not self.planning_frame.strip():
            raise ValueError("planning_frame must not be empty")
        vector = np.asarray(self.grasp_approach_vector, dtype=float)
        if not np.isclose(np.linalg.norm(vector), 1.0, atol=1e-6):
            raise ValueError("grasp_approach_vector must be a unit vector")
        return self


class _PickPhase(str, Enum):
    RESOLVE = "RESOLVE"
    PROPOSE = "PROPOSE"
    SELECT = "SELECT"
    PREPARE = "PREPARE"
    APPROACH = "APPROACH"
    GRASP = "GRASP"
    CLOSE = "CLOSE"
    VERIFY = "VERIFY"
    RETREAT = "RETREAT"
    DONE = "DONE"


class _CandidateRejection(str, Enum):
    INVALID = "invalid"
    PRE_GRASP_INFEASIBLE = "pre_grasp_infeasible"
    GRASP_INFEASIBLE = "grasp_infeasible"
    RETREAT_INFEASIBLE = "retreat_infeasible"


@dataclass(frozen=True)
class _FeasibleGrasp:
    candidate: GraspCandidate
    rank: int
    pre_grasp_pose: Pose
    retreat_pose: Pose


@dataclass(frozen=True)
class _GraspVerification:
    held: bool
    position: float | None
    detail: str


@dataclass
class _PickTransaction:
    object_id: str = ""
    object_name: str = ""
    proposal_source: Literal["grasp_provider", "heuristic"] = "grasp_provider"
    phase: _PickPhase = _PickPhase.RESOLVE
    selected: _FeasibleGrasp | None = None
    rejections: Counter[str] = field(default_factory=Counter)
    gripper_closed: bool = False


class _PickPipelineError(RuntimeError):
    def __init__(self, code: ManipulationSkillError, message: str) -> None:
        super().__init__(message)
        self.code = code


class PickAndPlaceModule(ManipulationModule):
    """Manipulation module with perception integration and pick-and-place skills.

    Extends ManipulationModule with:
    - Perception: objects port, obstacle monitor, scan_objects, get_scene_info
    - @rpc: generate_grasps (GraspGen Docker), refresh_obstacles, perception status
    - @skill: pick, place, place_back, pick_and_place, scan_objects, get_scene_info
    """

    config: PickAndPlaceModuleConfig
    _object_scene: ObjectSceneRegistrationSpec | None = None
    _grasp_generator: GraspGenSpec | None = None

    # Input: Objects from perception (for obstacle integration)
    objects: In[list[DetObject]]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # Last pick pose: stored during pick so place_back() can return the object
        self._last_pick_pose: Pose | None = None

        # Snapshotted detections from the last scan_objects/refresh call.
        # The live detection cache is volatile (labels change every frame),
        # so pick/place use this stable snapshot instead.
        self._detection_snapshot: list[DetObject] = []
        self._pick_guard = threading.Lock()

    @rpc
    def start(self) -> None:
        """Start the pick-and-place module (adds perception subscriptions)."""
        super().start()

        # Subscribe to objects port for perception obstacle integration
        if self.objects is not None:
            self.objects.observable().subscribe(self._on_objects)
            logger.info("Subscribed to objects port (async)")

        # Start obstacle monitor for perception integration
        if self._world_monitor is not None:
            self._world_monitor.start_obstacle_monitor()

        logger.info("PickAndPlaceModule started")

    def _on_objects(self, objects: list[DetObject]) -> None:
        """Callback when objects received from perception (runs on RxPY thread pool)."""
        try:
            if self._world_monitor is not None:
                self._world_monitor.on_objects(objects)
        except Exception as e:
            logger.error(f"Exception in _on_objects: {e}")

    @rpc
    def refresh_obstacles(self, min_duration: float = 0.0) -> list[dict[str, Any]]:
        """Refresh perception obstacles. Returns the list of obstacles added.

        Also snapshots the current detections so pick/place can use stable labels.
        """
        if self._world_monitor is None:
            return []
        result = self._world_monitor.refresh_obstacles(min_duration)
        # Snapshot detections at refresh time — the live cache is volatile
        self._detection_snapshot = self._world_monitor.get_cached_objects()
        logger.info(f"Detection snapshot: {[d.name for d in self._detection_snapshot]}")
        return result

    @skill
    def clear_perception_obstacles(self) -> SkillResult[ManipulationSkillError]:
        """Clear all perception obstacles from the planning world.

        Use this when the planner reports COLLISION_AT_START — detected objects
        may overlap the robot's current position and block planning.
        """
        if self._world_monitor is None:
            return SkillResult.fail(
                "WORLD_MONITOR_UNAVAILABLE",
                "No world monitor available",
            )
        count = self._world_monitor.clear_perception_obstacles()
        self._detection_snapshot = []
        return SkillResult.ok(f"Cleared {count} perception obstacle(s) from planning world")

    @rpc
    def get_perception_status(self) -> dict[str, int]:
        """Get perception obstacle status (cached/added counts)."""
        if self._world_monitor is None:
            return {"cached": 0, "added": 0}
        return self._world_monitor.get_perception_status()

    @rpc
    def list_cached_detections(self) -> list[dict[str, Any]]:
        """List cached detections from perception."""
        if self._world_monitor is None:
            return []
        return self._world_monitor.list_cached_detections()

    @rpc
    def list_added_obstacles(self) -> list[dict[str, Any]]:
        """List perception obstacles currently in the planning world."""
        if self._world_monitor is None:
            return []
        return self._world_monitor.list_added_obstacles()

    @rpc
    def generate_grasps(
        self,
        pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2 | None = None,
    ) -> PoseArray | None:
        """Generate grasp poses for the given point cloud via GraspGen module."""
        raise NotImplementedError(
            "GraspGen Docker support removed; see issue #1266 for re-implementation as NativeModule subclass"
        )

    def _compute_pre_grasp_pose(
        self,
        grasp_pose: Pose,
        offset: float = 0.10,
        approach_vector: Vector3 | None = None,
    ) -> Pose:
        """Compute a pre-grasp pose offset along the approach direction (local -Z).

        Args:
            grasp_pose: The final grasp pose
            offset: Distance to retract along the approach direction (meters)

        Returns:
            Pre-grasp pose offset from the grasp pose
        """
        return offset_distance(
            grasp_pose,
            offset,
            approach_vector if approach_vector is not None else Vector3(0.0, 0.0, -1.0),
        )

    def _find_object_in_detections(
        self, object_name: str, object_id: str | None = None
    ) -> DetObject | None:
        """Find an object in the detection snapshot by name or ID.

        Uses the snapshot taken during the last scan_objects/refresh call,
        not the volatile live cache (which changes labels every frame).

        Args:
            object_name: Name/label to search for
            object_id: Optional specific object ID

        Returns:
            Matching DetObject, or None
        """
        if not self._detection_snapshot:
            logger.warning("No detection snapshot — call scan_objects() first")
            return None

        # First pass: match by object_id (supports both full and truncated IDs)
        if object_id:
            matches = [
                det
                for det in self._detection_snapshot
                if det.object_id == object_id or det.object_id.startswith(object_id)
            ]
            if len(matches) == 1:
                return matches[0]
            if len(matches) > 1:
                ids = [det.object_id for det in matches]
                logger.warning(f"Ambiguous object_id prefix '{object_id}' matches {ids}")
                return None

        # Second pass: require a unique name match.
        normalized = object_name.casefold()
        name_matches = [
            det
            for det in self._detection_snapshot
            if normalized in det.name.casefold() or det.name.casefold() in normalized
        ]
        if len(name_matches) == 1:
            return name_matches[0]
        if len(name_matches) > 1:
            ids = [det.object_id for det in name_matches]
            logger.warning("Ambiguous object name", object_name=object_name, object_ids=ids)
            return None

        available = [det.name for det in self._detection_snapshot]
        logger.warning(f"Object '{object_name}' not found in snapshot. Available: {available}")
        return None

    @staticmethod
    def _occlusion_offset(
        center: Vector3, size: Vector3, inset: float = 0.02
    ) -> tuple[float, float]:
        """Offset a detected object center toward the robot to compensate for single-viewpoint occlusion.

        Returns adjusted (x, y) shifted toward the nearest visible surface + inset.
        """
        xy_dist = (center.x**2 + center.y**2) ** 0.5
        if xy_dist > 1e-3:
            dx, dy = -center.x / xy_dist, -center.y / xy_dist
            half_depth = max(size.x, size.y) / 2.0
            offset = half_depth - inset
            return center.x + dx * offset, center.y + dy * offset
        return center.x, center.y

    @staticmethod
    def _grasp_orientation(gx: float, gy: float, xy_dist: float) -> Quaternion:
        """Compute grasp orientation that tilts toward the object for far reaches.

        Close objects (< 0.6m): top-down (pitch = 180°)
        Far objects (> 1.0m): tilted 45° toward object
        In between: linear interpolation
        """
        near = 0.6
        far = 1.0
        max_tilt = math.pi / 4  # 45° from vertical

        if xy_dist <= near:
            tilt = 0.0
        elif xy_dist >= far:
            tilt = max_tilt
        else:
            tilt = max_tilt * (xy_dist - near) / (far - near)

        # Yaw to face the object direction
        yaw = math.atan2(gy, gx)
        pitch = math.pi - tilt
        return Quaternion.from_euler(Vector3(0.0, pitch, yaw))

    def _generate_grasps_for_pick(
        self, object_name: str, object_id: str | None = None
    ) -> list[Pose] | None:
        """Generate a grasp pose for an object.

        Near objects (< 0.6m XY): apply occlusion offset to compensate for
        single-viewpoint depth underestimation.
        Far objects (>= 0.6m XY): use raw detected center — depth error
        already pushes the center too deep, offset would overshoot.

        Uses distance-adaptive pitch tilt for all distances.

        Args:
            object_name: Name of the object
            object_id: Optional object ID

        Returns:
            List with one grasp pose, or None if object not found
        """
        det = self._find_object_in_detections(object_name, object_id)
        if det is None:
            logger.warning(f"Object '{object_name}' not found in detections")
            return None

        cx, cy, cz = det.center.x, det.center.y, det.center.z
        xy_dist = (cx**2 + cy**2) ** 0.5

        # Distance-adaptive occlusion offset:
        # Near (< 0.8m): small inset — grasp shifted well toward robot (front surface)
        # Far (>= 0.8m): larger inset — less toward-robot shift (grasp closer to true center)
        inset = 0.01 if xy_dist < _FAR_OCCLUSION_XY_THRESHOLD else 0.05
        gx, gy = self._occlusion_offset(det.center, det.size, inset=inset)

        # For tall objects, grasp in the upper third instead of center
        # to avoid plunging deep and colliding with the object.
        obj_height = det.size.z
        if obj_height > _TALL_OBJECT_MIN_HEIGHT:
            gz = cz + obj_height * 0.2  # shift up ~20% from center (upper third)
        else:
            gz = cz

        grasp_dist = (gx**2 + gy**2) ** 0.5
        orientation = self._grasp_orientation(gx, gy, grasp_dist)
        pose = Pose(Vector3(gx, gy, gz), orientation)

        logger.info(
            f"Heuristic grasp for '{object_name}': center=({cx:.3f}, {cy:.3f}, {cz:.3f}), "
            f"grasp=({gx:.3f}, {gy:.3f}, {gz:.3f}), xy_dist={xy_dist:.2f}m, "
            f"inset={inset:.2f}m, "
            f"size=({det.size.x:.3f}, {det.size.y:.3f}, {det.size.z:.3f})"
        )
        return [pose]

    def _resolve_object_position(self, object_name: str) -> tuple[float, float, float] | None:
        """Resolve an object name to its detected center position.

        Returns (x, y, z) or None if object not found in detections.
        No occlusion offset — used for drop_on where we want the true center.
        """
        det = self._find_object_in_detections(object_name)
        if det is None:
            return None
        return det.center.x, det.center.y, det.center.z

    @skill
    def get_scene_info(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Get current robot state, detected objects, and scene information.

        Returns a summary of the robot's joint positions, end-effector pose,
        gripper state, detected objects, and obstacle count.

        Args:
            robot_name: Robot to query (only needed for multi-arm setups).
        """
        lines: list[str] = []

        # Robot state
        joints = self.get_current_joints(robot_name)
        if joints is not None:
            lines.append(f"Joints: [{', '.join(f'{j:.3f}' for j in joints)}]")
        else:
            lines.append("Joints: unavailable (no state received)")

        ee_pose = self.get_ee_pose(robot_name)
        if ee_pose is not None:
            p = ee_pose.position
            lines.append(f"EE pose: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")
        else:
            lines.append("EE pose: unavailable")

        # Gripper
        gripper_pos = self.get_gripper(robot_name)
        if gripper_pos is not None:
            lines.append(f"Gripper: {gripper_pos:.3f}m")
        else:
            lines.append("Gripper: not configured")

        # Perception
        perception = self.get_perception_status()
        lines.append(
            f"Perception: {perception.get('cached', 0)} cached, "
            f"{perception.get('added', 0)} obstacles added"
        )

        detections = self._detection_snapshot
        if detections:
            lines.append(f"Detected objects ({len(detections)}):")
            for det in detections:
                c = det.center
                lines.append(f"  - {det.name}: ({c.x:.3f}, {c.y:.3f}, {c.z:.3f})")
        else:
            lines.append("Detected objects: none")

        # Visualization
        url = self.get_visualization_url()
        if url:
            lines.append(f"Visualization: {url}")

        # State
        lines.append(f"State: {self.get_state()}")

        return SkillResult.ok("\n".join(lines))

    @skill
    def look(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Quick check of what objects are visible from the current camera position.

        Does NOT move the arm. Returns objects currently detected in the camera view.

        Args:
            robot_name: Robot context (only needed for multi-arm setups).
        """
        obstacles = self.refresh_obstacles(0.0)

        detections = self._detection_snapshot
        if not detections:
            return SkillResult.ok("No objects visible from current position")

        lines = [f"Currently see {len(detections)} object(s):"]
        for det in detections:
            c = det.center
            lines.append(
                f"  - {det.name} [id={det.object_id[:8]}]: ({c.x:.3f}, {c.y:.3f}, {c.z:.3f})"
            )

        if obstacles:
            lines.append(f"\n{len(obstacles)} obstacle(s) added to planning world")

        return SkillResult.ok("\n".join(lines))

    @skill
    def scan_objects(
        self,
        min_duration: float = 0.0,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Scan for objects — moves to init position first for a clear camera view, \
then refreshes perception obstacles.

        Use this before pick/place operations or after a failed attempt.

        Args:
            min_duration: Minimum time an object must be seen to be included.
            robot_name: Robot context (only needed for multi-arm setups).
        """
        # Go to init for a clear camera view
        init_result = self.go_init(robot_name)
        if not init_result.is_success():
            return init_result

        obstacles = self.refresh_obstacles(min_duration)

        detections = self._detection_snapshot
        if not detections:
            # See look(): an empty scan is a valid observation, not a failure.
            return SkillResult.ok("No objects detected in scene")

        lines = [f"Detected {len(detections)} object(s):"]
        for det in detections:
            c = det.center
            lines.append(
                f"  - {det.name} [id={det.object_id[:8]}]: "
                f"({c.x:.3f}, {c.y:.3f}, {c.z:.3f}) [{det.detections_count} views]"
            )

        if obstacles:
            lines.append(f"\n{len(obstacles)} obstacle(s) added to planning world")

        return SkillResult.ok("\n".join(lines))

    def _require_pick_object(self, object_name: str, object_id: str | None) -> DetObject:
        detection = self._find_object_in_detections(object_name, object_id)
        if detection is not None:
            return detection
        selector = f"id '{object_id}'" if object_id else f"name '{object_name}'"
        raise _PickPipelineError(
            "OBJECT_NOT_DETECTED",
            f"No unique current detection matches {selector}; scan again and use an object ID",
        )

    def _provider_candidates(
        self, detection: DetObject, transaction: _PickTransaction
    ) -> list[GraspCandidate]:
        if self._grasp_generator is None:
            if not self.config.heuristic_grasp_fallback:
                raise _PickPipelineError(
                    "GRASP_PROVIDER_UNAVAILABLE",
                    "No grasp proposal provider is connected and heuristic fallback is disabled",
                )
            transaction.proposal_source = "heuristic"
            poses = self._generate_grasps_for_pick(detection.name, detection.object_id)
            if not poses:
                raise _PickPipelineError(
                    "GRASP_GENERATION_FAILED",
                    f"Heuristic grasp generation failed for '{detection.name}'",
                )
            return [GraspCandidate(pose=pose, score=0.0) for pose in poses]

        if self._object_scene is None:
            raise _PickPipelineError(
                "GRASP_PROVIDER_UNAVAILABLE",
                "No object-scene provider is connected for learned grasp input",
            )

        pointcloud = self._object_scene.get_object_pointcloud_by_object_id(detection.object_id)
        if pointcloud is None:
            raise _PickPipelineError(
                "GRASP_INPUT_INVALID",
                f"No point cloud is available for object '{detection.object_id}'",
            )
        points = pointcloud.points_f32()
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            raise _PickPipelineError(
                "GRASP_INPUT_INVALID",
                f"Object '{detection.object_id}' has an empty or invalid point cloud",
            )
        if (
            pointcloud.ts is None
            or time.time() - pointcloud.ts > self.config.max_object_pointcloud_age
        ):
            raise _PickPipelineError(
                "GRASP_INPUT_INVALID",
                f"Object '{detection.object_id}' point cloud is stale",
            )
        if pointcloud.frame_id != self.config.planning_frame:
            raise _PickPipelineError(
                "GRASP_FRAME_MISMATCH",
                f"Object cloud frame '{pointcloud.frame_id}' does not match "
                f"planning frame '{self.config.planning_frame}'",
            )

        try:
            proposals = self._grasp_generator.propose_grasps(pointcloud)
        except Exception as exc:
            raise _PickPipelineError(
                "GRASP_GENERATION_FAILED", f"Grasp proposal failed: {exc}"
            ) from exc
        if proposals.header.frame_id != self.config.planning_frame:
            raise _PickPipelineError(
                "GRASP_FRAME_MISMATCH",
                f"Proposal frame '{proposals.header.frame_id}' does not match "
                f"planning frame '{self.config.planning_frame}'",
            )
        if not proposals.candidates:
            raise _PickPipelineError(
                "GRASP_GENERATION_FAILED",
                f"No grasp proposals were generated for '{detection.name}'",
            )
        return sorted(proposals.candidates, key=lambda candidate: candidate.score, reverse=True)

    @staticmethod
    def _valid_candidate(candidate: GraspCandidate) -> bool:
        pose = candidate.pose
        values = np.asarray(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                candidate.score,
            ],
            dtype=float,
        )
        quaternion = values[3:7]
        return bool(
            np.all(np.isfinite(values)) and np.isclose(np.linalg.norm(quaternion), 1.0, atol=1e-5)
        )

    @staticmethod
    def _candidate_retraction_vector(candidate: GraspCandidate, vector: Vector3) -> np.ndarray:
        local = np.asarray([vector.x, vector.y, vector.z], dtype=float)
        norm = np.linalg.norm(local)
        if norm == 0.0:
            return np.zeros(3, dtype=float)
        return candidate.pose.orientation.to_rotation_matrix() @ (local / norm)

    def _select_feasible_grasp(
        self,
        candidates: list[GraspCandidate],
        robot_name: str,
        robot_pre_grasp_offset: float,
        transaction: _PickTransaction,
        sequence_start: JointState | None = None,
    ) -> _FeasibleGrasp:
        vector = Vector3(self.config.grasp_approach_vector)
        pre_offset = self.config.grasp_pre_grasp_offset or robot_pre_grasp_offset
        retreat_offset = self.config.grasp_retreat_offset or pre_offset
        limit = min(len(candidates), self.config.max_grasp_candidates_to_check)

        for rank, candidate in enumerate(candidates[:limit], start=1):
            if not self._valid_candidate(candidate):
                transaction.rejections[_CandidateRejection.INVALID.value] += 1
                logger.info(
                    "Rejected grasp candidate rank=%d score=%.4f: invalid",
                    rank,
                    candidate.score,
                )
                continue
            pre_grasp = self._compute_pre_grasp_pose(candidate.pose, pre_offset, vector)
            retreat = self._compute_pre_grasp_pose(candidate.pose, retreat_offset, vector)
            retraction = self._candidate_retraction_vector(candidate, vector)
            rejections = (
                _CandidateRejection.PRE_GRASP_INFEASIBLE,
                _CandidateRejection.GRASP_INFEASIBLE,
                _CandidateRejection.RETREAT_INFEASIBLE,
            )
            failed_index, _ = self._check_connected_pose_sequence(
                (pre_grasp, candidate.pose, retreat),
                robot_name,
                start=sequence_start,
            )
            if failed_index is not None:
                transaction.rejections[rejections[failed_index].value] += 1
                logger.info(
                    "Rejected grasp candidate rank=%d score=%.4f retraction=(%.3f, %.3f, %.3f) "
                    "grasp=(%.3f, %.3f, %.3f) pre_grasp=(%.3f, %.3f, %.3f): %s",
                    rank,
                    candidate.score,
                    retraction[0],
                    retraction[1],
                    retraction[2],
                    candidate.pose.position.x,
                    candidate.pose.position.y,
                    candidate.pose.position.z,
                    pre_grasp.position.x,
                    pre_grasp.position.y,
                    pre_grasp.position.z,
                    rejections[failed_index].value,
                )
                continue
            return _FeasibleGrasp(candidate, rank, pre_grasp, retreat)

        summary = ", ".join(
            f"{reason}={count}" for reason, count in sorted(transaction.rejections.items())
        )
        raise _PickPipelineError(
            "GRASP_ATTEMPTS_EXHAUSTED",
            f"No feasible grasp among {limit} candidate(s)" + (f" ({summary})" if summary else ""),
        )

    def _verify_grasp(self, robot_name: str) -> _GraspVerification:
        verification = self.config.grasp_verification
        if not verification.enabled:
            return _GraspVerification(True, None, "gripper feedback verification disabled")

        deadline = time.monotonic() + verification.timeout
        last_position: float | None = None
        while time.monotonic() < deadline:
            last_position = self.get_gripper(robot_name)
            if last_position is not None:
                closes_upward = verification.closed_position > verification.open_position
                empty = (
                    last_position >= verification.held_threshold
                    if closes_upward
                    else last_position <= verification.held_threshold
                )
                if empty:
                    return _GraspVerification(
                        False, last_position, "gripper reached the empty-closed region"
                    )
            time.sleep(verification.poll_interval)

        if last_position is None:
            return _GraspVerification(False, None, "gripper feedback was unavailable")
        movement = abs(last_position - verification.open_position)
        if movement < 1e-3:
            return _GraspVerification(
                False, last_position, "gripper did not leave the open position"
            )
        closes_upward = verification.closed_position > verification.open_position
        held = (
            last_position < verification.held_threshold
            if closes_upward
            else last_position > verification.held_threshold
        )
        detail = (
            "grasp verified by gripper closure feedback"
            if held
            else "gripper reached the empty-closed region"
        )
        return _GraspVerification(held, last_position, detail)

    @staticmethod
    def _phase_failure(
        transaction: _PickTransaction,
        code: ManipulationSkillError,
        message: str,
    ) -> SkillResult[ManipulationSkillError]:
        may_hold = transaction.gripper_closed
        suffix = "; object may be held" if may_hold else ""
        result = SkillResult[ManipulationSkillError].fail(
            code, f"{transaction.phase.value}: {message}{suffix}"
        )
        result.metadata = {
            "phase": transaction.phase.value,
            "object_id": transaction.object_id,
            "proposal_source": transaction.proposal_source,
            "object_may_be_held": may_hold,
            "rejections": dict(transaction.rejections),
        }
        if transaction.selected is not None:
            result.metadata.update(
                candidate_rank=transaction.selected.rank,
                candidate_score=transaction.selected.candidate.score,
            )
        return result

    def _execute_selected_pick(
        self, transaction: _PickTransaction, robot_name: str
    ) -> SkillResult[ManipulationSkillError]:
        assert transaction.selected is not None
        selected = transaction.selected
        verification = self.config.grasp_verification

        transaction.phase = _PickPhase.PREPARE
        lift = self._lift_if_low(robot_name)
        if not lift.is_success():
            return self._phase_failure(
                transaction, lift.error_code or "EXECUTION_FAILED", lift.message
            )
        if not self._set_gripper_position(float(verification.open_position), robot_name):
            return self._phase_failure(transaction, "GRIPPER_FAILED", "open command failed")

        transaction.phase = _PickPhase.APPROACH
        if not self.plan_to_pose(selected.pre_grasp_pose, robot_name):
            return self._phase_failure(transaction, "PLANNING_FAILED", "pre-grasp planning failed")
        execution = self._preview_execute_wait(robot_name)
        if not execution.is_success():
            return self._phase_failure(
                transaction, execution.error_code or "EXECUTION_FAILED", execution.message
            )

        transaction.phase = _PickPhase.GRASP
        if not self.plan_to_pose(selected.candidate.pose, robot_name):
            return self._phase_failure(transaction, "PLANNING_FAILED", "grasp planning failed")
        execution = self._preview_execute_wait(robot_name)
        if not execution.is_success():
            return self._phase_failure(
                transaction, execution.error_code or "EXECUTION_FAILED", execution.message
            )

        transaction.phase = _PickPhase.CLOSE
        if not self._set_gripper_position(float(verification.closed_position), robot_name):
            return self._phase_failure(transaction, "GRIPPER_FAILED", "close command failed")
        transaction.gripper_closed = True

        transaction.phase = _PickPhase.VERIFY
        verified = self._verify_grasp(robot_name)
        if not verified.held:
            return self._phase_failure(transaction, "GRASP_VERIFICATION_FAILED", verified.detail)

        transaction.phase = _PickPhase.RETREAT
        if not self.plan_to_pose(selected.retreat_pose, robot_name):
            return self._phase_failure(transaction, "PLANNING_FAILED", "retreat planning failed")
        execution = self._preview_execute_wait(robot_name)
        if not execution.is_success():
            return self._phase_failure(
                transaction, execution.error_code or "EXECUTION_FAILED", execution.message
            )

        transaction.phase = _PickPhase.DONE
        self._last_pick_pose = selected.candidate.pose
        return SkillResult.ok(
            f"Pick complete — grasped '{transaction.object_name}' using candidate "
            f"{selected.rank} (score={selected.candidate.score:.4f}); {verified.detail}",
            object_id=transaction.object_id,
            proposal_source=transaction.proposal_source,
            candidate_rank=selected.rank,
            candidate_score=selected.candidate.score,
            verification=verified.detail,
            rejections=dict(transaction.rejections),
        )

    @skill
    def pick(
        self,
        object_name: str,
        object_id: str | None = None,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Pick up an object by name using grasp planning and motion execution.

        Generates grasp poses, plans collision-free approach/grasp/retract motions,
        and executes them.

        Args:
            object_name: Name of the object to pick (e.g. "cup", "bottle", "can").
            object_id: Optional unique object ID from perception for precise identification.
            robot_name: Robot to use (only needed for multi-arm setups).
        """
        if not self._pick_guard.acquire(blocking=False):
            return SkillResult.fail("PICK_BUSY", "Another pick transaction is active")

        transaction = _PickTransaction()
        suppression = None
        result: SkillResult[ManipulationSkillError]
        try:
            robot = self._get_robot(robot_name)
            if robot is None:
                return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
            rname, _, robot_config, _ = robot

            detection = self._require_pick_object(object_name, object_id)
            transaction.object_id = detection.object_id
            transaction.object_name = detection.name
            transaction.phase = _PickPhase.PROPOSE
            candidates = self._provider_candidates(detection, transaction)

            if self._world_monitor is None:
                raise _PickPipelineError(
                    "WORLD_MONITOR_UNAVAILABLE", "Planning world monitor is unavailable"
                )

            suppression_context = (
                self._world_monitor.suppress_all_object_obstacles()
                if self.config.pick_suppress_all_object_obstacles
                else self._world_monitor.suppress_object_obstacle(detection.object_id)
            )
            with suppression_context as suppression:
                sequence_start = None
                lift_pose = self._safety_lift_pose(rname)
                if lift_pose is not None:
                    transaction.phase = _PickPhase.PREPARE
                    failed_index, sequence_start = self._check_connected_pose_sequence(
                        (lift_pose,), rname
                    )
                    if failed_index is not None:
                        raise _PickPipelineError(
                            "PLANNING_FAILED",
                            "Required safety-lift planning failed",
                        )
                transaction.phase = _PickPhase.SELECT
                transaction.selected = self._select_feasible_grasp(
                    candidates,
                    rname,
                    robot_config.pre_grasp_offset,
                    transaction,
                    sequence_start,
                )
                result = self._execute_selected_pick(transaction, rname)
            if suppression.cleanup_error is not None:
                if result.is_success():
                    return self._phase_failure(
                        transaction, "WORLD_MONITOR_UNAVAILABLE", suppression.cleanup_error
                    )
                result.message = f"{result.message}; cleanup: {suppression.cleanup_error}"
            return result
        except _PickPipelineError as exc:
            return self._phase_failure(transaction, exc.code, str(exc))
        except RuntimeError as exc:
            return self._phase_failure(transaction, "WORLD_MONITOR_UNAVAILABLE", str(exc))
        finally:
            self._pick_guard.release()

    @skill
    def place(
        self,
        x: float,
        y: float,
        z: float,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Place a held object at the specified position.

        Plans and executes an approach, lowers to the target, releases the gripper,
        and retracts.

        Args:
            x: Target X position in meters.
            y: Target Y position in meters.
            z: Target Z position in meters.
            robot_name: Robot to use (only needed for multi-arm setups).
        """
        xy_dist = (x**2 + y**2) ** 0.5
        orientation = self._grasp_orientation(x, y, xy_dist)
        return self._place_with_orientation(x, y, z, orientation, robot_name)

    def _place_with_orientation(
        self,
        x: float,
        y: float,
        z: float,
        orientation: Quaternion,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Internal place with explicit orientation."""
        robot = self._get_robot(robot_name)
        if robot is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
        rname, _, config, _ = robot
        pre_place_offset = config.pre_grasp_offset

        # Reduce pre-place height for far targets
        xy_dist = (x**2 + y**2) ** 0.5
        if xy_dist >= _FAR_REACH_XY_THRESHOLD:
            pre_place_offset = 0.05

        place_pose = Pose(Vector3(x, y, z), orientation)
        pre_place_pose = self._compute_pre_grasp_pose(place_pose, pre_place_offset)

        # Lift if EE is low before approaching
        lift = self._lift_if_low(rname)
        if not lift.is_success():
            return lift

        # 1. Move to pre-place
        logger.info(f"Planning approach to place position ({x:.3f}, {y:.3f}, {z:.3f})...")
        if not self.plan_to_pose(pre_place_pose, rname):
            return SkillResult.fail("PLANNING_FAILED", "Pre-place approach planning failed")

        exec_result = self._preview_execute_wait(rname)
        if not exec_result.is_success():
            return exec_result

        # 2. Lower to place position
        logger.info("Lowering to place position...")
        if not self.plan_to_pose(place_pose, rname):
            return SkillResult.fail("PLANNING_FAILED", "Place pose planning failed")
        exec_result = self._preview_execute_wait(rname)
        if not exec_result.is_success():
            return exec_result

        # 3. Release
        logger.info("Releasing object...")
        self._set_gripper_position(0.85, rname)
        time.sleep(1.0)

        # 4. Retract
        logger.info("Retracting...")
        if not self.plan_to_pose(pre_place_pose, rname):
            return SkillResult.fail("PLANNING_FAILED", "Retract planning failed")
        exec_result = self._preview_execute_wait(rname)
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok(f"Place complete — object released at ({x:.3f}, {y:.3f}, {z:.3f})")

    @skill
    def place_back(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        """Place the held object back at its original pick position.

        Uses the position stored from the last successful pick operation.

        Args:
            robot_name: Robot to use (only needed for multi-arm setups).
        """
        if self._last_pick_pose is None:
            return SkillResult.fail(
                "NO_PRIOR_POSE",
                "No previous pick position stored — run pick() first",
            )

        p = self._last_pick_pose.position
        o = self._last_pick_pose.orientation
        logger.info(f"Placing back at original position ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})...")
        return self._place_with_orientation(p.x, p.y, p.z, o, robot_name)

    @skill
    def drop_on(
        self,
        target_object_name: str,
        z_offset: float = 0.1,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Drop a held object on top of a detected object.

        Resolves the target object's position with occlusion correction and
        places the held object above it.

        Args:
            target_object_name: Name of the target object to drop onto (e.g. "cup", "bowl").
            z_offset: Height above the target object's center to release (meters).
            robot_name: Robot to use (only needed for multi-arm setups).
        """
        pos = self._resolve_object_position(target_object_name)
        if pos is None:
            return SkillResult.fail(
                "OBJECT_NOT_DETECTED",
                f"Target object '{target_object_name}' not found in detections",
            )
        x, y, z = pos
        z += z_offset
        logger.info(
            f"Dropping on '{target_object_name}' at corrected position ({x:.3f}, {y:.3f}, {z:.3f})"
        )
        return self.place(x, y, z, robot_name)

    @skill
    def pick_and_place(
        self,
        object_name: str,
        place_x: float,
        place_y: float,
        place_z: float,
        object_id: str | None = None,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        """Pick up an object and place it at a target location.

        Combines the pick and place skills into a single end-to-end operation.

        Args:
            object_name: Name of the object to pick (e.g. "cup", "bottle").
            place_x: Target X position to place the object (meters).
            place_y: Target Y position to place the object (meters).
            place_z: Target Z position to place the object (meters).
            object_id: Optional unique object ID from perception.
            robot_name: Robot to use (only needed for multi-arm setups).
        """
        logger.info(
            f"Starting pick and place: pick '{object_name}' → place at "
            f"({place_x:.3f}, {place_y:.3f}, {place_z:.3f})"
        )

        # Pick phase
        pick_result = self.pick(object_name, object_id, robot_name)
        if not pick_result.is_success():
            return pick_result

        # Place phase
        return self.place(place_x, place_y, place_z, robot_name)

    @rpc
    def stop(self) -> None:
        """Stop the pick-and-place module."""
        logger.info("Stopping PickAndPlaceModule")
        super().stop()
