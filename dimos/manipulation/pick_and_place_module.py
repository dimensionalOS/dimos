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

import math
import time
from typing import TYPE_CHECKING, Any, Literal

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
)
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental.object import (
    Object as DetObject,
)
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseArray import PoseArray
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


class PickAndPlaceModuleConfig(ManipulationModuleConfig):
    """Configuration for PickAndPlaceModule."""

    pick_verification: Literal["none", "observed_lift"] = "none"
    pick_minimum_lift_m: float = Field(default=0.02, gt=0.0)
    pick_verification_timeout: float = Field(default=1.0, gt=0.0)


class PickAndPlaceModule(ManipulationModule):
    """Manipulation module with perception integration and pick-and-place skills.

    Extends ManipulationModule with:
    - Perception: objects port, obstacle monitor, scan_objects, get_scene_info
    - @rpc: generate_grasps (GraspGen Docker), refresh_obstacles, perception status
    - @skill: pick, place, place_back, pick_and_place, scan_objects, get_scene_info
    """

    config: PickAndPlaceModuleConfig

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
        direction: Vector3 | None = None,
    ) -> Pose:
        """Compute a pre-grasp pose along the configured TCP-local direction.

        Args:
            grasp_pose: The final grasp pose
            offset: Distance to retract along the approach direction (meters)
            direction: TCP-local retreat direction. Defaults to local -Z.

        Returns:
            Pre-grasp pose offset from the grasp pose
        """
        from dimos.utils.transform_utils import offset_distance

        return offset_distance(
            grasp_pose,
            offset,
            approach_vector=direction or Vector3(0.0, 0.0, -1.0),
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

        # Second pass: match by name
        for det in self._detection_snapshot:
            if object_name.lower() in det.name.lower() or det.name.lower() in object_name.lower():
                return det

        available = [det.name for det in self._detection_snapshot]
        logger.warning(f"Object '{object_name}' not found in snapshot. Available: {available}")
        return None

    @staticmethod
    def _occlusion_offset(
        center: Vector3,
        size: Vector3,
        inset: float = 0.02,
        base_pose: PoseStamped | None = None,
    ) -> tuple[float, float]:
        """Offset a detected object center toward the robot to compensate for single-viewpoint occlusion.

        Returns adjusted (x, y) shifted toward the nearest visible surface + inset.
        """
        base_x = 0.0 if base_pose is None else base_pose.position.x
        base_y = 0.0 if base_pose is None else base_pose.position.y
        toward_base_x = base_x - center.x
        toward_base_y = base_y - center.y
        xy_dist = math.hypot(toward_base_x, toward_base_y)
        if xy_dist > 1e-3:
            dx, dy = toward_base_x / xy_dist, toward_base_y / xy_dist
            half_depth = max(size.x, size.y) / 2.0
            offset = half_depth - inset
            return center.x + dx * offset, center.y + dy * offset
        return center.x, center.y

    @staticmethod
    def _xy_from_base(x: float, y: float, base_pose: PoseStamped) -> tuple[float, float, float]:
        dx = x - base_pose.position.x
        dy = y - base_pose.position.y
        return dx, dy, math.hypot(dx, dy)

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
        self,
        object_name: str,
        object_id: str | None = None,
        grasp_frame_to_tcp: Pose | None = None,
        base_pose: PoseStamped | None = None,
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
        resolved_base = base_pose or PoseStamped()
        _, _, xy_dist = self._xy_from_base(cx, cy, resolved_base)

        if self._is_mechanics_truth(det):
            gx, gy, gz = cx, cy, cz
            source = "mechanics truth"
        else:
            # Perception estimates can represent the visible front surface instead of
            # the object center. These corrections apply only to perception data.
            inset = 0.01 if xy_dist < _FAR_OCCLUSION_XY_THRESHOLD else 0.05
            gx, gy = self._occlusion_offset(
                det.center,
                det.size,
                inset=inset,
                base_pose=resolved_base,
            )
            obj_height = det.size.z
            gz = cz + obj_height * 0.2 if obj_height > _TALL_OBJECT_MIN_HEIGHT else cz
            source = f"perception heuristic, inset={inset:.2f}m"

        grasp_dx, grasp_dy, grasp_dist = self._xy_from_base(gx, gy, resolved_base)
        orientation = self._grasp_orientation(grasp_dx, grasp_dy, grasp_dist)
        grasp_pose = Pose(Vector3(gx, gy, gz), orientation)
        pose = grasp_pose
        if grasp_frame_to_tcp is not None:
            pose = matrix_to_pose(pose_to_matrix(grasp_pose) @ pose_to_matrix(grasp_frame_to_tcp))

        logger.info(
            f"Heuristic grasp for '{object_name}': center=({cx:.3f}, {cy:.3f}, {cz:.3f}), "
            f"grasp=({gx:.3f}, {gy:.3f}, {gz:.3f}), "
            f"tcp=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}), "
            f"xy_dist={xy_dist:.2f}m, "
            f"source={source}, "
            f"size=({det.size.x:.3f}, {det.size.y:.3f}, {det.size.z:.3f})"
        )
        return [pose]

    @staticmethod
    def _is_mechanics_truth(det: DetObject) -> bool:
        """Return whether a detection contains privileged simulator state."""
        return (
            det.identity_status == "privileged_ground_truth"
            and det.identity_basis == "pimsim_mechanics_diagnostic"
        )

    def _verify_observed_lift(self, target: DetObject) -> bool:
        """Verify a pick from the public object-observation stream."""
        if self._world_monitor is None:
            return False

        deadline = time.monotonic() + self.config.pick_verification_timeout
        lifted = False
        while time.monotonic() <= deadline:
            for observed in self._world_monitor.get_cached_objects():
                if observed.object_id != target.object_id:
                    continue
                lifted = observed.center.z - target.center.z >= self.config.pick_minimum_lift_m
                break
            time.sleep(0.02)
        return lifted

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

        lines: list[str] = []
        if any(self._is_mechanics_truth(det) for det in detections):
            lines.append("Observation source: privileged simulator mechanics truth")
        lines.append(f"Currently see {len(detections)} object(s):")
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
        # Go to init for a clear camera view. Pick/place remains a legacy
        # subclass, so resolve its robot selector internally and call primitives.
        robot = self._get_robot(robot_name)
        if robot is None or self._world_monitor is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
        resolved_name, _, _ = robot
        group_id = self._world_monitor.planning_groups.default_group_id_for_robot(resolved_name)
        if group_id is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Planning group is ambiguous")
        init = self.get_state().groups[group_id].joint_presets.get("init")
        if init is None:
            return SkillResult.fail("NOT_CONFIGURED", "No init joints captured")
        plan = self.plan_to_joints({group_id: init})
        if not plan.succeeded:
            return SkillResult.fail("PLANNING_FAILED", plan.message)
        execution = self.execute(blocking=True)
        if not execution.succeeded:
            return SkillResult.fail("EXECUTION_FAILED", execution.message)

        obstacles = self.refresh_obstacles(min_duration)

        detections = self._detection_snapshot
        if not detections:
            # See look(): an empty scan is a valid observation, not a failure.
            return SkillResult.ok("No objects detected in scene")

        lines = [f"Detected {len(detections)} object(s):"]
        for det in detections:
            c = det.center
            lines.append(
                f"  - {det.name}: ({c.x:.3f}, {c.y:.3f}, {c.z:.3f}) [{det.detections_count} views]"
            )

        if obstacles:
            lines.append(f"\n{len(obstacles)} obstacle(s) added to planning world")

        return SkillResult.ok("\n".join(lines))

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
        robot = self._get_robot(robot_name)
        if robot is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
        rname, _, config = robot
        pre_grasp_offset = config.pre_grasp_offset

        target = self._find_object_in_detections(object_name, object_id)
        if target is None:
            return SkillResult.fail(
                "GRASP_GENERATION_FAILED",
                f"No grasp poses found for '{object_name}'. Object may not be detected.",
            )

        # 1. Generate grasps (uses already-cached detections — call scan_objects first)
        logger.info(f"Generating grasp poses for '{object_name}'...")
        grasp_poses = self._generate_grasps_for_pick(
            object_name,
            object_id,
            config.grasp_frame_to_tcp,
            config.base_pose,
        )
        if not grasp_poses:
            return SkillResult.fail(
                "GRASP_GENERATION_FAILED",
                f"No grasp poses found for '{object_name}'. Object may not be detected.",
            )

        # The selected object must remain in simulator physics, but it cannot be
        # a collision obstacle for the gripper motion intended to contact it.
        if self._world_monitor is not None:
            self._world_monitor.remove_object_obstacle(target.object_id)

        # Lift if EE is low before approaching
        lift = self._lift_if_low(rname)
        if not lift.is_success():
            return lift

        # 2. Try each grasp candidate
        max_attempts = min(len(grasp_poses), 5)
        for i, grasp_pose in enumerate(grasp_poses[:max_attempts]):
            # Reduce pre-grasp height for far objects (arm can't reach high + far)
            gp = grasp_pose.position
            _, _, xy_dist = self._xy_from_base(gp.x, gp.y, config.base_pose)
            offset = pre_grasp_offset if xy_dist < _FAR_REACH_XY_THRESHOLD else 0.05
            pre_grasp_pose = self._compute_pre_grasp_pose(
                grasp_pose,
                offset,
                config.pre_grasp_direction,
            )

            logger.info(f"Planning approach to pre-grasp (attempt {i + 1}/{max_attempts})...")
            if not self.plan_to_pose(pre_grasp_pose, rname):
                logger.info(f"Grasp candidate {i + 1} approach planning failed, trying next")
                continue  # Try next candidate

            # 3. Open gripper before approach
            logger.info("Opening gripper...")
            self._set_gripper_position(0.85, rname)
            time.sleep(0.5)

            # 4. Execute approach to pre-grasp
            exec_result = self._preview_execute_wait(rname)
            if not exec_result.is_success():
                return exec_result

            # 5. Move to grasp pose
            logger.info("Moving to grasp position...")
            if not self.plan_to_pose(grasp_pose, rname):
                return SkillResult.fail("PLANNING_FAILED", "Grasp pose planning failed")
            exec_result = self._preview_execute_wait(rname)
            if not exec_result.is_success():
                return exec_result

            # 6. Close gripper
            logger.info("Closing gripper...")
            self._set_gripper_position(0.0, rname)
            time.sleep(1.5)  # Wait for gripper to close

            # 7. Retract to pre-grasp
            logger.info("Retracting with object...")
            if not self.plan_to_pose(pre_grasp_pose, rname):
                return SkillResult.fail("PLANNING_FAILED", "Retract planning failed")
            exec_result = self._preview_execute_wait(rname)
            if not exec_result.is_success():
                return exec_result

            # Store pick pose so place_back() can return with same orientation
            self._last_pick_pose = grasp_pose

            if self.config.pick_verification == "observed_lift" and not self._verify_observed_lift(
                target
            ):
                return SkillResult.fail(
                    "PICK_NOT_VERIFIED",
                    f"The arm completed the motion, but '{object_name}' did not lift with it.",
                )

            return SkillResult.ok(f"Pick complete — grasped '{object_name}' successfully")

        return SkillResult.fail(
            "GRASP_ATTEMPTS_EXHAUSTED",
            f"All {max_attempts} grasp attempts failed for '{object_name}'",
        )

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
        robot = self._get_robot(robot_name)
        if robot is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
        _, _, config = robot
        dx, dy, xy_dist = self._xy_from_base(x, y, config.base_pose)
        orientation = self._grasp_orientation(dx, dy, xy_dist)
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
        rname, _, config = robot
        pre_place_offset = config.pre_grasp_offset

        # Reduce pre-place height for far targets
        _, _, xy_dist = self._xy_from_base(x, y, config.base_pose)
        if xy_dist >= _FAR_REACH_XY_THRESHOLD:
            pre_place_offset = 0.05

        contact_pose = Pose(Vector3(x, y, z), orientation)
        place_pose = matrix_to_pose(
            pose_to_matrix(contact_pose) @ pose_to_matrix(config.grasp_frame_to_tcp)
        )
        pre_place_pose = self._compute_pre_grasp_pose(
            place_pose,
            pre_place_offset,
            config.pre_grasp_direction,
        )

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
