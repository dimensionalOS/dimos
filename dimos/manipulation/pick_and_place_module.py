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
from pathlib import Path
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.docker_module import DockerModuleProxy as DockerRunner
from dimos.core.stream import In
from dimos.manipulation.grasping.graspgen_module import GraspGenModule
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
)
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.detection.type.detection3d.object import (
    Object as DetObject,
)
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseArray import PoseArray
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

logger = setup_logger()

# The host-side path (graspgen_visualization_output_path) is volume-mounted here.
_GRASPGEN_VIZ_CONTAINER_DIR = "/output/graspgen"
_GRASPGEN_VIZ_CONTAINER_PATH = f"{_GRASPGEN_VIZ_CONTAINER_DIR}/visualization.json"

# Beyond this XY distance from the base, the arm cannot reach both high and far,
# so pre-grasp/pre-place offsets are reduced.
_FAR_REACH_XY_THRESHOLD = 0.7

# Beyond this XY distance, the occlusion inset is increased so the grasp
# targets closer to the true center rather than the front surface.
_FAR_OCCLUSION_XY_THRESHOLD = 0.8

# Objects taller than this are grasped in the upper third to avoid
# plunging deep and colliding with the object body.
_TALL_OBJECT_MIN_HEIGHT = 0.06

# Empirical grasp correction: the gripper was consistently knocking objects ~3cm to one
# side, so shift every (ground-truth) grasp this far along +X (the robot's right, since
# it faces +Y). Flip the sign or zero it if it over/under-corrects.
_GRASP_RIGHT_OFFSET_M = 0.03

# Ground-truth object obstacles are seeded at this fraction of their true size: the full
# AABB + the planner's conservative arm hulls leave no collision-free pre-grasp in a
# cluttered scene, while a shrunk box does. The objects are physical in MuJoCo, so a
# light graze is harmless.
_OBSTACLE_SHRINK = 0.6


@dataclass
class _GroundTruthDetection:
    """Lightweight ground-truth 'detection' (duck-typed for the pick pipeline,
    which only needs name/center/size/detections_count). Used in sim where YOLO
    is unreliable on synthetic objects; the sim already knows every object pose."""

    name: str
    center: Vector3
    size: Vector3
    object_id: str = ""
    detections_count: int = 1


class PickAndPlaceModuleConfig(ManipulationModuleConfig):
    """Configuration for PickAndPlaceModule (adds GraspGen settings)."""

    # Sim ground-truth objects (name/position/dimensions) used as 'detections' when
    # real perception is unavailable/unreliable; scan_objects() returns these. The
    # 'manip_table'/table entry is ignored (it's not graspable). Empty = use the
    # real perception (objects port) instead.
    ground_truth_objects: list[dict[str, Any]] = Field(default_factory=list)

    # GraspGen Docker settings
    graspgen_docker_image: str = "dimos-graspgen:latest"
    graspgen_gripper_type: str = "robotiq_2f_140"
    graspgen_num_grasps: int = 400
    graspgen_topk_num_grasps: int = 100
    graspgen_grasp_threshold: float = -1.0
    graspgen_filter_collisions: bool = False
    graspgen_save_visualization_data: bool = False
    graspgen_visualization_output_path: Path = (
        Path.home() / ".dimos" / "graspgen" / "visualization.json"
    )


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

        # GraspGen Docker runner (lazy initialized on first generate_grasps call)
        self._graspgen: DockerRunner | None = None

        # Last pick pose + arm: stored during pick so place_back() returns the object
        # with the same hand/orientation.
        self._last_pick_pose: Pose | None = None
        self._last_pick_arm: str = "left"
        self._last_grasp_roll: float = 0.0  # gripper roll used for the last pick

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
        # Ground-truth objects are the KNOWN SCENE, not transient perception: keep the
        # detections (they live only in the snapshot, with no live cache to refresh from)
        # AND re-seed them in the planning world so they stay visible/avoided. pick()
        # already drops just the target object for its approach, so a blanket clear here
        # would only make the whole desk vanish. Live-perception runs still clear fully
        # (the next scan refreshes from the cache).
        if self.config.ground_truth_objects:
            self._seed_ground_truth_obstacles(self._detection_snapshot)
            return SkillResult.ok("Ground-truth objects kept (clear is a no-op in sim)")
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

    def _get_graspgen(self) -> DockerRunner:
        """Get or create GraspGen Docker module (lazy init, thread-safe)."""
        # Fast path: already initialized (no lock needed for read)
        if self._graspgen is not None:
            return self._graspgen

        # Slow path: need to initialize (acquire lock to prevent race condition)
        with self._lock:
            # Double-check: another thread may have initialized while we waited for lock
            if self._graspgen is not None:
                return self._graspgen

            # Ensure GraspGen model checkpoints are pulled from LFS
            get_data("models_graspgen")

            docker_file = (
                DIMOS_PROJECT_ROOT
                / "dimos"
                / "manipulation"
                / "grasping"
                / "docker_context"
                / "Dockerfile"
            )

            # Auto-mount host directory for visualization output when enabled.
            docker_volumes: list[tuple[str, str, str]] = []
            if self.config.graspgen_save_visualization_data:
                host_dir = self.config.graspgen_visualization_output_path.parent
                host_dir.mkdir(parents=True, exist_ok=True)
                docker_volumes.append((str(host_dir), _GRASPGEN_VIZ_CONTAINER_DIR, "rw"))

            graspgen = DockerRunner(
                GraspGenModule,  # type: ignore[arg-type]
                docker_file=docker_file,
                docker_build_context=DIMOS_PROJECT_ROOT,
                docker_image=self.config.graspgen_docker_image,
                docker_env={"CI": "1"},  # skip interactive system config prompt in container
                docker_volumes=docker_volumes,
                gripper_type=self.config.graspgen_gripper_type,
                num_grasps=self.config.graspgen_num_grasps,
                topk_num_grasps=self.config.graspgen_topk_num_grasps,
                grasp_threshold=self.config.graspgen_grasp_threshold,
                filter_collisions=self.config.graspgen_filter_collisions,
                save_visualization_data=self.config.graspgen_save_visualization_data,
                visualization_output_path=_GRASPGEN_VIZ_CONTAINER_PATH,
            )
            graspgen.start()
            self._graspgen = graspgen  # cache only after successful start
            return self._graspgen

    @rpc
    def generate_grasps(
        self,
        pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2 | None = None,
    ) -> PoseArray | None:
        """Generate grasp poses for the given point cloud via GraspGen Docker module."""
        try:
            graspgen = self._get_graspgen()
            return graspgen.generate_grasps(pointcloud, scene_pointcloud)  # type: ignore[no-any-return]
        except Exception as e:
            logger.error(f"Grasp generation failed: {e}")
            return None

    def _compute_pre_grasp_pose(self, grasp_pose: Pose, offset: float = 0.10) -> Pose:
        """Compute a pre-grasp pose offset along the approach direction (local -Z).

        Args:
            grasp_pose: The final grasp pose
            offset: Distance to retract along the approach direction (meters)

        Returns:
            Pre-grasp pose offset from the grasp pose
        """
        from dimos.utils.transform_utils import offset_distance

        return offset_distance(grasp_pose, offset)

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
        center: Vector3, size: Vector3, inset: float = 0.02, base_xy: tuple[float, float] = (0.0, 0.0)
    ) -> tuple[float, float]:
        """Offset a detected object center toward the robot base to compensate for
        single-viewpoint occlusion. base_xy is the robot base in world coords (the
        reference for 'toward the robot' — NOT the world origin, which only matches
        the base for a robot welded at the origin).
        """
        rel_x, rel_y = center.x - base_xy[0], center.y - base_xy[1]
        dist = (rel_x**2 + rel_y**2) ** 0.5
        if dist > 1e-3:
            dx, dy = -rel_x / dist, -rel_y / dist  # toward the base
            half_depth = max(size.x, size.y) / 2.0
            offset = half_depth - inset
            return center.x + dx * offset, center.y + dy * offset
        return center.x, center.y

    @staticmethod
    def _resolve_arm(arm: str, x: float, base_xy: tuple[float, float]) -> str:
        """Resolve 'auto'/'left'/'right' to a concrete side. 'auto' picks the nearer
        arm: the LEFT arm for objects at/left-of the base X, the RIGHT arm otherwise.
        (Verified by FK: with the base facing +Y, left_arm sits at world x<0, right at
        x>0.) Single-arm robots ignore the side downstream (no per-arm tips)."""
        if arm in ("left", "right"):
            return arm
        return "left" if x <= base_xy[0] else "right"

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
        base_xy: tuple[float, float] = (0.0, 0.0),
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
        # Distances/directions are relative to the ROBOT BASE, not the world origin
        # (they coincide only for a robot welded at the origin).
        bx, by = base_xy
        xy_dist = ((cx - bx) ** 2 + (cy - by) ** 2) ** 0.5

        # Grasp height: tall objects grasped in the upper third (avoid plunging deep).
        obj_height = det.size.z
        gz = cz + obj_height * 0.2 if obj_height > _TALL_OBJECT_MIN_HEIGHT else cz

        def _grasp_at(px: float, py: float) -> Pose:
            rx, ry = px - bx, py - by
            orient = self._grasp_orientation(rx, ry, (rx**2 + ry**2) ** 0.5)
            return Pose(Vector3(px, py, gz), orient)

        if self.config.ground_truth_objects:
            # Exact pose known: grasp the TRUE CENTER (the occlusion shift below would put
            # the fingers off the object's edge and push it), plus a fixed +X correction so
            # the gripper stops knocking objects to one side. Add a small toward-base nudge
            # as a SECOND candidate so objects at the arm's reach edge are still graspable;
            # the center is tried first (best grip).
            gx0 = cx + _GRASP_RIGHT_OFFSET_M
            poses = [_grasp_at(gx0, cy)]
            rel_x, rel_y = gx0 - bx, cy - by
            d = (rel_x**2 + rel_y**2) ** 0.5
            if d > 1e-3:
                nudge = min(0.03, max(0.0, min(det.size.x, det.size.y) / 2.0 - 0.01))
                poses.append(_grasp_at(gx0 - rel_x / d * nudge, cy - rel_y / d * nudge))
            logger.info(
                f"Ground-truth grasp for '{object_name}': center=({cx:.3f},{cy:.3f},{gz:.3f}), "
                f"{len(poses)} candidate(s), "
                f"size=({det.size.x:.3f},{det.size.y:.3f},{det.size.z:.3f})"
            )
            return poses

        # Real perception: distance-adaptive occlusion offset (depth-error compensation).
        inset = 0.01 if xy_dist < _FAR_OCCLUSION_XY_THRESHOLD else 0.05
        gx, gy = self._occlusion_offset(det.center, det.size, inset=inset, base_xy=base_xy)
        logger.info(
            f"Heuristic grasp for '{object_name}': center=({cx:.3f}, {cy:.3f}, {cz:.3f}), "
            f"grasp=({gx:.3f}, {gy:.3f}, {gz:.3f}), xy_dist={xy_dist:.2f}m, inset={inset:.2f}m, "
            f"size=({det.size.x:.3f}, {det.size.y:.3f}, {det.size.z:.3f})"
        )
        return [_grasp_at(gx, gy)]

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

    def _ground_truth_detections(self) -> list[_GroundTruthDetection]:
        """Build sim ground-truth detections from config (skips the table/floor)."""
        dets: list[_GroundTruthDetection] = []
        for spec in self.config.ground_truth_objects:
            raw = str(spec.get("name", ""))
            name = raw.replace("manip_", "")
            if not name or "table" in name or name == "floor":
                continue
            pos, dim = spec["position"], spec["dimensions"]
            dets.append(
                _GroundTruthDetection(
                    name=name,
                    center=Vector3(float(pos[0]), float(pos[1]), float(pos[2])),
                    size=Vector3(float(dim[0]), float(dim[1]), float(dim[2])),
                    object_id=raw,
                )
            )
        return dets

    def _seed_ground_truth_obstacles(self, detections: list[_GroundTruthDetection]) -> None:
        """Add each ground-truth detection to the planning world as a PERCEPTION obstacle
        so it renders in the viz (viser) AND the path planner routes the arm AROUND it
        (instead of sweeping through it on the way to the pre-grasp). The boxes are shrunk
        by _OBSTACLE_SHRINK: the full AABB plus the planner's conservative arm hulls leaves
        no collision-free pre-grasp config in a cluttered scene, while a 0.6x box does (and
        the objects are physical in MuJoCo, so a light graze is harmless). pick() drops the
        TARGET's box so the gripper can reach it."""
        if self._world_monitor is None:
            return
        for det in detections:
            obstacle = Obstacle(
                name=det.name,
                pose=Pose(det.center, Quaternion(0.0, 0.0, 0.0, 1.0)),
                obstacle_type=ObstacleType.BOX,
                dimensions=(
                    float(det.size.x) * _OBSTACLE_SHRINK,
                    float(det.size.y) * _OBSTACLE_SHRINK,
                    float(det.size.z) * _OBSTACLE_SHRINK,
                ),
                color=(0.2, 0.7, 0.9, 0.9),
            )
            self._world_monitor.add_object_obstacle(det.object_id or det.name, obstacle)

    def _reseed_objects(self, exclude: str | None = None) -> None:
        """Re-add the ground-truth object obstacles still on the desk (for viz between
        picks), skipping a just-grasped object — its static ground-truth pose is stale
        once it's in the gripper."""
        if not self.config.ground_truth_objects or self._world_monitor is None:
            return
        self._seed_ground_truth_obstacles(
            [d for d in self._detection_snapshot if d.name != exclude]
        )

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
        # Sim ground-truth: the sim knows every object pose, so use those directly
        # (YOLO is unreliable on synthetic objects). The objects are already in the
        # planning world as static obstacles; this just exposes them for pick/place.
        # No camera view is needed, so we SKIP the go_init repositioning that would
        # otherwise swing the arm to home+observe for nothing before the scan.
        if self.config.ground_truth_objects:
            self._detection_snapshot = self._ground_truth_detections()
            dets = self._detection_snapshot
            if not dets:
                return SkillResult.ok("No objects in scene")
            # Show the objects in the planning world (viser) + make the planner
            # object-aware. They're added as PERCEPTION obstacles, so pick() drops
            # them (clear_perception_obstacles) before the grasp approach.
            self._seed_ground_truth_obstacles(dets)
            lines = [f"Detected {len(dets)} object(s):"]
            lines += [f"  - {d.name}: ({d.center.x:.3f}, {d.center.y:.3f}, {d.center.z:.3f})" for d in dets]
            return SkillResult.ok("\n".join(lines))

        # Real perception: move to init for a clear camera view, then refresh.
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
        arm: str = "auto",
    ) -> SkillResult[ManipulationSkillError]:
        """Pick up an object by name using grasp planning and motion execution.

        Generates grasp poses, plans collision-free approach/grasp/retract motions,
        and executes them.

        Args:
            object_name: Name of the object to pick (e.g. "cup", "bottle", "can").
            object_id: Optional unique object ID from perception for precise identification.
            robot_name: Robot to use (only needed for multi-arm setups).
            arm: Which arm to grasp with — "left", "right", or "auto" (default; picks the
                arm nearer the object). Ignored by single-arm robots.
        """
        if arm not in ("auto", "left", "right"):
            return SkillResult.fail("INVALID_ARM", f"arm must be auto/left/right, got '{arm}'")
        robot = self._get_robot(robot_name)
        if robot is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
        rname, _, config, _ = robot
        pre_grasp_offset = config.pre_grasp_offset
        base_xy = (config.base_pose.position.x, config.base_pose.position.y)

        # 1. Generate grasps (uses already-cached detections — call scan_objects first)
        logger.info(f"Generating grasp poses for '{object_name}'...")
        grasp_poses = self._generate_grasps_for_pick(object_name, object_id, base_xy)
        if not grasp_poses:
            return SkillResult.fail(
                "GRASP_GENERATION_FAILED",
                f"No grasp poses found for '{object_name}'. Object may not be detected.",
            )

        target_det = self._find_object_in_detections(object_name, object_id)

        # Roll the gripper so its finger axis lines up with the object's NARROW horizontal
        # dimension (the natural roll leaves it along world X). Without this, a wider-than-
        # gripper axis is gripped and the object is pushed instead of pinched.
        grasp_roll = (
            math.pi / 2.0
            if (target_det is not None and target_det.size.y < target_det.size.x)
            else 0.0
        )
        self._last_grasp_roll = grasp_roll

        # Drop ONLY the TARGET object's obstacle so the gripper can reach it; the others
        # stay (shrunk, see _seed_ground_truth_obstacles) so the path routes AROUND them
        # instead of sweeping the gripper through them on the way to the pre-grasp. Real
        # perception still clears everything (the folded camera-over-desk pose can sit
        # inside a detection box).
        if self.config.ground_truth_objects and self._world_monitor is not None:
            if target_det is not None:
                self._world_monitor.remove_object_obstacle(target_det.object_id or target_det.name)
        else:
            self.clear_perception_obstacles()

        # Lift if EE is low before approaching
        lift = self._lift_if_low(rname)
        if not lift.is_success():
            return lift

        # 2. Try each grasp candidate
        max_attempts = min(len(grasp_poses), 5)
        for i, grasp_pose in enumerate(grasp_poses[:max_attempts]):
            # Reduce pre-grasp height for far objects (arm can't reach high + far)
            gp = grasp_pose.position
            xy_dist = (gp.x**2 + gp.y**2) ** 0.5
            offset = pre_grasp_offset if xy_dist < _FAR_REACH_XY_THRESHOLD else 0.05
            pre_grasp_pose = self._compute_pre_grasp_pose(grasp_pose, offset)

            # Choose the grasping arm (nearer side for "auto"); the idle arm is held.
            chosen = self._resolve_arm(arm, gp.x, base_xy)

            logger.info(
                f"Planning approach to pre-grasp with {chosen} arm "
                f"(attempt {i + 1}/{max_attempts})..."
            )
            if not self._plan_arm_to_pose(
                pre_grasp_pose, chosen, rname, grasp_tcp=True, grasp_roll=grasp_roll
            ):
                logger.info(f"Grasp candidate {i + 1} approach planning failed, trying next")
                self._clear_planning_fault()  # so the next candidate can plan
                continue  # Try next candidate

            # 3. Open gripper before approach
            logger.info("Opening gripper...")
            self._set_gripper_position(0.85, rname, arm=chosen)
            time.sleep(0.5)

            # 4. Execute approach to pre-grasp
            exec_result = self._preview_execute_wait(rname)
            if not exec_result.is_success():
                return exec_result

            # 5. Move to grasp pose
            logger.info("Moving to grasp position...")
            if not self._plan_arm_to_pose(grasp_pose, chosen, rname, grasp_tcp=True, grasp_roll=grasp_roll):
                return SkillResult.fail("PLANNING_FAILED", "Grasp pose planning failed")
            exec_result = self._preview_execute_wait(rname)
            if not exec_result.is_success():
                return exec_result

            # 6. Close gripper
            logger.info("Closing gripper...")
            self._set_gripper_position(0.0, rname, arm=chosen)
            time.sleep(1.5)  # Wait for gripper to close

            # 7. Retract to pre-grasp
            logger.info("Retracting with object...")
            if not self._plan_arm_to_pose(pre_grasp_pose, chosen, rname, grasp_tcp=True, grasp_roll=grasp_roll):
                return SkillResult.fail("PLANNING_FAILED", "Retract planning failed")
            exec_result = self._preview_execute_wait(rname)
            if not exec_result.is_success():
                return exec_result

            # Store pick pose + arm so place_back() returns with the same hand/orientation
            self._last_pick_pose = grasp_pose
            self._last_pick_arm = chosen

            # Grasped: leave the target's box dropped (it's in the gripper); the others
            # were never removed, so the desk stays as-is in the viz.
            return SkillResult.ok(
                f"Pick complete — grasped '{object_name}' with the {chosen} arm"
            )

        # All candidates failed: put the target's box back (only it was dropped) and clear
        # the FAULT so the next command can plan.
        self._reseed_objects()
        self._clear_planning_fault()
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
        arm: str = "auto",
    ) -> SkillResult[ManipulationSkillError]:
        """Place a held object at the specified position.

        Plans and executes an approach, lowers to the target, releases the gripper,
        and retracts.

        Args:
            x: Target X position in meters.
            y: Target Y position in meters.
            z: Target Z position in meters.
            robot_name: Robot to use (only needed for multi-arm setups).
            arm: Which arm places — "left"/"right", or "auto" (default; the hand that
                picked the object). Ignored by single-arm robots.
        """
        xy_dist = (x**2 + y**2) ** 0.5
        orientation = self._grasp_orientation(x, y, xy_dist)
        return self._place_with_orientation(x, y, z, orientation, robot_name, arm)

    def _place_with_orientation(
        self,
        x: float,
        y: float,
        z: float,
        orientation: Quaternion,
        robot_name: str | None = None,
        arm: str = "auto",
    ) -> SkillResult[ManipulationSkillError]:
        """Internal place with explicit orientation."""
        if arm not in ("auto", "left", "right"):
            return SkillResult.fail("INVALID_ARM", f"arm must be auto/left/right, got '{arm}'")
        robot = self._get_robot(robot_name)
        if robot is None:
            return SkillResult.fail("ROBOT_NOT_FOUND", "Robot not found")
        rname, _, config, _ = robot
        pre_place_offset = config.pre_grasp_offset
        # The object is held by the arm that picked it; "auto" places with that hand.
        chosen = arm if arm in ("left", "right") else self._last_pick_arm

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
        logger.info(
            f"Planning approach to place ({x:.3f}, {y:.3f}, {z:.3f}) with {chosen} arm..."
        )
        if not self._plan_arm_to_pose(pre_place_pose, chosen, rname, grasp_tcp=True, grasp_roll=self._last_grasp_roll):
            return SkillResult.fail("PLANNING_FAILED", "Pre-place approach planning failed")

        exec_result = self._preview_execute_wait(rname)
        if not exec_result.is_success():
            return exec_result

        # 2. Lower to place position
        logger.info("Lowering to place position...")
        if not self._plan_arm_to_pose(place_pose, chosen, rname, grasp_tcp=True, grasp_roll=self._last_grasp_roll):
            return SkillResult.fail("PLANNING_FAILED", "Place pose planning failed")
        exec_result = self._preview_execute_wait(rname)
        if not exec_result.is_success():
            return exec_result

        # 3. Release
        logger.info("Releasing object...")
        self._set_gripper_position(0.85, rname, arm=chosen)
        time.sleep(1.0)

        # 4. Retract
        logger.info("Retracting...")
        if not self._plan_arm_to_pose(pre_place_pose, chosen, rname, grasp_tcp=True, grasp_roll=self._last_grasp_roll):
            return SkillResult.fail("PLANNING_FAILED", "Retract planning failed")
        exec_result = self._preview_execute_wait(rname)
        if not exec_result.is_success():
            return exec_result

        return SkillResult.ok(f"Place complete — object released at ({x:.3f}, {y:.3f}, {z:.3f})")

    @skill
    def place_back(
        self, robot_name: str | None = None, arm: str = "auto"
    ) -> SkillResult[ManipulationSkillError]:
        """Place the held object back at its original pick position.

        Uses the position stored from the last successful pick operation.

        Args:
            robot_name: Robot to use (only needed for multi-arm setups).
            arm: Which arm — "left"/"right", or "auto" (default; the hand that picked).
        """
        if self._last_pick_pose is None:
            return SkillResult.fail(
                "NO_PRIOR_POSE",
                "No previous pick position stored — run pick() first",
            )

        p = self._last_pick_pose.position
        o = self._last_pick_pose.orientation
        logger.info(f"Placing back at original position ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})...")
        return self._place_with_orientation(p.x, p.y, p.z, o, robot_name, arm)

    @skill
    def drop_on(
        self,
        target_object_name: str,
        z_offset: float = 0.1,
        robot_name: str | None = None,
        arm: str = "auto",
    ) -> SkillResult[ManipulationSkillError]:
        """Drop a held object on top of a detected object.

        Resolves the target object's position with occlusion correction and
        places the held object above it.

        Args:
            target_object_name: Name of the target object to drop onto (e.g. "cup", "bowl").
            z_offset: Height above the target object's center to release (meters).
            robot_name: Robot to use (only needed for multi-arm setups).
            arm: Which arm — "left"/"right", or "auto" (default; the hand that picked).
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
        return self.place(x, y, z, robot_name, arm)

    @skill
    def pick_and_place(
        self,
        object_name: str,
        place_x: float,
        place_y: float,
        place_z: float,
        object_id: str | None = None,
        robot_name: str | None = None,
        arm: str = "auto",
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
            arm: Which arm — "left"/"right", or "auto" (default; pick uses the nearer
                arm, place uses that same hand).
        """
        logger.info(
            f"Starting pick and place: pick '{object_name}' → place at "
            f"({place_x:.3f}, {place_y:.3f}, {place_z:.3f})"
        )

        # Pick phase
        pick_result = self.pick(object_name, object_id, robot_name, arm)
        if not pick_result.is_success():
            return pick_result

        # Place phase (the same hand holds the object → arm="auto" uses _last_pick_arm)
        return self.place(place_x, place_y, place_z, robot_name)

    @rpc
    def stop(self) -> None:
        """Stop the pick-and-place module (cleanup GraspGen + delegate to base)."""
        logger.info("Stopping PickAndPlaceModule")

        # Stop GraspGen Docker container (thread-safe access to shared state)
        with self._lock:
            if self._graspgen is not None:
                self._graspgen.stop()
                self._graspgen = None

        super().stop()
