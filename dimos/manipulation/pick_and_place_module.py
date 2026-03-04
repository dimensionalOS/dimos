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
from typing import TYPE_CHECKING, Any

from dimos.agents.annotation import skill
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.docker_runner import DockerModule as DockerRunner
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.stream import In
from dimos.manipulation.grasping.graspgen_module import GraspGenModule
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
)
from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.perception.detection.type.detection3d.object import (
    Object as DetObject,
)
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs import PoseArray
    from dimos.msgs.sensor_msgs import PointCloud2

logger = setup_logger()

# The host-side path (graspgen_visualization_output_path) is volume-mounted here.
_GRASPGEN_VIZ_CONTAINER_DIR = "/output/graspgen"
_GRASPGEN_VIZ_CONTAINER_PATH = f"{_GRASPGEN_VIZ_CONTAINER_DIR}/visualization.json"


class PickAndPlaceModuleConfig(ManipulationModuleConfig):
    """Configuration for PickAndPlaceModule (adds GraspGen settings)."""

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

    default_config = PickAndPlaceModuleConfig

    # Type annotation for the config attribute (mypy uses this)
    config: PickAndPlaceModuleConfig

    # Input: Objects from perception (for obstacle integration)
    objects: In[list[DetObject]]

    def __init__(self, global_config: GlobalConfig = global_config, **kwargs: Any) -> None:
        super().__init__(global_config, **kwargs)

        # GraspGen Docker runner (lazy initialized on first generate_grasps call)
        self._graspgen: DockerRunner | None = None

        # Last pick position: stored during pick so place_back() can return the object
        self._last_pick_position: Vector3 | None = None

        # Snapshotted detections from the last scan_objects/refresh call.
        # The live detection cache is volatile (labels change every frame),
        # so pick/place use this stable snapshot instead.
        self._detection_snapshot: list[DetObject] = []

    # =========================================================================
    # Lifecycle (perception integration)
    # =========================================================================

    @rpc
    def start(self) -> None:
        """Start the pick-and-place module (adds perception subscriptions)."""
        super().start()

        # Subscribe to objects port for perception obstacle integration
        if self.objects is not None:
            self.objects.observable().subscribe(self._on_objects)  # type: ignore[no-untyped-call]
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

    # =========================================================================
    # Perception RPC Methods
    # =========================================================================

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
    def clear_perception_obstacles(self) -> str:
        """Clear all perception obstacles from the planning world.

        Use this when the planner reports COLLISION_AT_START — detected objects
        may overlap the robot's current position and block planning.
        """
        if self._world_monitor is None:
            return "No world monitor available"
        count = self._world_monitor.clear_perception_obstacles()
        self._detection_snapshot = []
        return f"Cleared {count} perception obstacle(s) from planning world"

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

    # =========================================================================
    # GraspGen
    # =========================================================================

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

    # =========================================================================
    # Pick/Place Helpers
    # =========================================================================

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

        for det in self._detection_snapshot:
            if object_id and det.object_id == object_id:
                return det
            if object_name.lower() in det.name.lower() or det.name.lower() in object_name.lower():
                return det

        available = [det.name for det in self._detection_snapshot]
        logger.warning(f"Object '{object_name}' not found in snapshot. Available: {available}")
        return None

    def _generate_grasps_for_pick(
        self, object_name: str, object_id: str | None = None
    ) -> list[Pose] | None:
        """Generate grasp poses for an object.

        Computes a top-down approach grasp from the object's detected position.

        Args:
            object_name: Name of the object
            object_id: Optional object ID

        Returns:
            List of grasp poses (best first), or None if object not found
        """
        det = self._find_object_in_detections(object_name, object_id)
        if det is None:
            logger.warning(f"Object '{object_name}' not found in detections")
            return None

        c = det.center
        grasp_pose = Pose(Vector3(c.x, c.y, c.z), Quaternion.from_euler(Vector3(0.0, math.pi, 0.0)))
        logger.info(f"Heuristic grasp for '{object_name}' at ({c.x:.3f}, {c.y:.3f}, {c.z:.3f})")
        return [grasp_pose]

    # =========================================================================
    # Perception Skills
    # =========================================================================

    @skill
    def get_scene_info(self, robot_name: str | None = None) -> str:
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
            f"Perception: {perception.get('cached', 0)} cached, {perception.get('added', 0)} obstacles added"
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

        return "\n".join(lines)

    @skill
    def scan_objects(self, min_duration: float = 1.0, robot_name: str | None = None) -> str:
        """Scan the scene and list detected objects with their 3D positions.

        Refreshes perception obstacles from the latest sensor data and returns
        a formatted list of all detected objects.

        Args:
            min_duration: Minimum time in seconds to wait for stable detections.
            robot_name: Robot context (only needed for multi-arm setups).
        """
        obstacles = self.refresh_obstacles(min_duration)

        detections = self._detection_snapshot
        if not detections:
            return "No objects detected in scene"

        lines = [f"Detected {len(detections)} object(s):"]
        for det in detections:
            c = det.center
            lines.append(f"  - {det.name}: ({c.x:.3f}, {c.y:.3f}, {c.z:.3f})")

        if obstacles:
            lines.append(f"\n{len(obstacles)} obstacle(s) added to planning world")

        return "\n".join(lines)

    # =========================================================================
    # Long-Horizon Skills — Pick and Place
    # =========================================================================

    @skill
    def pick(
        self,
        object_name: str,
        object_id: str | None = None,
        robot_name: str | None = None,
    ) -> str:
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
            return "Error: Robot not found"
        rname, _, config, _ = robot
        pre_grasp_offset = config.pre_grasp_offset

        # 1. Generate grasps (uses already-cached detections — call scan_objects first)
        logger.info(f"Generating grasp poses for '{object_name}'...")
        grasp_poses = self._generate_grasps_for_pick(object_name, object_id)
        if not grasp_poses:
            return f"Error: No grasp poses found for '{object_name}'. Object may not be detected."

        # 2. Try each grasp candidate
        max_attempts = min(len(grasp_poses), 5)
        for i, grasp_pose in enumerate(grasp_poses[:max_attempts]):
            pre_grasp_pose = self._compute_pre_grasp_pose(grasp_pose, pre_grasp_offset)

            logger.info(f"Planning approach to pre-grasp (attempt {i + 1}/{max_attempts})...")
            if not self.plan_to_pose(pre_grasp_pose, rname):
                logger.info(f"Grasp candidate {i + 1} approach planning failed, trying next")
                continue  # Try next candidate

            # Open gripper before approach
            logger.info("Opening gripper...")
            self._set_gripper_position(0.85, rname)
            time.sleep(0.5)

            # 3. Preview + execute approach
            err = self._preview_execute_wait(rname)
            if err:
                return err

            # 4. Move to grasp pose
            logger.info("Moving to grasp position...")
            if not self.plan_to_pose(grasp_pose, rname):
                return "Error: Grasp pose planning failed"
            err = self._preview_execute_wait(rname)
            if err:
                return err

            # 5. Close gripper
            logger.info("Closing gripper...")
            self._set_gripper_position(0.0, rname)
            time.sleep(1.5)  # Wait for gripper to close

            # 6. Retract to pre-grasp
            logger.info("Retracting with object...")
            if not self.plan_to_pose(pre_grasp_pose, rname):
                return "Error: Retract planning failed"
            err = self._preview_execute_wait(rname)
            if err:
                return err

            # Store pick position so place_back() can return the object
            self._last_pick_position = grasp_pose.position

            return f"Pick complete — grasped '{object_name}' successfully"

        return f"Error: All {max_attempts} grasp attempts failed for '{object_name}'"

    @skill
    def place(
        self,
        x: float,
        y: float,
        z: float,
        robot_name: str | None = None,
    ) -> str:
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
            return "Error: Robot not found"
        rname, _, config, _ = robot
        pre_place_offset = config.pre_grasp_offset

        # Compute place pose (top-down approach)
        place_pose = Pose(Vector3(x, y, z), Quaternion.from_euler(Vector3(0.0, math.pi, 0.0)))
        pre_place_pose = self._compute_pre_grasp_pose(place_pose, pre_place_offset)

        # 1. Move to pre-place
        logger.info(f"Planning approach to place position ({x:.3f}, {y:.3f}, {z:.3f})...")
        if not self.plan_to_pose(pre_place_pose, rname):
            return "Error: Pre-place approach planning failed"

        err = self._preview_execute_wait(rname)
        if err:
            return err

        # 2. Lower to place position
        logger.info("Lowering to place position...")
        if not self.plan_to_pose(place_pose, rname):
            return "Error: Place pose planning failed"
        err = self._preview_execute_wait(rname)
        if err:
            return err

        # 3. Release
        logger.info("Releasing object...")
        self._set_gripper_position(0.85, rname)
        time.sleep(1.0)

        # 4. Retract
        logger.info("Retracting...")
        if not self.plan_to_pose(pre_place_pose, rname):
            return "Error: Retract planning failed"
        err = self._preview_execute_wait(rname)
        if err:
            return err

        return f"Place complete — object released at ({x:.3f}, {y:.3f}, {z:.3f})"

    @skill
    def place_back(self, robot_name: str | None = None) -> str:
        """Place the held object back at its original pick position.

        Uses the position stored from the last successful pick operation.

        Args:
            robot_name: Robot to use (only needed for multi-arm setups).
        """
        if self._last_pick_position is None:
            return "Error: No previous pick position stored — run pick() first"

        p = self._last_pick_position
        logger.info(f"Placing back at original position ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})...")
        return self.place(p.x, p.y, p.z, robot_name)

    @skill
    def pick_and_place(
        self,
        object_name: str,
        place_x: float,
        place_y: float,
        place_z: float,
        object_id: str | None = None,
        robot_name: str | None = None,
    ) -> str:
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
            f"Starting pick and place: pick '{object_name}' → place at ({place_x:.3f}, {place_y:.3f}, {place_z:.3f})"
        )

        # Pick phase
        result = self.pick(object_name, object_id, robot_name)
        if result.startswith("Error:"):
            return result

        # Place phase
        return self.place(place_x, place_y, place_z, robot_name)

    # =========================================================================
    # Lifecycle
    # =========================================================================

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


# Expose blueprint for declarative composition
pick_and_place_module = PickAndPlaceModule.blueprint
