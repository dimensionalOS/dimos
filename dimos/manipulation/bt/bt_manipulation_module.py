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

"""BT-specific ManipulationModule with perception + GraspGen integration.

Extends ManipulationModule with the RPC methods the BT PickPlaceModule needs:
- Perception: objects stream, obstacle monitor, refresh_obstacles, list_cached_detections
- Grasping: generate_grasps (GraspGen Docker), visualize_grasps (MeshCat)

Unlike PickAndPlaceModule, this has NO @skill methods — only the BT PickPlaceModule
exposes skills to the agent, keeping a clean separation of concerns.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.docker_runner import DockerModule as DockerRunner
from dimos.core.stream import In
from dimos.manipulation.grasping.graspgen_module import GraspGenModule
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.perception.detection.type.detection3d.object import (
    Object as DetObject,
)
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseArray import PoseArray
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

logger = setup_logger()

_GRASPGEN_VIZ_CONTAINER_DIR = "/output/graspgen"
_GRASPGEN_VIZ_CONTAINER_PATH = f"{_GRASPGEN_VIZ_CONTAINER_DIR}/visualization.json"


class BTManipulationModuleConfig(ManipulationModuleConfig):
    """ManipulationModule config with GraspGen settings for BT workflow."""

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


class BTManipulationModule(ManipulationModule):
    """ManipulationModule extended with perception and GraspGen for BT workflow.

    Provides the RPC methods that BT PickPlaceModule calls:
    - refresh_obstacles, list_cached_detections (perception)
    - generate_grasps, visualize_grasps (grasping)

    No @skill methods — only BT PickPlaceModule registers skills with the agent.
    """

    default_config = BTManipulationModuleConfig
    config: BTManipulationModuleConfig

    # Perception input from ObjectSceneRegistrationModule
    objects: In[list[DetObject]]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._graspgen: DockerRunner | None = None
        self._detection_snapshot: list[DetObject] = []

    @rpc
    def start(self) -> None:
        super().start()

        # Subscribe to objects port for perception obstacle integration
        if self.objects is not None:
            self.objects.observable().subscribe(self._on_objects)  # type: ignore[no-untyped-call]
            logger.info("Subscribed to objects port (async)")

        # Start obstacle monitor for perception integration
        if self._world_monitor is not None:
            self._world_monitor.start_obstacle_monitor()

        logger.info("BTManipulationModule started")

    def _on_objects(self, objects: list[DetObject]) -> None:
        try:
            if self._world_monitor is not None:
                self._world_monitor.on_objects(objects)
        except Exception as e:
            logger.error(f"Exception in _on_objects: {e}")

    @rpc
    def stop(self) -> None:
        logger.info("Stopping BTManipulationModule")
        with self._lock:
            if self._graspgen is not None:
                self._graspgen.stop()
                self._graspgen = None
        super().stop()

    # Override parent's @skill go_home — remove skill registration so only
    # the BT PickPlaceModule's go_home is exposed to the agent.
    def go_home(self, robot_name: str | None = None) -> str:
        """Move to home position (RPC only, not registered as agent skill)."""
        return super().go_home(robot_name)

    @rpc
    def refresh_obstacles(self, min_duration: float = 0.0) -> list[dict[str, Any]]:
        """Refresh perception obstacles and snapshot detections."""
        if self._world_monitor is None:
            return []
        result = self._world_monitor.refresh_obstacles(min_duration)
        self._detection_snapshot = self._world_monitor.get_cached_objects()
        logger.info(f"Detection snapshot: {[d.name for d in self._detection_snapshot]}")
        return result

    @rpc
    def clear_perception_obstacles(self) -> str:
        """Clear all perception obstacles from the planning world."""
        if self._world_monitor is None:
            return "No world monitor available"
        count = self._world_monitor.clear_perception_obstacles()
        self._detection_snapshot = []
        return f"Cleared {count} perception obstacle(s) from planning world"

    @rpc
    def list_cached_detections(self) -> list[dict[str, Any]]:
        """List cached detections from perception."""
        if self._world_monitor is None:
            return []
        return self._world_monitor.list_cached_detections()

    def _get_graspgen(self) -> DockerRunner:
        """Get or create GraspGen Docker module (lazy init, thread-safe)."""
        if self._graspgen is not None:
            return self._graspgen

        with self._lock:
            if self._graspgen is not None:
                return self._graspgen

            get_data("models_graspgen")

            docker_file = (
                DIMOS_PROJECT_ROOT
                / "dimos"
                / "manipulation"
                / "grasping"
                / "docker_context"
                / "Dockerfile"
            )

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
                docker_env={"CI": "1"},
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
            self._graspgen = graspgen
            return self._graspgen

    @rpc
    def generate_grasps(
        self,
        pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2 | None = None,
    ) -> PoseArray | None:
        """Generate grasp poses via GraspGen Docker module."""
        try:
            graspgen = self._get_graspgen()
            return graspgen.generate_grasps(pointcloud, scene_pointcloud, rpc_timeout=300)  # type: ignore[no-any-return]
        except Exception as e:
            logger.error(f"Grasp generation failed: {e}")
            return None

    @rpc
    def visualize_grasps(
        self,
        poses: list[Pose],
        max_grasps: int = 100,
    ) -> bool:
        """Render grasp candidates as gripper wireframes in MeshCat."""
        import numpy as np
        from pydrake.geometry import Box, Rgba
        from pydrake.math import RigidTransform

        from dimos.manipulation.grasping import visualize_grasps as viz_grasps
        from dimos.msgs.geometry_msgs.Transform import Transform

        assert self._world_monitor is not None, "WorldMonitor is not initialized"
        meshcat = self._world_monitor.world._meshcat  # type: ignore[attr-defined]

        W = viz_grasps.GRIPPER_WIDTH / 2.0
        FL = viz_grasps.FINGER_LENGTH
        PD = viz_grasps.PALM_DEPTH
        TUBE = 0.004

        parts = [
            ("palm", Box(W * 2, TUBE, TUBE), [0, 0, -FL]),
            ("left", Box(TUBE, TUBE, FL * 1.25), [-W, 0, -0.375 * FL]),
            ("right", Box(TUBE, TUBE, FL * 1.25), [W, 0, -0.375 * FL]),
            ("approach", Box(TUBE, TUBE, PD), [0, 0, -FL - PD / 2]),
        ]

        meshcat.Delete("grasps")

        num = min(len(poses), max_grasps)
        for i in range(num):
            t = i / max(num - 1, 1)
            rgba = Rgba(min(1.0, 2 * t), max(0.0, 1.0 - t), 0.0, 0.8)
            grasp_mat = Transform(
                translation=poses[i].position,
                rotation=poses[i].orientation,
            ).to_matrix()
            for name, shape, xyz in parts:
                local = np.eye(4)
                local[:3, 3] = xyz
                path = f"grasps/grasp_{i}/{name}"
                meshcat.SetObject(path, shape, rgba)
                meshcat.SetTransform(path, RigidTransform(grasp_mat @ local))

        logger.info(f"Visualized {num}/{len(poses)} grasp poses in MeshCat")
        return True


bt_manipulation_module = BTManipulationModule.blueprint
