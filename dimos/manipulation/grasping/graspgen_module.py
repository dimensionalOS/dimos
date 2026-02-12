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
from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import sys
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.docker_runner import DockerModuleConfig
from dimos.core.module import Module
from dimos.msgs.geometry_msgs import PoseArray
from dimos.msgs.std_msgs import Header
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose

if TYPE_CHECKING:
    from dimos.core.stream import Out
    from dimos.msgs.sensor_msgs import PointCloud2

logger = setup_logger()

# Inference constants
MIN_POINTS_FOR_INFERENCE = 50
OUTLIER_REMOVAL_THRESHOLD = 100
COLLISION_FILTER_THRESHOLD = 0.02


@dataclass
class GraspGenConfig(DockerModuleConfig):
    """Configuration for GraspGen module."""

    # Docker defaults
    docker_image: str = "dimos-graspgen:latest"
    docker_gpus: str = "all"
    docker_shm_size: str = "4g"

    # GraspGen settings
    gripper_type: str = (
        "robotiq_2f_140"  # use any from robotiq_2f_140", "franka_panda", "single_suction_cup_30mm"
    )
    num_grasps: int = 400
    topk_num_grasps: int = 100
    grasp_threshold: float = -1.0
    filter_collisions: bool = False
    save_visualization_data: bool = False
    visualization_output_path: str = "/tmp/grasp_visualization.json"


class GraspGenModule(Module[GraspGenConfig]):
    """Grasp generation module running in Docker."""

    default_config = GraspGenConfig
    grasps: Out[PoseArray]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._sampler = self._gripper_info = None
        self._initialized = False

    @rpc
    def start(self) -> None:
        super().start()
        if not self._initialize_graspgen():
            raise RuntimeError("Failed to initialize GraspGen")
        logger.info(f"GraspGenModule started (gripper={self.config.gripper_type})")

    @rpc
    def stop(self) -> None:
        self._sampler = self._gripper_info = None
        self._initialized = False
        super().stop()

    @rpc
    def generate_grasps(
        self,
        pointcloud: PointCloud2,
        scene_pointcloud: PointCloud2 | None = None,
    ) -> PoseArray | None:
        """Generate grasp poses for the given pointcloud."""
        try:
            points = self._extract_points(pointcloud)
            if len(points) < 10:
                return None

            # Run inference (with optional collision filtering)
            scene_points = None
            if scene_pointcloud is not None and self.config.filter_collisions:
                scene_points = self._extract_points(scene_pointcloud)
            grasps, scores = self._run_inference(points, scene_points)
            if len(grasps) == 0:
                return None

            # Convert and publish results
            pose_array = self._grasps_to_pose_array(grasps, scores, pointcloud.frame_id)
            self.grasps.publish(pose_array)

            if self.config.save_visualization_data:
                self._save_visualization_data(points, grasps, scores, pointcloud.frame_id)
            return pose_array
        except Exception as e:
            logger.error(f"Grasp generation failed: {e}")
            return None

    def _initialize_graspgen(self) -> bool:
        """Load GraspGen model and gripper info. Returns True on success."""
        if self._initialized:
            return True

        try:
            # Setup GraspGen path and environment (must be set by Dockerfile)
            graspgen_path = os.environ.get("GRASPGEN_PATH")
            if graspgen_path is None:
                raise RuntimeError(
                    "GRASPGEN_PATH environment variable not set. Ensure Dockerfile sets ENV GRASPGEN_PATH."
                )
            if graspgen_path not in sys.path:
                sys.path.insert(0, graspgen_path)
            os.environ["PYOPENGL_PLATFORM"] = "egl"

            # Load model and gripper (Docker-only imports)
            from grasp_gen.grasp_server import (  # type: ignore[import-not-found]
                GraspGenSampler,
                load_grasp_cfg,
            )
            from grasp_gen.robot import get_gripper_info  # type: ignore[import-not-found]

            grasp_cfg = load_grasp_cfg(self._get_gripper_config_path())
            self._sampler = GraspGenSampler(grasp_cfg)
            self._gripper_info = get_gripper_info(self.config.gripper_type)
            self._initialized = True
            logger.info("GraspGen initialized")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize GraspGen: {e}")
            self._sampler = self._gripper_info = None
            return False

    def _get_gripper_config_path(self) -> str:
        graspgen_path = os.environ.get("GRASPGEN_PATH")
        if graspgen_path is None:
            raise RuntimeError("GRASPGEN_PATH environment variable not set")
        config_name = f"graspgen_{self.config.gripper_type}.yml"

        for subdir in ("GraspGenModels/checkpoints", "checkpoints"):
            path = os.path.join(graspgen_path, subdir, config_name)
            if os.path.exists(path):
                return path

        return os.path.join(graspgen_path, "checkpoints", config_name)

    def _run_inference(
        self, object_pc: np.ndarray[Any, Any], scene_pc: np.ndarray[Any, Any] | None = None
    ) -> tuple[np.ndarray[Any, Any], np.ndarray[Any, Any]]:
        if self._sampler is None:
            return np.array([]), np.array([])

        from grasp_gen.grasp_server import GraspGenSampler  # type: ignore[import-not-found]
        from grasp_gen.utils.point_cloud_utils import (  # type: ignore[import-not-found]
            filter_colliding_grasps,
            point_cloud_outlier_removal,
        )
        import torch  # type: ignore[import-not-found]
        import trimesh.transformations as tra  # type: ignore[import-not-found]

        pc_torch = torch.from_numpy(object_pc)

        if len(object_pc) > OUTLIER_REMOVAL_THRESHOLD:
            pc_filtered, _ = point_cloud_outlier_removal(pc_torch)
            object_pc_filtered = pc_filtered.numpy()
            if len(object_pc_filtered) < MIN_POINTS_FOR_INFERENCE:
                object_pc_filtered = object_pc
        else:
            object_pc_filtered = object_pc

        if len(object_pc_filtered) < MIN_POINTS_FOR_INFERENCE:
            return np.array([]), np.array([])

        grasps, scores = GraspGenSampler.run_inference(
            object_pc_filtered,
            self._sampler,
            grasp_threshold=self.config.grasp_threshold,
            num_grasps=self.config.num_grasps,
            topk_num_grasps=self.config.topk_num_grasps,
            remove_outliers=False,
        )

        if len(grasps) == 0:
            return np.array([]), np.array([])

        grasps_np = grasps.cpu().numpy()
        scores_np = scores.cpu().numpy()

        if self.config.filter_collisions and scene_pc is not None:
            if self._gripper_info is None:
                return grasps_np, scores_np

            pc_mean = object_pc_filtered.mean(axis=0)
            T_center = tra.translation_matrix(-pc_mean)
            grasps_centered = np.array([T_center @ g for g in grasps_np])
            scene_pc_centered = tra.transform_points(scene_pc, T_center)

            collision_free_mask = filter_colliding_grasps(
                scene_pc=scene_pc_centered,
                grasp_poses=grasps_centered,
                gripper_collision_mesh=self._gripper_info.collision_mesh,
                collision_threshold=COLLISION_FILTER_THRESHOLD,
            )
            grasps_np = grasps_np[collision_free_mask]
            scores_np = scores_np[collision_free_mask]

        return grasps_np, scores_np

    def _extract_points(self, msg: PointCloud2) -> np.ndarray[Any, Any]:
        points = msg.points().numpy()  # type: ignore[no-untyped-call]
        if not np.isfinite(points).all():
            raise ValueError("Point cloud contains NaN/Inf")
        return points  # type: ignore[no-any-return]

    def _grasps_to_pose_array(
        self, grasps: np.ndarray[Any, Any], scores: np.ndarray[Any, Any], frame_id: str
    ) -> PoseArray:
        sorted_indices = np.argsort(scores)[::-1]
        poses = [matrix_to_pose(grasps[idx]) for idx in sorted_indices]
        return PoseArray(header=Header(frame_id), poses=poses)

    def _save_visualization_data(
        self,
        points: np.ndarray[Any, Any],
        grasps: np.ndarray[Any, Any],
        scores: np.ndarray[Any, Any],
        frame_id: str,
    ) -> None:
        import json

        try:
            data = {
                "point_cloud": points.tolist(),
                "grasps": [g.tolist() for g in grasps],
                "scores": scores.tolist(),
                "frame_id": frame_id,
                "timestamp": time.time(),
            }
            output_path = Path(self.config.visualization_output_path)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with open(output_path, "w") as f:
                json.dump(data, f)
        except Exception as e:
            logger.warning(f"Failed to save visualization: {e}")


def graspgen(
    docker_file_path: Path | str, docker_build_context: Path | str | None = None, **kwargs: Any
) -> Any:
    """Create a GraspGen module blueprint. All kwargs passed through to config."""
    dockerfile = Path(docker_file_path)
    build_context = Path(docker_build_context) if docker_build_context else dockerfile.parent
    return GraspGenModule.blueprint(
        docker_file=dockerfile, docker_build_context=build_context, **kwargs
    )


__all__ = ["GraspGenConfig", "GraspGenModule", "graspgen"]
