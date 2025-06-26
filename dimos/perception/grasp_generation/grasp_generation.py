# Copyright 2025 Dimensional Inc.
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

"""
ContactGraspNet-based grasp generation for manipulation pipeline.
"""

import os
import sys
import time
import asyncio
import numpy as np
import torch
import open3d as o3d
from typing import Dict, List, Optional, Tuple, Any, Union

from dimos.types.manipulation import ObjectData
from dimos.utils.logging_config import setup_logger

from dimos.models.manipulation.contact_graspnet_pytorch.contact_graspnet_pytorch.contact_grasp_estimator import (
    GraspEstimator,
)
from dimos.models.manipulation.contact_graspnet_pytorch.contact_graspnet_pytorch.checkpoints import (
    CheckpointIO,
)
from dimos.models.manipulation.contact_graspnet_pytorch.contact_graspnet_pytorch import config_utils
from dimos.perception.grasp_generation.utils import (
    parse_anygrasp_results,
    parse_contactgraspnet_results,
)

logger = setup_logger("dimos.perception.grasp_generation")


class GraspGeneratorFactory:
    """Factory for creating grasp generators."""

    @staticmethod
    def create_generator(grasp_model: str, **kwargs):
        """
        Create a grasp generator.

        Args:
            grasp_model: "contactgraspnet" or "anygrasp"
            **kwargs: Parameters for the specific generator

        Returns:
            Grasp generator instance
        """
        if grasp_model.lower() == "contactgraspnet":
            return ContactGraspNetGenerator(**kwargs)
        elif grasp_model.lower() == "anygrasp":
            server_url = kwargs.get("server_url")
            if not server_url:
                raise ValueError("AnyGrasp requires server_url parameter")
            return AnyGraspGenerator(server_url)
        else:
            raise ValueError(f"Unknown grasp model: {grasp_model}")


class AnyGraspGenerator:
    """
    AnyGrasp-based grasp generator using WebSocket communication.
    """

    def __init__(self, server_url: str):
        """
        Initialize AnyGrasp generator.

        Args:
            server_url: WebSocket URL for AnyGrasp server
        """
        self.server_url = server_url
        logger.info(f"Initialized AnyGrasp generator with server: {server_url}")

    def generate_grasps_from_objects(
        self, objects: List[ObjectData], full_pcd: o3d.geometry.PointCloud
    ) -> Dict:
        """
        Generate grasps from ObjectData objects using AnyGrasp.

        Args:
            objects: List of ObjectData with point clouds
            full_pcd: Open3D point cloud of full scene

        Returns:
            Parsed grasp results in unified format
        """
        try:
            # Combine all point clouds
            all_points = []
            all_colors = []
            valid_objects = 0

            for obj in objects:
                if "point_cloud_numpy" not in obj or obj["point_cloud_numpy"] is None:
                    continue

                points = obj["point_cloud_numpy"]
                if not isinstance(points, np.ndarray) or points.size == 0:
                    continue

                if len(points.shape) != 2 or points.shape[1] != 3:
                    continue

                colors = None
                if "colors_numpy" in obj and obj["colors_numpy"] is not None:
                    colors = obj["colors_numpy"]
                    if isinstance(colors, np.ndarray) and colors.size > 0:
                        if (
                            colors.shape[0] != points.shape[0]
                            or len(colors.shape) != 2
                            or colors.shape[1] != 3
                        ):
                            colors = None

                all_points.append(points)
                if colors is not None:
                    all_colors.append(colors)
                valid_objects += 1

            if not all_points:
                return {}

            # Combine point clouds
            combined_points = np.vstack(all_points)
            combined_colors = None
            if len(all_colors) == valid_objects and len(all_colors) > 0:
                combined_colors = np.vstack(all_colors)
            # Send grasp request
            grasps = self._send_grasp_request_sync(combined_points, combined_colors)

            if not grasps:
                return {}

            # Parse and return results in unified format
            return parse_anygrasp_results(grasps)

        except Exception as e:
            logger.error(f"AnyGrasp generation failed: {e}")
            return {}

    def _send_grasp_request_sync(
        self, points: np.ndarray, colors: Optional[np.ndarray]
    ) -> Optional[List[Dict]]:
        """Send synchronous grasp request to AnyGrasp server."""

        try:
            # Prepare colors
            colors = np.ones((points.shape[0], 3), dtype=np.float32) * 0.5

            # Ensure correct data types
            points = points.astype(np.float32)
            colors = colors.astype(np.float32)

            # Validate ranges
            if np.any(np.isnan(points)) or np.any(np.isinf(points)):
                logger.error("Points contain NaN or Inf values")
                return None
            if np.any(np.isnan(colors)) or np.any(np.isinf(colors)):
                logger.error("Colors contain NaN or Inf values")
                return None

            colors = np.clip(colors, 0.0, 1.0)

            # Run async request in sync context
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                result = loop.run_until_complete(self._async_grasp_request(points, colors))
                return result
            finally:
                loop.close()

        except Exception as e:
            logger.error(f"Error in synchronous grasp request: {e}")
            return None

    async def _async_grasp_request(
        self, points: np.ndarray, colors: np.ndarray
    ) -> Optional[List[Dict]]:
        """Async grasp request helper."""
        import json
        import websockets

        try:
            async with websockets.connect(self.server_url) as websocket:
                request = {
                    "points": points.tolist(),
                    "colors": colors.tolist(),
                    "lims": [-1.0, 1.0, -1.0, 1.0, 0.0, 2.0],
                }

                await websocket.send(json.dumps(request))
                response = await websocket.recv()
                grasps = json.loads(response)

                if isinstance(grasps, dict) and "error" in grasps:
                    logger.error(f"Server returned error: {grasps['error']}")
                    return None
                elif isinstance(grasps, (int, float)) and grasps == 0:
                    return None
                elif not isinstance(grasps, list):
                    logger.error(f"Server returned unexpected response type: {type(grasps)}")
                    return None
                elif len(grasps) == 0:
                    return None

                return self._convert_grasp_format(grasps)

        except Exception as e:
            logger.error(f"Async grasp request failed: {e}")
            return None

    def _convert_grasp_format(self, anygrasp_grasps: List[dict]) -> List[dict]:
        """Convert AnyGrasp format to visualization format."""
        converted = []

        for i, grasp in enumerate(anygrasp_grasps):
            rotation_matrix = np.array(grasp.get("rotation_matrix", np.eye(3)))
            euler_angles = self._rotation_matrix_to_euler(rotation_matrix)

            converted_grasp = {
                "id": f"grasp_{i}",
                "score": grasp.get("score", 0.0),
                "width": grasp.get("width", 0.0),
                "height": grasp.get("height", 0.0),
                "depth": grasp.get("depth", 0.0),
                "translation": grasp.get("translation", [0, 0, 0]),
                "rotation_matrix": rotation_matrix.tolist(),
                "euler_angles": euler_angles,
            }
            converted.append(converted_grasp)

        converted.sort(key=lambda x: x["score"], reverse=True)
        return converted

    def _rotation_matrix_to_euler(self, rotation_matrix: np.ndarray) -> Dict[str, float]:
        """Convert rotation matrix to Euler angles (in radians)."""
        sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)

        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            y = np.arctan2(-rotation_matrix[2, 0], sy)
            z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            y = np.arctan2(-rotation_matrix[2, 0], sy)
            z = 0

        return {"roll": x, "pitch": y, "yaw": z}

    def cleanup(self):
        """Clean up resources."""
        logger.info("AnyGrasp generator cleaned up")


class ContactGraspNetGenerator:
    """
    ContactGraspNet-based grasp generator for manipulation pipeline.

    Takes ObjectData with point clouds from pointcloud filtering and generates
    6-DoF grasp poses using ContactGraspNet.
    """

    def __init__(
        self,
        checkpoint_dir: str = "dimos/models/manipulation/contact_graspnet_pytorch/checkpoints",
        local_regions: bool = True,
        filter_grasps: bool = True,
        forward_passes: int = 1,
        z_range: List[float] = [0.2, 1.8],
        min_points_for_grasp: int = 100,
    ):
        """
        Initialize ContactGraspNet grasp generator.

        Args:
            checkpoint_dir: Path to ContactGraspNet checkpoint directory
            device: Device to use ('cuda', 'cpu', or None for auto)
            local_regions: Crop 3D local regions around object segments for prediction
            filter_grasps: Filter grasp contacts to lie within object segments
            forward_passes: Number of forward passes for prediction
            z_range: Z distance range [min, max] in meters to filter point clouds
            min_points_for_grasp: Minimum points required for grasp generation
            max_objects: Maximum number of objects to process for grasps
        """
        self.checkpoint_dir = checkpoint_dir
        self.local_regions = local_regions
        self.filter_grasps = filter_grasps
        self.forward_passes = forward_passes
        self.z_range = z_range
        self.min_points_for_grasp = min_points_for_grasp

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        logger.info(f"Initializing ContactGraspNet on device: {self.device}")

        # Load configuration
        self._load_config()

        # Initialize grasp estimator
        self._initialize_model()

        logger.info("ContactGraspNet grasp generator initialized successfully")

    def _load_config(self):
        """Load ContactGraspNet configuration."""
        try:
            self.global_config = config_utils.load_config(
                self.checkpoint_dir, batch_size=self.forward_passes
            )
            logger.info(f"Loaded config from {self.checkpoint_dir}")
        except Exception as e:
            logger.error(f"Failed to load config: {e}")
            raise

    def _initialize_model(self):
        """Initialize the ContactGraspNet model."""
        try:
            self.grasp_estimator = GraspEstimator(self.global_config)
            logger.info("ContactGraspNet model initialized")
        except Exception as e:
            logger.error(f"Failed to initialize model: {e}")
            raise

    def generate_grasps_from_objects(
        self, objects: List[ObjectData], full_pcd: o3d.geometry.PointCloud
    ) -> Dict:
        """
        Generate grasps from ObjectData objects.

        Args:
            objects: List of ObjectData with point clouds from pointcloud filtering
            full_pcd: Open3D point cloud of full scene

        Returns:
            Parsed grasp results in unified format
        """
        try:
            start_time = time.time()

            # Convert ObjectData to point cloud format
            full_pc = np.asarray(full_pcd.points)
            pc_segments = {}

            # Build pc_segments dictionary as expected by ContactGraspNet
            for i, obj in enumerate(objects):
                object_id = obj.get("object_id", i)
                pc_segments[object_id] = obj["point_cloud_numpy"]

            # Generate grasps using ContactGraspNet
            pred_grasps_cam, scores, contact_pts, gripper_openings = (
                self.grasp_estimator.predict_scene_grasps(
                    pc_full=full_pc,
                    pc_segments=pc_segments,
                    local_regions=self.local_regions,
                    filter_grasps=self.filter_grasps,
                    forward_passes=self.forward_passes,
                )
            )

            processing_time = time.time() - start_time

            # Log results
            total_grasps = (
                sum(len(grasps) for grasps in pred_grasps_cam.values()) if pred_grasps_cam else 0
            )
            logger.info(
                f"Generated {total_grasps} grasps across {len(pred_grasps_cam)} objects in {processing_time:.2f}s"
            )

            # Parse and return results in unified format
            return parse_contactgraspnet_results(
                pred_grasps_cam, scores, contact_pts, gripper_openings
            )

        except Exception as e:
            logger.error(f"Grasp generation failed: {e}")
            return {}

    def cleanup(self):
        """Clean up resources."""
        if hasattr(self, "grasp_estimator"):
            del self.grasp_estimator
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        logger.info("ContactGraspNet grasp generator cleaned up")
