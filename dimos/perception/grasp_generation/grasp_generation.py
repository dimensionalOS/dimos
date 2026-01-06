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
Dimensional-hosted grasp generation for manipulation pipeline.
"""

import asyncio

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.perception.grasp_generation.utils import parse_grasp_results
from dimos.types.manipulation import ObjectData
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class HostedGraspGenerator:
    """
    Dimensional-hosted grasp generator using WebSocket communication.
    """

    def __init__(self, server_url: str) -> None:
        """
        Initialize Dimensional-hosted grasp generator.

        Args:
            server_url: WebSocket URL for Dimensional-hosted grasp generator server
        """
        self.server_url = server_url
        logger.info(f"Initialized grasp generator with server: {server_url}")

    def generate_grasps_from_objects(
        self, objects: list[ObjectData], full_pcd: o3d.geometry.PointCloud
    ) -> list[dict]:  # type: ignore[type-arg]
        """
        Generate grasps from ObjectData objects using grasp generator.

        Args:
            objects: List of ObjectData with point clouds
            full_pcd: Open3D point cloud of full scene

        Returns:
            Parsed grasp results as list of dictionaries
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
                if "colors_numpy" in obj and obj["colors_numpy"] is not None:  # type: ignore[typeddict-item]
                    colors = obj["colors_numpy"]  # type: ignore[typeddict-item]
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
                return []

            # Combine point clouds
            combined_points = np.vstack(all_points)
            combined_colors = None
            if len(all_colors) == valid_objects and len(all_colors) > 0:
                combined_colors = np.vstack(all_colors)

            # Send grasp request
            grasps = self._send_grasp_request_sync(combined_points, combined_colors)

            if not grasps:
                return []

            # Parse and return results in list of dictionaries format
            return parse_grasp_results(grasps)

        except Exception as e:
            logger.error(f"Grasp generation failed: {e}")
            return []

    def _send_grasp_request_sync(
        self,
        points: np.ndarray,  # type: ignore[type-arg]
        colors: np.ndarray | None,  # type: ignore[type-arg]
    ) -> list[dict] | None:  # type: ignore[type-arg]
        """Send synchronous grasp request to grasp server."""

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
        self,
        points: np.ndarray,  # type: ignore[type-arg]
        colors: np.ndarray,  # type: ignore[type-arg]
    ) -> list[dict] | None:  # type: ignore[type-arg]
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
                elif isinstance(grasps, int | float) and grasps == 0:
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

    def _convert_grasp_format(self, grasps: list[dict]) -> list[dict]:  # type: ignore[type-arg]
        """Convert Dimensional Grasp format to visualization format."""
        converted = []

        for i, grasp in enumerate(grasps):
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

    def _rotation_matrix_to_euler(self, rotation_matrix: np.ndarray) -> dict[str, float]:  # type: ignore[type-arg]
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

    def cleanup(self) -> None:
        """Clean up resources."""
        logger.info("Grasp generator cleaned up")
