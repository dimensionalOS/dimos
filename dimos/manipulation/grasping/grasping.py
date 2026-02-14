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
"""Grasping skill module

Provides @skill interface for agents and orchestrates the grasp generation pipeline:
perception (get pointcloud) to graspgen (generate grasps in Docker) to output grasps
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import quaternion_to_euler

if TYPE_CHECKING:
    from dimos.core.stream import Out
    from dimos.msgs.geometry_msgs import PoseArray
    from dimos.msgs.sensor_msgs import PointCloud2

logger = setup_logger()


class GraspingModule(Module):
    """Grasping skill and orchestrator module"""

    grasps: Out[PoseArray]

    rpc_calls: list[str] = [
        "ObjectSceneRegistrationModule.get_object_pointcloud_by_name",
        "ObjectSceneRegistrationModule.get_object_pointcloud_by_object_id",
        "ObjectSceneRegistrationModule.get_full_scene_pointcloud",
        "GraspGenModule.generate_grasps",
    ]

    @rpc
    def start(self) -> None:
        super().start()
        logger.info("GraspingModule started")

    @rpc
    def stop(self) -> None:
        super().stop()
        logger.info("GraspingModule stopped")

    @skill
    def generate_grasps(
        self,
        object_name: str = "object",
        object_id: str | None = None,
        filter_collisions: bool = True,
    ) -> str:
        """Generate grasp poses for the specified object.

        Args:
            object_name: Name of the object to grasp (e.g. "coke can", "cup", "bottle").
            object_id: Optional unique object ID from perception. If provided, uses this
                instead of object_name for lookup.
            filter_collisions: Whether to filter grasps that collide with scene geometry.

        """
        # Get object pointcloud from perception
        pc = self._get_object_pointcloud(object_name, object_id)
        if pc is None:
            msg = f"No pointcloud found for '{object_id or object_name}'"
            logger.warning(msg)
            return msg

        # Get scene pointcloud for collision filtering
        scene_pc = None
        if filter_collisions:
            scene_pc = self._get_scene_pointcloud(exclude_object_id=object_id)

        # Call GraspGenModule RPC (running in Docker)
        try:
            generate = self.get_rpc_calls("GraspGenModule.generate_grasps")
            result = generate(pc, scene_pc)
        except Exception as e:
            msg = f"Grasp generation failed: {e}"
            logger.error(msg)
            return msg

        if result is None or len(result.poses) == 0:
            msg = f"No grasps generated for '{object_name}'"
            logger.info(msg)
            return msg

        self.grasps.publish(result)
        logger.info(f"Generated {len(result.poses)} grasps for '{object_name}'")

        # Format result for agent/human
        return self._format_grasp_result(result, object_name)

    def _get_object_pointcloud(
        self, object_name: str, object_id: str | None = None
    ) -> PointCloud2 | None:
        """Fetch object pointcloud from perception."""
        try:
            if object_id is not None:
                get_pc = self.get_rpc_calls(
                    "ObjectSceneRegistrationModule.get_object_pointcloud_by_object_id"
                )
                return get_pc(object_id)  # type: ignore[no-any-return]

            get_pc = self.get_rpc_calls(
                "ObjectSceneRegistrationModule.get_object_pointcloud_by_name"
            )
            return get_pc(object_name)  # type: ignore[no-any-return]
        except Exception as e:
            logger.error(f"Failed to get object pointcloud: {e}")
            return None

    def _get_scene_pointcloud(self, exclude_object_id: str | None = None) -> PointCloud2 | None:
        """Fetch scene pointcloud from perception for collision filtering."""
        try:
            get_scene = self.get_rpc_calls(
                "ObjectSceneRegistrationModule.get_full_scene_pointcloud"
            )
            return get_scene(exclude_object_id=exclude_object_id)  # type: ignore[no-any-return]
        except Exception as e:
            logger.debug(f"Could not get scene pointcloud: {e}")
            return None

    def _format_grasp_result(self, grasps: PoseArray, object_name: str) -> str:
        """Format grasp result for agent/human consumption."""
        best = grasps.poses[0]
        pos = best.position
        rpy = quaternion_to_euler(best.orientation, degrees=True)
        return (
            f"Generated {len(grasps.poses)}"
            f"Best grasp: pos=({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}), "
            f"rpy=({rpy.x:.1f}, {rpy.y:.1f}, {rpy.z:.1f}) degrees"
        )


grasping_module = GraspingModule.blueprint
__all__ = ["GraspingModule", "grasping_module"]
