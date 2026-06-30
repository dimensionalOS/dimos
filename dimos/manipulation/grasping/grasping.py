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
from dimos.core.stream import Out
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec, TSDFGraspGenSpec
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.perception.object_scene_registration_spec import ObjectSceneRegistrationSpec
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import quaternion_to_euler

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

logger = setup_logger()


class GraspingModule(Module):
    """Grasping skill and orchestrator module"""

    grasps: Out[PoseArray]

    _scene_registration: ObjectSceneRegistrationSpec
    _grasp_gen: GraspGenSpec | None = None
    _tsdf_grasp_gen: TSDFGraspGenSpec | None = None

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

        # Call GraspGenModule (running in Docker)
        try:
            if self._grasp_gen is None:
                msg = "Pointcloud grasp generator is not available."
                logger.warning(msg)
                return msg
            result = self._grasp_gen.generate_grasps(pc, scene_pc)
        except Exception as e:
            msg = f"Grasp generation failed: {e}"
            logger.error(msg)
            return msg

        if result is None or len(result.poses) == 0:
            msg = f"Pointcloud grasp generator returned no grasps for '{object_name}'."
            logger.info(msg)
            return msg

        self.grasps.publish(result)
        logger.info(f"Generated {len(result.poses)} grasps for '{object_name}'")

        # Format result for agent/human
        return self._format_grasp_result(result, object_name)

    @skill
    def generate_grasps_for_object(self, object_id: str, cushion_m: float = 0.03) -> str:
        """Generate TSDF grasp candidates for a specific registered object id.

        Args:
            object_id: Stable runtime object id returned by perception after detection.
            cushion_m: Extra padding around object bounds in meters.
        """
        if self._tsdf_grasp_gen is None:
            msg = "TSDF grasp generator is not available."
            logger.warning(msg)
            return msg

        try:
            target = self._scene_registration.get_object_by_object_id(object_id)
        except Exception as exc:
            msg = f"Failed to look up registered object '{object_id}': {exc}"
            logger.error(msg)
            return msg

        if target is None:
            msg = f"No registered object found with object_id '{object_id}'."
            logger.warning(msg)
            return msg

        try:
            candidates = self._tsdf_grasp_gen.generate_grasps_for_target_bounds(
                target_center=target.center,
                target_size=target.size,
                target_frame_id=target.frame_id,
                target_ts=target.ts,
                cushion_m=cushion_m,
            )
        except Exception as exc:
            msg = f"Target-conditioned grasp generation failed for '{object_id}': {exc}"
            logger.error(msg)
            return msg

        if candidates is None:
            msg = f"No target-conditioned grasps generated for '{target.name}' ({object_id})."
            logger.info(msg)
            return msg
        if len(candidates) == 0:
            msg = f"VGN returned no target-conditioned grasps for '{target.name}' ({object_id})."
            logger.info(msg)
            return msg

        poses = candidates.to_pose_array()
        self.grasps.publish(poses)
        logger.info(
            "Generated %s target-conditioned grasps for '%s' (%s)",
            len(candidates),
            target.name,
            object_id,
        )
        return self._format_grasp_result(poses, target.name)

    def _get_object_pointcloud(
        self, object_name: str, object_id: str | None = None
    ) -> PointCloud2 | None:
        """Fetch object pointcloud from perception."""
        try:
            if object_id is not None:
                return self._scene_registration.get_object_pointcloud_by_object_id(object_id)

            return self._scene_registration.get_object_pointcloud_by_name(object_name)
        except Exception as e:
            logger.error(f"Failed to get object pointcloud: {e}")
            return None

    def _get_scene_pointcloud(self, exclude_object_id: str | None = None) -> PointCloud2 | None:
        """Fetch scene pointcloud from perception for collision filtering."""
        try:
            return self._scene_registration.get_full_scene_pointcloud(
                exclude_object_id=exclude_object_id
            )
        except Exception as e:
            logger.debug(f"Could not get scene pointcloud: {e}")
            return None

    def _format_grasp_result(self, grasps: PoseArray, object_name: str) -> str:
        """Format grasp result for agent/human consumption."""
        best = grasps.poses[0]
        pos = best.position
        rpy = quaternion_to_euler(best.orientation, degrees=True)
        return (
            f"Generated {len(grasps.poses)} grasp(s). "
            f"Best grasp: pos=({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}), "
            f"rpy=({rpy.x:.1f}, {rpy.y:.1f}, {rpy.z:.1f}) degrees"
        )
