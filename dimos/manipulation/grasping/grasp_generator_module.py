# Copyright 2026 Dimensional Inc.
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

"""Shared base for grasp generators an agent can ask for grasps on a scanned object."""

from __future__ import annotations

import math
from typing import Any

from dimos.core.module import Module
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.experimental.object_scene_registration_spec import ObjectSceneRegistrationSpec


def _approach_angle(pose: Pose, approach: Vector3 | None) -> float:
    """Angle, in degrees, between the way the gripper moves into a grasp and ``approach``.

    Args:
        pose: The grasp's tool tip pose.
        approach: Unit vector in the pose's frame, or None, which gives 0.
    """
    if approach is None:
        return 0.0
    # A pick's pre-grasp sits back along the tip's -Z, so the gripper travels
    # along the tip's +Z into the grasp.
    axis = pose.orientation.rotate_vector(Vector3(0.0, 0.0, 1.0))
    return math.degrees(math.acos(max(-1.0, min(1.0, axis.dot(approach)))))


class GraspGeneratorModule(Module, GraspGenSpec):
    """A grasp generator that also proposes grasps for a scanned object, by its id.

    Subclasses implement ``propose_grasps`` and expose ``propose_grasps_for`` as
    an RPC that calls ``_propose_for``. An agent reaches every module with
    ``propose_grasps_for`` through PickAndPlaceModule's propose_grasps skill.
    """

    # Looks up a scanned object's points; a stack without a scene has none.
    _scene: ObjectSceneRegistrationSpec | None = None

    def _propose_for(
        self, object_id: str, approach: list[float] | None, max_angle_deg: float, limit: int
    ) -> dict[str, Any]:
        """Propose grasps on a scanned object that come in from about one direction.

        The grasps are not checked for reach or collisions; pick_object checks
        the path when it runs one.

        Args:
            object_id: ID from the latest scan_objects call.
            approach: Unit vector [x, y, z] in the object point cloud's frame, the
                direction the gripper moves into the object. None keeps every grasp.
            max_angle_deg: Half-angle of the cone around ``approach``, degrees.
            limit: Most grasps to return, best score first.

        Returns ``generator`` (this module's name), ``object_id``, ``frame`` (the
        point cloud's frame, which the poses are in), ``grasps``, ``considered``
        (how many grasps the generator made) and ``nearest_angle_deg`` (the
        smallest angle among those, or None when there were none). Each grasp has
        ``rank`` (0 is best), ``score``, ``angle_deg``, ``position`` ([x, y, z],
        metres), ``orientation`` ([x, y, z, w]) and ``generator``.

        Raises ValueError for an object without points, a stack without a
        scene, an ``approach`` of zero length or a ``limit`` below 1.
        """
        if self._scene is None:
            raise ValueError("This stack has no scene to look objects up in")
        if limit < 1:
            raise ValueError(f"limit must be at least 1, got {limit}")
        direction = None if approach is None else Vector3(*approach).normalize()
        if direction is not None and not direction.length() > 0.0:
            raise ValueError(f"approach must be a direction, got {approach}")
        cloud = self._scene.get_object_pointcloud_by_object_id(object_id)
        if cloud is None or len(cloud) == 0:
            raise ValueError(
                f"No points for object {object_id}; use an id from the latest scan_objects call"
            )
        candidates = self.propose_grasps(cloud).candidates
        angles = [_approach_angle(c.pose, direction) for c in candidates]
        in_cone = [(c, a) for c, a in zip(candidates, angles, strict=True) if a <= max_angle_deg]
        best = sorted(in_cone, key=lambda pair: -pair[0].score)[:limit]
        name = self._generator_name()
        return {
            "generator": name,
            "object_id": object_id,
            "frame": cloud.frame_id,
            "grasps": [
                {
                    "rank": rank,
                    "score": float(c.score),
                    "angle_deg": angle,
                    "position": list(c.pose.position.to_tuple()),
                    "orientation": list(c.pose.orientation.to_tuple()),
                    "generator": name,
                }
                for rank, (c, angle) in enumerate(best)
            ],
            "nearest_angle_deg": min(angles, default=None),
            "considered": len(angles),
        }

    def _generator_name(self) -> str:
        """The name other modules call this one by."""
        return self.config.rpc_name or self.config.instance_name or type(self).__name__
