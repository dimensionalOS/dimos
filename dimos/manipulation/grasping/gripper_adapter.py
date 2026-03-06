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

"""Adapts grasp poses between gripper geometries (base frame -> TCP frame)."""

from __future__ import annotations

from dataclasses import dataclass

from dimos.msgs.geometry_msgs import Pose, Transform, Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

@dataclass(frozen=True)
class GripperGeometry:
    """Parallel-jaw gripper dimensions."""

    name: str
    tcp_offset: float  # Base-to-TCP distance along approach axis (m)
    grasp_depth_correction: float = 0.0  # Extra approach shift for contact (m)

GRIPPER_GEOMETRIES: dict[str, GripperGeometry] = {
    "robotiq_2f_140": GripperGeometry(name="robotiq_2f_140", tcp_offset=0.175),
    "ufactory_xarm": GripperGeometry(name="ufactory_xarm", tcp_offset=0.172),
    "franka_panda": GripperGeometry(name="franka_panda", tcp_offset=0.103),
}

class GripperAdapter:
    """Shifts grasp poses along the approach axis to convert base frame -> TCP frame."""

    def __init__(self, source: str = "robotiq_2f_140", target: str = "ufactory_xarm") -> None:
        self.source = GRIPPER_GEOMETRIES[source]
        self.target = GRIPPER_GEOMETRIES[target]

    def adapt_grasps(self, poses: list[Pose]) -> list[Pose]:
        """Shift each pose along its +Z (approach) axis by the TCP offset."""
        shift = self.source.tcp_offset + self.target.grasp_depth_correction
        adapted = [self._shift(p, shift) for p in poses]
        logger.info(
            f"[GripperAdapter] Adapted {len(adapted)}/{len(poses)} grasps "
            f"({self.source.name} -> {self.target.name}, shift={shift:.3f}m)"
        )
        return adapted

    @staticmethod
    def _shift(pose: Pose, distance: float) -> Pose:
        approach = Transform(
            translation=pose.position, rotation=pose.orientation,
        ).to_matrix()[:3, 2]
        offset = approach * distance
        return Pose(
            position=Vector3(
                x=pose.position.x + offset[0],
                y=pose.position.y + offset[1],
                z=pose.position.z + offset[2],
            ),
            orientation=pose.orientation,
        )
