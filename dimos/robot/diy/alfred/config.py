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

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.config import G1_LOCAL_PLANNER_PRECOMPUTED_PATHS

DEFAULT_ADDRESS = "172.6.2.20:11323"

# just as a starting point. May re-compute these later. In principle robot-specific
LOCAL_PLANNER_PRECOMPUTED_PATHS = G1_LOCAL_PLANNER_PRECOMPUTED_PATHS


@dataclass(frozen=True)
class AlfredConfig:
    """Physical metadata used by Alfred navigation and sensor blueprints."""

    name: str
    height_clearance: float
    width_clearance: float
    # Lidar mount pose relative to base (used as the LIO init pose). Alfred has no
    # URDF, so the static transform below is defined manually from this mount.
    sensor_mount: Pose

    @property
    def static_transforms(self) -> dict[str, Transform]:
        mount = self.sensor_mount
        return {
            "mid360_link": Transform(
                translation=Vector3(mount.x, mount.y, mount.z),
                rotation=mount.orientation,
                frame_id="base_link",
                child_frame_id="mid360_link",
            ),
        }


ALFRED = AlfredConfig(
    name="alfred",
    height_clearance=2.0,  # meters
    width_clearance=1.0,
    # Mid-360 lidar: a bit forward, and a bit to the right of base center, above ground.
    sensor_mount=Pose(0.20, -0.20, 0.30),
)
