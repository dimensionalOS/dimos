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

"""G1 physical description, URDF frames, and sensor mounting."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.urdf_loader import UrdfLoader
from dimos.utils.data import LfsPath

# this is robot-specific, but only needed for the local_planner module
# generated via CMU' `pathGenerator` (autonomous_exploration_development_environment/local_planner)
# probably only needs to be regenerated on robots that are notably different than the g1 (the go2 in rage mode probably needs different local planning paths)
G1_LOCAL_PLANNER_PRECOMPUTED_PATHS = LfsPath("unitree_g1_local_planner_precomputed_paths")


@dataclass(frozen=True)
class G1Config:
    """Physical metadata used by G1 navigation and sensor blueprints."""

    name: str
    urdf: UrdfLoader
    height_clearance: float
    width_clearance: float
    # Lidar height above the ground when standing (used as the LIO init pose).
    # This is a stance value, not a kinematic mount, so it is NOT in the URDF.
    sensor_height: float

    @property
    def static_transforms(self) -> dict[str, Transform]:
        return self.urdf.static_transforms


G1 = G1Config(
    name="unitree_g1",
    urdf=UrdfLoader(name="unitree_g1", model_path=Path(__file__).parent / "g1.urdf"),
    height_clearance=1.2,
    width_clearance=0.6,
    sensor_height=1.2,
)
