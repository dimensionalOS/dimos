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

"""G1 physical description and sensor odometry offsets."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


@dataclass(frozen=True)
class G1Config:
    """Physical metadata used by G1 navigation and sensor blueprints."""

    name: str
    model_path: Path
    height_clearance: float
    width_clearance: float
    internal_odom_offsets: dict[str, Any] = field(default_factory=dict)


G1 = G1Config(
    name="unitree_g1",
    model_path=Path(__file__).parent / "g1.urdf",
    height_clearance=1.2,
    width_clearance=0.6,
    internal_odom_offsets={
        # Mid-360 lidar: 1.2 m above ground.
        "mid360_link": Pose(0.0, 0.0, 1.2, *Quaternion.from_euler(Vector3(0, 0, 0))),
    },
)
