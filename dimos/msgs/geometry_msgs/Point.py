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

from dimos_lcm.geometry_msgs import Point as LCMPoint


class Point(LCMPoint):  # type: ignore[misc]
    """DimOS wrapper for geometry_msgs.Point (3D position).

    Inherits x/y/z from LCMPoint. Wire-identical to Vector3 but
    semantically represents a position, not a direction/displacement.
    """

    msg_name = "geometry_msgs.Point"

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        super().__init__(float(x), float(y), float(z))

    def __repr__(self) -> str:
        return f"Point(x={self.x}, y={self.y}, z={self.z})"
