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
Simple path history class for visualization purposes.
This is a minimal implementation to support websocket visualization.
"""

from dimos.msgs.geometry_msgs import Vector3


class PathHistory:
    """A simple container for storing a history of positions for visualization."""

    def __init__(self, points: list[Vector3 | tuple | list] | None = None) -> None:  # type: ignore[type-arg]
        """Initialize with optional list of points."""
        self.points: list[Vector3] = []
        if points:
            for p in points:
                if isinstance(p, Vector3):
                    self.points.append(p)
                else:
                    self.points.append(Vector3(*p))

    def ipush(self, point: Vector3 | tuple | list) -> "PathHistory":  # type: ignore[type-arg]
        """Add a point to the history (in-place) and return self."""
        if isinstance(point, Vector3):
            self.points.append(point)
        else:
            self.points.append(Vector3(*point))
        return self

    def iclip_tail(self, max_length: int) -> "PathHistory":
        """Keep only the last max_length points (in-place) and return self."""
        if max_length > 0 and len(self.points) > max_length:
            self.points = self.points[-max_length:]
        return self

    def last(self) -> Vector3 | None:
        """Return the last point in the history, or None if empty."""
        return self.points[-1] if self.points else None

    def length(self) -> float:
        """Calculate the total length of the path."""
        if len(self.points) < 2:
            return 0.0

        total = 0.0
        for i in range(1, len(self.points)):
            p1 = self.points[i - 1]
            p2 = self.points[i]
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dz = p2.z - p1.z
            total += (dx * dx + dy * dy + dz * dz) ** 0.5
        return total

    def __len__(self) -> int:
        """Return the number of points in the history."""
        return len(self.points)

    def __getitem__(self, index: int) -> Vector3:
        """Get a point by index."""
        return self.points[index]
