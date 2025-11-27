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
Simple costmap wrapper for visualization purposes.
This is a minimal implementation to support websocket visualization.
"""

import numpy as np

from dimos.msgs.nav_msgs import OccupancyGrid


class CostmapViz:
    """A wrapper around OccupancyGrid for visualization compatibility."""

    def __init__(self, occupancy_grid: OccupancyGrid | None = None) -> None:
        """Initialize from an OccupancyGrid."""
        self.occupancy_grid = occupancy_grid

    @property
    def data(self) -> np.ndarray | None:  # type: ignore[type-arg]
        """Get the costmap data as a numpy array."""
        if self.occupancy_grid:
            return self.occupancy_grid.grid
        return None

    @property
    def width(self) -> int:
        """Get the width of the costmap."""
        if self.occupancy_grid:
            return self.occupancy_grid.width
        return 0

    @property
    def height(self) -> int:
        """Get the height of the costmap."""
        if self.occupancy_grid:
            return self.occupancy_grid.height
        return 0

    @property
    def resolution(self) -> float:
        """Get the resolution of the costmap."""
        if self.occupancy_grid:
            return self.occupancy_grid.resolution
        return 1.0

    @property
    def origin(self):  # type: ignore[no-untyped-def]
        """Get the origin pose of the costmap."""
        if self.occupancy_grid:
            return self.occupancy_grid.origin
        return None
