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
"""
ClearanceMonitor
================
Answers the question from the GitHub issue:

    "Should we add acceptable tolerance to a trajectory requested?
     Precision of control doesn't matter if there is lots of free space
     around the robot, but should walk carefully if there isn't."

How it works
------------
At every control tick it reads the current binary costmap and measures
the minimum distance from the robot's position to the nearest OCCUPIED
cell.  Based on that clearance it outputs two adaptive values:

  speed_scale   : multiplier on the velocity profiler's target speed
                  (1.0 = full speed in open space, down to ~0.3 in tight corridors)

  cte_tolerance : acceptable cross-track error before the controller
                  switches to precision mode
                  (loose in open space, tight near walls)

Integration
-----------
LocalPlanner calls ClearanceMonitor.update() every tick and passes
the returned (speed_scale, cte_tolerance) to TrajectoryController.advance().

The TrajectoryController multiplies target_speed by speed_scale and
uses cte_tolerance to gate the lateral correction gain.
"""
from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


class ClearanceMonitor:
    """
    Measures robot clearance from obstacles and returns adaptive
    speed and tolerance values.

    Parameters
    ----------
    robot_width     : robot body width in metres (used as minimum safe clearance)
    free_space_dist : clearance above which we consider space "open" (full speed)
    tight_dist      : clearance below which we consider space "tight" (slow, precise)
    """

    # Clearance thresholds (metres)
    FREE_SPACE_DIST: float = 1.5    # full speed above this
    CAUTION_DIST: float    = 0.8    # start slowing below this
    TIGHT_DIST: float      = 0.35   # minimum safe clearance

    # Speed scale range
    MIN_SPEED_SCALE: float = 0.30
    MAX_SPEED_SCALE: float = 1.00

    # CTE tolerance range (metres)
    LOOSE_TOLERANCE: float  = 0.25   # open space: don't overcorrect small errors
    TIGHT_TOLERANCE: float  = 0.04   # near walls: keep tight to the path

    def __init__(self, robot_width: float = 0.35) -> None:
        self._robot_width = robot_width
        self._min_safe    = robot_width / 2.0 + 0.05   # half-width + 5cm margin

        self._last_clearance: float = self.FREE_SPACE_DIST
        self._last_speed_scale: float = 1.0
        self._last_cte_tolerance: float = self.LOOSE_TOLERANCE

        # Exponential smoothing to avoid jitter
        self._alpha: float = 0.3   # lower = smoother but slower to react

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(
        self,
        robot_pos_world: NDArray[np.float64],
        costmap,                               # OccupancyGrid
    ) -> tuple[float, float]:
        """
        Update clearance estimate and return (speed_scale, cte_tolerance).

        Parameters
        ----------
        robot_pos_world : (2,) array  [x, y] in world frame
        costmap         : OccupancyGrid  (binary, 0=free, 100=occupied)

        Returns
        -------
        speed_scale     : float in [MIN_SPEED_SCALE, 1.0]
        cte_tolerance   : float in [TIGHT_TOLERANCE, LOOSE_TOLERANCE]
        """
        clearance = self._measure_clearance(robot_pos_world, costmap)

        # Smooth the clearance signal
        clearance = (self._alpha * clearance
                     + (1.0 - self._alpha) * self._last_clearance)
        self._last_clearance = clearance

        speed_scale   = self._clearance_to_speed_scale(clearance)
        cte_tolerance = self._clearance_to_tolerance(clearance)

        self._last_speed_scale    = speed_scale
        self._last_cte_tolerance  = cte_tolerance

        return speed_scale, cte_tolerance

    @property
    def clearance(self) -> float:
        return self._last_clearance

    @property
    def speed_scale(self) -> float:
        return self._last_speed_scale

    @property
    def cte_tolerance(self) -> float:
        return self._last_cte_tolerance

    def is_tight(self) -> bool:
        """True when robot is in a tight corridor."""
        return self._last_clearance < self.CAUTION_DIST

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _measure_clearance(
        self,
        robot_pos_world: NDArray[np.float64],
        costmap,
    ) -> float:
        """
        Convert robot world position to grid cell and measure
        minimum Euclidean distance to any OCCUPIED cell within
        a local search window.
        """
        try:
            from dimos.msgs.nav_msgs.OccupancyGrid import CostValues
            grid = costmap.grid
            info = costmap.info
            resolution = float(info.resolution)
            origin_x   = float(info.origin.position.x)
            origin_y   = float(info.origin.position.y)

            # Robot in grid coordinates
            gx = int((robot_pos_world[0] - origin_x) / resolution)
            gy = int((robot_pos_world[1] - origin_y) / resolution)

            h, w = grid.shape
            if not (0 <= gx < w and 0 <= gy < h):
                return self.FREE_SPACE_DIST

            # Search window: 2m radius
            search_r = int(2.0 / resolution)
            x0 = max(0, gx - search_r); x1 = min(w, gx + search_r)
            y0 = max(0, gy - search_r); y1 = min(h, gy + search_r)

            window = grid[y0:y1, x0:x1]
            occ_ys, occ_xs = np.where(window == int(CostValues.OCCUPIED))

            if len(occ_xs) == 0:
                return self.FREE_SPACE_DIST

            # World-frame distances to occupied cells
            occ_wx = (occ_xs + x0) * resolution + origin_x
            occ_wy = (occ_ys + y0) * resolution + origin_y
            dists  = np.hypot(occ_wx - robot_pos_world[0],
                              occ_wy - robot_pos_world[1])
            return float(np.min(dists))

        except Exception:
            # If costmap is unavailable or malformed, assume open space
            return self.FREE_SPACE_DIST

    def _clearance_to_speed_scale(self, clearance: float) -> float:
        """
        Piecewise linear mapping: clearance → speed_scale.

          clearance >= FREE_SPACE_DIST  →  1.0  (full speed)
          clearance == CAUTION_DIST     →  0.65
          clearance <= TIGHT_DIST       →  MIN_SPEED_SCALE
        """
        if clearance >= self.FREE_SPACE_DIST:
            return self.MAX_SPEED_SCALE
        if clearance <= self.TIGHT_DIST:
            return self.MIN_SPEED_SCALE
        if clearance >= self.CAUTION_DIST:
            # Free → caution: linear 1.0 → 0.65
            t = (clearance - self.CAUTION_DIST) / (self.FREE_SPACE_DIST - self.CAUTION_DIST)
            return float(0.65 + t * 0.35)
        else:
            # Caution → tight: linear 0.65 → MIN
            t = (clearance - self.TIGHT_DIST) / (self.CAUTION_DIST - self.TIGHT_DIST)
            return float(self.MIN_SPEED_SCALE + t * (0.65 - self.MIN_SPEED_SCALE))

    def _clearance_to_tolerance(self, clearance: float) -> float:
        """
        Piecewise linear mapping: clearance → cte_tolerance.

          clearance >= FREE_SPACE_DIST  →  LOOSE_TOLERANCE
          clearance <= TIGHT_DIST       →  TIGHT_TOLERANCE
        """
        if clearance >= self.FREE_SPACE_DIST:
            return self.LOOSE_TOLERANCE
        if clearance <= self.TIGHT_DIST:
            return self.TIGHT_TOLERANCE
        t = (clearance - self.TIGHT_DIST) / (self.FREE_SPACE_DIST - self.TIGHT_DIST)
        return float(self.TIGHT_TOLERANCE + t * (self.LOOSE_TOLERANCE - self.TIGHT_TOLERANCE))