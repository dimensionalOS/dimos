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

"""Agent-callable lidar signal queries (wishlist item 3).

`LidarSignalSkills` is the MCP-facing surface for
`dimos.mapping.pointclouds.signals`: it accumulates the live world-frame
`/lidar` cloud (reusing `LidarPointCloudClient`, fed from the in-graph
`In[PointCloud2]` stream rather than its own LCM subscription) and tracks the
latest `/odom` pose, then exposes four `@skill`s an LLM agent can call:

- `measure_space` — how big is the scanned space (bounding box + dimensions).
- `nearest_obstacle_distance` — how far to the closest thing at body height.
- `check_clearance` — how far the robot can go in a direction before hitting
  something.
- `coverage_report` — how much has been scanned so far.

The robot-relative queries (nearest obstacle, clearance) are answered from the
robot's current odom pose, because the Go2's `/lidar` is a world-frame cloud —
"nearest" and "ahead" only mean something relative to where the robot is now.
"""

from __future__ import annotations

import math
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.mapping.pointclouds.live import LidarPointCloudClient
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class LidarSignalSkills(Module):
    """Agent-callable measure / clearance / coverage queries over live lidar."""

    lidar: In[PointCloud2]
    odom: In[PoseStamped]

    _client: LidarPointCloudClient
    _latest_pose: PoseStamped | None = None

    def __init__(
        self,
        # z-band, relative to the robot's odom z, treated as "body height" for
        # obstacle/clearance queries — excludes the floor below and anything
        # overhead. Robust across multi-level buildings (relative, not
        # absolute world z).
        body_band: tuple[float, float] = (-0.1, 1.2),
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        # Not started(): we feed it from the in-graph stream, not its own LCM sub.
        self._client = LidarPointCloudClient()
        self._body_band = body_band
        self._latest_pose = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.lidar.subscribe(self._client.add_lidar)))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_odom(self, pose: PoseStamped) -> None:
        self._latest_pose = pose

    def _robot_origin(self) -> tuple[float, float, float] | None:
        if self._latest_pose is None:
            return None
        p = self._latest_pose.position
        return (float(p.x), float(p.y), float(p.z))

    def _robot_yaw(self) -> float | None:
        if self._latest_pose is None:
            return None
        return float(self._latest_pose.orientation.to_euler().z)

    def _body_band_abs(self, origin: tuple[float, float, float]) -> tuple[float, float]:
        lo, hi = self._body_band
        return (origin[2] + lo, origin[2] + hi)

    def _has_points(self) -> bool:
        return self._client.snapshot().shape[0] > 0

    # Percentile bounds for measure_space: real accumulations carry stray
    # long-range returns (reflections, through-glass hits) that inflate exact
    # min/max bounds to fantasy dimensions (measured: 144x184x105m "room").
    MEASURE_PERCENTILES = (1.0, 99.0)

    @skill
    def measure_space(self) -> str:
        """Measure the size of the space the robot has scanned so far.

        Returns the overall dimensions (width x depth x height, in meters) of
        everything the lidar has seen, using outlier-robust bounds (1st-99th
        percentile per axis) so a few stray long-range returns don't inflate
        the answer. Use this to answer "how big is this room/area?".
        """
        if not self._has_points():
            return "No lidar data yet; nothing scanned."
        ext = self._client.extents(percentiles=self.MEASURE_PERCENTILES)
        return (
            f"Scanned space is about {ext.span[0]:.1f} x {ext.span[1]:.1f} m "
            f"(x {ext.min[0]:.1f}..{ext.max[0]:.1f} m, y {ext.min[1]:.1f}..{ext.max[1]:.1f} m) "
            f"and {ext.span[2]:.1f} m tall (robust 1-99% bounds)."
        )

    @skill
    def nearest_obstacle_distance(self) -> str:
        """Report how far the closest obstacle is from the robot right now.

        Measures the horizontal distance from the robot to the nearest lidar
        point at body height (the floor and overhead are ignored). Use this
        for a quick "is anything close to me?" safety check.
        """
        origin = self._robot_origin()
        if origin is None:
            return "Robot position unknown (no odometry received yet)."
        if not self._has_points():
            return "No lidar data yet."
        dist = self._client.nearest_obstacle(origin=origin, height_band=self._body_band_abs(origin))
        if math.isinf(dist):
            return "No obstacles detected at body height nearby."
        return f"The nearest obstacle is {dist:.2f} m away."

    @skill
    def check_clearance(self, heading_deg: float | None = None, fov_deg: float = 60.0) -> str:
        """Report how far the robot can travel in a direction before an obstacle.

        Looks at lidar points within a cone of `fov_deg` degrees around a
        heading and returns the distance to the closest one. Use before moving
        to check a direction is clear.

        Args:
            heading_deg: Direction to check, in world-frame degrees. Omit (or
                pass null) to check straight ahead of the robot's current
                facing.
            fov_deg: Full width of the cone to consider, in degrees.
        """
        origin = self._robot_origin()
        if origin is None:
            return "Robot position unknown (no odometry received yet)."
        if not self._has_points():
            return "No lidar data yet."

        if heading_deg is None:
            yaw = self._robot_yaw()
            heading = yaw if yaw is not None else 0.0
            which = "straight ahead"
        else:
            heading = math.radians(heading_deg)
            which = f"toward {heading_deg:.0f}°"

        dist = self._client.clearance(
            heading, fov_deg=fov_deg, origin=origin, height_band=self._body_band_abs(origin)
        )
        if math.isinf(dist):
            return f"Clear {which}: nothing within the {fov_deg:.0f}° cone."
        return f"Clearance {which}: {dist:.2f} m to the nearest obstacle."

    @skill
    def coverage_report(self) -> str:
        """Report how much of the space has been scanned so far.

        Returns the number of lidar points and frames accumulated and the
        approximate floor area covered (m^2). Use to judge whether more
        exploration is needed before answering questions about the space.
        """
        summary = self._client.coverage_summary()
        if summary["area_m2"] is None:
            return "Nothing scanned yet."
        return (
            f"Scanned {summary['points']:,} points over {summary['messages']} lidar frame(s), "
            f"covering about {summary['area_m2']:.1f} m²."
        )
