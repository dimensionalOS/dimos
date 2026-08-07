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

"""Where the floor is, so the body band can sit on it.

`planners/target.py` slices the cloud at an ABSOLUTE z of 0.05..0.45 m,
which is only the body's band if the map's z origin is the ground. On a LIO
stack it is not: odometry starts at the sensor, so the origin sits at base
height and the band reads a slab well over the robot's head-room — blind to
the bottom of every obstacle and steering off table tops.

A constant trim cannot fix that (`cloud_z_offset` is a knife edge: one voxel
too much and the floor's own slab lands in the band and walls the robot in).
So the floor is estimated per tick from the cloud under the robot, sanity
bounded against what tf says the base height above ground is, and the slab
itself is dropped before the band is taken.

BOTH SIDES OF THE STACK READ THE SAME SLICE. The planner plans on the anchored
cloud and the follower measures its room hint off it, so `FloorAnchor` is the
one place either of them anchors: a follower that measured the raw band would
govern its speed by a different world than the one the plan was priced in.

The rust twin is `adapter/rust/src/floor.rs`; this is the specification. There
the per-module state below lives in the module struct and the knobs in its
config, since that is how a rust module holds anything.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.tf_pose import base_height_above_ground
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.navigation.tf_pose import OdomBasePose

logger = setup_logger()

# Neighbourhood the floor is read from. Wide enough to hold ground in a
# cluttered room, narrow enough that a ramp or a stair flight is still locally
# planar — which is the planner's own operating assumption.
FLOOR_RADIUS_M = 2.5
# Low quantile of that neighbourhood: the ground, not what stands on it.
FLOOR_PERCENTILE = 5.0
# Fewer returns than this is not a floor sample, it is noise.
FLOOR_MIN_POINTS = 100
# How far the estimate may sit from the tf prior before the prior wins. The
# prior is exact on flat ground; this is the room a step or a slope needs.
FLOOR_TOLERANCE_M = 0.35


def estimate_floor(
    points: NDArray[np.float32] | NDArray[np.float64],
    xy: tuple[float, float],
    prior: float | None = None,
    radius: float = FLOOR_RADIUS_M,
    percentile: float = FLOOR_PERCENTILE,
    min_points: int = FLOOR_MIN_POINTS,
    tolerance: float = FLOOR_TOLERANCE_M,
) -> float | None:
    """The floor's z under `xy`, or the tf prior when the cloud cannot say."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if not len(pts):
        return prior
    centre = np.asarray(xy, dtype=np.float64)
    near = pts[np.linalg.norm(pts[:, :2] - centre, axis=1) < radius]
    if len(near) < min_points:
        return prior
    floor = float(np.percentile(near[:, 2], percentile))
    if prior is not None and abs(floor - prior) > tolerance:
        return prior
    return floor


def anchor_to_floor(
    points: NDArray[np.float32], floor: float, margin: float
) -> NDArray[np.float32]:
    """Cloud re-zeroed on the floor, with the ground slab within `margin` dropped."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    shifted = pts - np.array([0.0, 0.0, floor], dtype=np.float32)
    if margin <= 0.0:
        return shifted
    return np.ascontiguousarray(shifted[shifted[:, 2] > np.float32(margin)])


class FloorAnchor:
    """A module's floor: the tf prior it is bounded by, and the anchoring itself."""

    def __init__(
        self,
        doing: str,
        enabled: bool,
        lidar_height: float,
        ground_margin: float,
        base_frame: str,
    ) -> None:
        # what the module is doing on the cloud, for the one warning line
        self.doing = doing
        self.enabled = enabled
        self.lidar_height = lidar_height
        self.ground_margin = ground_margin
        self.base_frame = base_frame
        self.base_z: float | None = None
        self.base_height: float | None = None
        self._warned = False

    def observe(self, resolver: OdomBasePose, sensor_frame: str, base_z: float) -> None:
        """Take the base's z every tick, and its height above ground once off the leg."""
        self.base_z = base_z
        if self.base_height is not None or self.lidar_height <= 0.0:
            return
        # Base-stamped odometry has no mount leg to subtract, so it cannot say
        # where the ground is; the cloud is then the only source.
        if sensor_frame == self.base_frame:
            return
        leg = resolver.sensor_to_base(sensor_frame)
        if leg is not None:
            self.base_height = base_height_above_ground(self.lidar_height, -leg)

    @property
    def prior(self) -> float | None:
        """Where tf puts the ground under the base, when it can say."""
        if self.base_z is None or self.base_height is None:
            return None
        return self.base_z - self.base_height

    def anchor(self, pts: NDArray[np.float32], xy: tuple[float, float]) -> NDArray[np.float32]:
        """Cloud re-zeroed on the floor under the robot, ground slab dropped.

        THE TF PRIOR IS REQUIRED, not optional. A low quantile of the cloud
        alone is only the floor if the floor is in the cloud; hand it a wall and
        it anchors to the wall's base and deletes the wall. Without the prior
        the band stays exactly where it was.
        """
        prior = self.prior
        if not self.enabled or prior is None:
            self._warn()
            return pts
        floor = estimate_floor(pts, xy, prior=prior)
        if floor is None:
            return pts
        return anchor_to_floor(pts, floor, self.ground_margin)

    def _warn(self) -> None:
        """Say once that the band is not on the floor — silence would hide it."""
        if self._warned or not self.enabled:
            return
        self._warned = True
        logger.warning(
            f"{self.doing} on an unanchored cloud: the body z-band is wherever "
            "the map's z origin puts it. Set lidar_height (and give tf the mount "
            "leg) to anchor it to the floor."
        )
