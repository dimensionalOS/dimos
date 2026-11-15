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

"""Relocalize the last few seconds of the map against a premap, for the PGO to weigh."""

from __future__ import annotations

import time

import numpy as np

from dimos.mapping.relocalization.go2.relocalize import Target, prepare, relocalize, wall_fitness
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PremapMatcher:
    """Matches a window of raw-odom-frame lidar against the premap with the Go2 strategy."""

    def __init__(
        self, premap: PointCloud2, *, min_points: int = 20_000, fitness_threshold: float = 0.45
    ) -> None:
        self._premap = premap.pointcloud
        self._target: Target | None = None
        self._min_points = min_points
        self._threshold = fitness_threshold

    def _prepared(self) -> Target:
        # the premap's features, on the matching thread the first time they are needed
        if self._target is None:
            self._target = prepare(self._premap)
            del self._premap  # the target holds everything matching needs
        return self._target

    def attempt(self, window: PointCloud2) -> tuple[Transform, float] | None:
        """The ``world -> map`` fix for this window and its fitness, or None below the threshold."""
        if len(window) < self._min_points:
            logger.info(f"relocalize: waiting for points n_pts={len(window)} < {self._min_points}")
            return None
        logger.info(f"relocalize: attempt started n_pts={len(window)}")
        t0 = time.monotonic()
        try:
            placement, fitness = relocalize(self._prepared(), window.pointcloud)
        except Exception:
            logger.exception("relocalize() failed")
            return None
        dt = time.monotonic() - t0
        if fitness < self._threshold:
            logger.info(
                f"relocalize: refused fitness={fitness:.3f} < {self._threshold} took {dt:.1f}s"
            )
            return None
        fix = Transform.from_matrix(
            np.linalg.inv(placement), frame_id="world", child_frame_id="map", ts=window.ts
        )
        t = fix.translation
        logger.info(
            f"relocalize: fix fitness={fitness:.3f} took {dt:.1f}s "
            f"t=({t.x:.2f}, {t.y:.2f}, {t.z:.2f})"
        )
        return fix, fitness

    def score(self, window: PointCloud2, placement: Transform) -> float:
        """How well ``window`` fits the premap with ``world -> map`` at ``placement``, as `attempt` scores."""
        return wall_fitness(
            self._prepared(), window.pointcloud, np.linalg.inv(placement.to_matrix())
        )


def attempt_fix(
    matcher: PremapMatcher, window: PointCloud2, current: Transform | None
) -> tuple[Transform, float, float | None] | None:
    """A fix for ``window``, its fitness, and the current placement's on the same window."""
    found = matcher.attempt(window)
    if found is None:
        return None
    return (*found, matcher.score(window, current) if current is not None else None)
