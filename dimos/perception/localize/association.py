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

"""Geometric association: one label's sightings in a window grouped into objects."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from dimos.perception.localize.memento import Sighting

# One row per sighting. Everything verification reports is a reduction over
# these columns.
M_TS, M_SCORE, M_CX, M_CY, M_CZ, M_KX, M_KY, M_KZ = range(8)
M_WIDTH = 8


@dataclass
class Groups:
    """Sightings of one label grouped into objects, built from one window's sightings.

    Parallel per-group arrays. The only geometry held is each group's fused
    union; per-sighting evidence is eight floats in a flat row.
    """

    centers: list[np.ndarray] = field(default_factory=list)  # running weighted center
    clouds: list[np.ndarray] = field(default_factory=list)  # fused union
    weights: list[np.ndarray | None] = field(default_factory=list)  # per fused point
    raw: list[float] = field(default_factory=list)  # raw points folded in so far
    members: list[list[float]] = field(default_factory=list)  # flat, M_WIDTH per row
    sightings: list[list[Sighting]] = field(default_factory=list)

    def add(self, sighting: Sighting, radius: float, voxel: float) -> int:
        """Assign one sighting to its object, folding its cloud in."""
        points = sighting.points.astype(np.float64)
        centroid = points.mean(axis=0)

        hit = next(
            (i for i, c in enumerate(self.centers) if np.linalg.norm(c - centroid) <= radius), -1
        )
        if hit < 0:
            hit = len(self.centers)
            self.centers.append(centroid)
            self.clouds.append(points)
            self.weights.append(None)
            self.raw.append(float(len(points)))
            self.members.append([])
            self.sightings.append([])
        else:
            self._fuse(hit, points, centroid, voxel)

        self.members[hit].extend((sighting.ts, sighting.score, *centroid, *sighting.viewpoint))
        self.sightings[hit].append(sighting)
        return hit

    def _fuse(self, i: int, points: np.ndarray, centroid: np.ndarray, voxel: float) -> None:
        """Voxel-average a sighting into the group's union.

        Each fused point carries the raw points behind it, so the union is a
        weighted mean over every viewpoint rather than a mean of means, and the
        running center follows the same counts.
        """
        na, nb = self.raw[i], float(len(points))
        self.centers[i] = (na * self.centers[i] + nb * centroid) / (na + nb)
        self.raw[i] = na + nb
        merged = np.vstack([self.clouds[i], points])

        if voxel > 0:
            w = np.ones(len(merged))
            weights = self.weights[i]
            if weights is not None:
                w[: len(weights)] = weights
            _, inverse = np.unique(
                np.floor(merged / voxel).astype(np.int64), axis=0, return_inverse=True
            )
            wsum = np.zeros(inverse.max() + 1)
            np.add.at(wsum, inverse, w)
            psum = np.zeros((len(wsum), 3))
            np.add.at(psum, inverse, merged * w[:, None])
            merged = psum / wsum[:, None]
            self.weights[i] = wsum
        self.clouds[i] = merged

    def rows(self, i: int) -> np.ndarray:
        return np.asarray(self.members[i]).reshape(-1, M_WIDTH)
