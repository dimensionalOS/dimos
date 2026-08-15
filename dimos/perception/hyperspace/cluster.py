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

"""Clustering of high-scoring voxels for a text query."""

from __future__ import annotations

from typing import TypedDict

import numpy as np
from numpy.typing import NDArray
from scipy import ndimage


class VoxelCluster(TypedDict):
    """One connected blob of high-scoring voxels."""

    x: float
    y: float
    z: float
    score: float
    voxels: int


def top_clusters(
    centers: NDArray[np.floating],
    scores: NDArray[np.floating],
    voxel_size: float,
    top_k: int = 5,
    score_percentile: float = 97.0,
) -> list[VoxelCluster]:
    """Top-k connected clusters among voxels above the score percentile.

    Voxels above ``score_percentile`` are grouped by 26-connectivity on the
    voxel grid; clusters rank by their max score. Centers are score-weighted
    centroids in world coordinates.
    """
    centers = np.asarray(centers, dtype=np.float64)
    scores = np.asarray(scores, dtype=np.float32)
    if centers.shape[0] == 0:
        return []

    threshold = np.percentile(scores, score_percentile)
    hot = scores >= threshold
    if not hot.any():
        return []
    hot_centers = centers[hot]
    hot_scores = scores[hot]

    # Dense boolean grid over the hot voxels' bounding box (small by construction).
    idx = np.floor(hot_centers / voxel_size).astype(np.int64)
    origin = idx.min(axis=0)
    idx -= origin
    grid = np.zeros(idx.max(axis=0) + 1, dtype=bool)
    grid[idx[:, 0], idx[:, 1], idx[:, 2]] = True

    labels, count = ndimage.label(grid, structure=np.ones((3, 3, 3), dtype=bool))
    if count == 0:
        return []
    voxel_labels = labels[idx[:, 0], idx[:, 1], idx[:, 2]]

    clusters: list[VoxelCluster] = []
    for label in range(1, count + 1):
        members = voxel_labels == label
        weights = hot_scores[members] - hot_scores[members].min() + 1e-6
        centroid = np.average(hot_centers[members], axis=0, weights=weights)
        clusters.append(
            VoxelCluster(
                x=float(centroid[0]),
                y=float(centroid[1]),
                z=float(centroid[2]),
                score=float(hot_scores[members].max()),
                voxels=int(members.sum()),
            )
        )

    clusters.sort(key=lambda c: c["score"], reverse=True)
    return clusters[:top_k]
