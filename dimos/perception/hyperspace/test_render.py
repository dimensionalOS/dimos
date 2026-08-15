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

import math

import numpy as np
import pytest

from dimos.perception.hyperspace.cluster import top_clusters
from dimos.perception.hyperspace.render import (
    VIEWS,
    normalize_scores,
    render_nearby,
    render_view,
    scores_to_colors,
)


def _room(n: int = 2000, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """Synthetic room floor with one hot blob in a corner."""
    rng = np.random.default_rng(seed)
    centers = np.column_stack([rng.uniform(0, 8, n), rng.uniform(0, 6, n), rng.uniform(0, 0.2, n)])
    scores = rng.uniform(0.0, 0.2, n).astype(np.float32)
    blob = (centers[:, 0] > 6.5) & (centers[:, 1] > 4.5)
    scores[blob] = rng.uniform(0.8, 1.0, int(blob.sum()))
    return centers, scores


def test_scores_to_colors_endpoints() -> None:
    colors = scores_to_colors(np.array([0.0, 1.0]))
    assert colors.shape == (2, 3)
    np.testing.assert_array_equal(colors[0], [130, 130, 130])  # gray
    np.testing.assert_array_equal(colors[1], [255, 30, 30])  # red
    assert colors.dtype == np.uint8


def test_normalize_scores_constant_input() -> None:
    out = normalize_scores(np.full(10, 0.5))
    np.testing.assert_array_equal(out, 0.0)


@pytest.mark.parametrize("view", VIEWS)
def test_render_views_shapes(view: str) -> None:
    centers, scores = _room()
    img = render_view(centers, scores, view=view, voxel_size=0.1, out_px=300)
    assert img.ndim == 3
    assert img.shape[2] == 3
    assert img.dtype == np.uint8
    assert max(img.shape[:2]) >= 250


def test_render_empty_map() -> None:
    img = render_view(np.empty((0, 3)), np.empty(0), view="top", out_px=100)
    assert img.shape == (100, 100, 3)


def test_hot_blob_is_red_in_top_view() -> None:
    centers, scores = _room()
    img = render_view(centers, scores, view="top", voxel_size=0.1, out_px=400)
    h, w = img.shape[:2]
    # Blob is at high x (right) and high y (top of image).
    blob_region = img[: h // 4, 3 * w // 4 :]
    rest_region = img[h // 2 :, : w // 2]
    red_dominance = blob_region[:, :, 0].astype(int) - blob_region[:, :, 1].astype(int)
    rest_dominance = rest_region[:, :, 0].astype(int) - rest_region[:, :, 1].astype(int)
    assert red_dominance.max() > 100  # strongly red pixels present in blob corner
    assert rest_dominance.max() < 50  # rest stays grayish


def test_render_nearby_draws_inside_cylinder() -> None:
    centers, scores = _room()
    img = render_nearby(
        centers,
        scores,
        robot_xy=(4.0, 3.0),
        robot_yaw_rad=0.0,
        heading_rad=math.pi / 2,
        radius_m=3.0,
        voxel_size=0.1,
        out_px=300,
    )
    assert img.shape == (300, 300, 3)
    assert img.dtype == np.uint8
    # The green best-direction arrow must be present.
    green = (img[:, :, 1] > 200) & (img[:, :, 0] < 100)
    assert green.sum() > 20


def test_top_clusters_finds_blob() -> None:
    centers, scores = _room()
    clusters = top_clusters(centers, scores, voxel_size=0.1, top_k=5)
    assert 1 <= len(clusters) <= 5
    best = clusters[0]
    assert best["x"] > 6.0
    assert best["y"] > 4.0
    assert best["voxels"] >= 1
    assert best["score"] >= 0.8


def test_top_clusters_ranks_by_score() -> None:
    # Two blobs: one hotter than the other, far apart.
    centers = np.array([[0, 0, 0], [0.1, 0, 0], [5, 5, 0], [5.1, 5, 0]] * 10, dtype=float)
    centers += np.random.default_rng(2).normal(0, 0.01, centers.shape)
    scores = np.array([0.9, 0.9, 0.7, 0.7] * 10, dtype=np.float32)
    clusters = top_clusters(centers, scores, voxel_size=0.2, top_k=5, score_percentile=0.0)
    assert len(clusters) == 2
    assert clusters[0]["score"] > clusters[1]["score"]
    assert abs(clusters[0]["x"]) < 1.0  # hotter blob is at the origin


def test_top_clusters_empty() -> None:
    assert top_clusters(np.empty((0, 3)), np.empty(0), voxel_size=0.1) == []
