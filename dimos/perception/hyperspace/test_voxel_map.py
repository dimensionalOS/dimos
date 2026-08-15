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

import numpy as np
import pytest

from dimos.perception.hyperspace.voxel_map import (
    EmbeddingVoxelMap,
    dilate_keys,
    keys_near_mask,
)


def test_insert_and_mean() -> None:
    m = EmbeddingVoxelMap(voxel_size=0.1, dim=4)
    # Two points in the same voxel, one in another.
    points = np.array([[0.01, 0.01, 0.01], [0.05, 0.05, 0.05], [1.0, 0.0, 0.0]])
    embs = np.array(
        [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]],
        dtype=np.float32,
    )
    m.insert_points(points, embs)
    assert m.pending_count == 1

    assert m.reduce() == 2
    keys, means = m.mean_embeddings(normalize=False)
    assert keys.shape == (2,)
    # Voxel with two points averages their embeddings.
    np.testing.assert_allclose(sorted(means.max(axis=1)), [0.5, 1.0])


def test_palette_insert_matches_dense_insert() -> None:
    rng = np.random.default_rng(0)
    points = rng.uniform(-1, 1, size=(500, 3))
    palette = rng.normal(size=(16, 8)).astype(np.float32)
    patch_idx = rng.integers(0, 16, size=500)

    dense = EmbeddingVoxelMap(voxel_size=0.25, dim=8)
    dense.insert_points(points, palette[patch_idx])
    sparse = EmbeddingVoxelMap(voxel_size=0.25, dim=8)
    sparse.insert_points(points, palette, patch_indices=patch_idx)

    dk, dm = dense.mean_embeddings(normalize=False)
    sk, sm = sparse.mean_embeddings(normalize=False)
    np.testing.assert_array_equal(dk, sk)
    np.testing.assert_allclose(dm, sm, atol=1e-5)


def test_incremental_reduce_equals_single_batch() -> None:
    rng = np.random.default_rng(1)
    points = rng.uniform(-2, 2, size=(300, 3))
    embs = rng.normal(size=(300, 5)).astype(np.float32)

    whole = EmbeddingVoxelMap(voxel_size=0.5, dim=5)
    whole.insert_points(points, embs)

    parts = EmbeddingVoxelMap(voxel_size=0.5, dim=5)
    for chunk in range(3):
        parts.insert_points(points[chunk::3], embs[chunk::3])
        parts.reduce()  # interleave folds with inserts

    wk, wm = whole.mean_embeddings(normalize=False)
    pk, pm = parts.mean_embeddings(normalize=False)
    np.testing.assert_array_equal(wk, pk)
    np.testing.assert_allclose(wm, pm, atol=1e-4)


def test_query_scores_cosine() -> None:
    m = EmbeddingVoxelMap(voxel_size=0.1, dim=3)
    m.insert_points(
        np.array([[0.0, 0.0, 0.0], [5.0, 0.0, 0.0]]),
        np.array([[1, 0, 0], [0, 1, 0]], dtype=np.float32),
    )
    centers, scores = m.query(np.array([2.0, 0.0, 0.0]))  # normalized internally
    assert centers.shape == (2, 3)
    by_x = np.argsort(centers[:, 0])
    np.testing.assert_allclose(scores[by_x], [1.0, 0.0], atol=1e-6)
    # Centers are voxel centers in world coords.
    np.testing.assert_allclose(centers[by_x][0], [0.05, 0.05, 0.05], atol=1e-6)


def test_negative_coordinates() -> None:
    m = EmbeddingVoxelMap(voxel_size=0.1, dim=2)
    m.insert_points(
        np.array([[-0.35, -0.35, -0.35]]),
        np.array([[1, 0]], dtype=np.float32),
    )
    centers, _ = m.query(np.array([1.0, 0.0]))
    np.testing.assert_allclose(centers[0], [-0.35, -0.35, -0.35], atol=0.051)


def test_smooth_scores_averages_neighbors() -> None:
    m = EmbeddingVoxelMap(voxel_size=1.0, dim=2)
    # Two adjacent voxels along x and one far away.
    m.insert_points(
        np.array([[0.5, 0.5, 0.5], [1.5, 0.5, 0.5], [9.5, 0.5, 0.5]]),
        np.eye(3, 2, dtype=np.float32),
    )
    m.reduce()
    scores = np.array([1.0, 0.0, 1.0], dtype=np.float32)
    smoothed = m.smooth_scores(scores, iterations=1)
    # Adjacent pair averages toward each other, isolated voxel is unchanged.
    np.testing.assert_allclose(smoothed, [0.5, 0.5, 1.0])


def test_save_load_roundtrip(tmp_path) -> None:
    rng = np.random.default_rng(3)
    m = EmbeddingVoxelMap(voxel_size=0.2, dim=6)
    m.insert_points(rng.uniform(-1, 1, (100, 3)), rng.normal(size=(100, 6)).astype(np.float32))
    path = str(tmp_path / "map.npz")
    m.save(path)
    loaded = EmbeddingVoxelMap.load(path)
    assert loaded.voxel_size == m.voxel_size
    ok, om = m.mean_embeddings(normalize=False)
    lk, lm = loaded.mean_embeddings(normalize=False)
    np.testing.assert_array_equal(ok, lk)
    np.testing.assert_allclose(om, lm, atol=1e-6)


def test_median_rejects_outlier_frame() -> None:
    m = EmbeddingVoxelMap(voxel_size=1.0, dim=3, aggregate="median")
    point = np.array([[0.5, 0.5, 0.5]])
    # Four consistent frames and one outlier frame in the same voxel.
    for _ in range(4):
        m.insert_points(point, np.array([[1.0, 0.0, 0.0]], dtype=np.float32))
    m.insert_points(point, np.array([[0.0, 0.0, 9.0]], dtype=np.float32))
    _, agg = m.embeddings(normalize=False)
    np.testing.assert_allclose(agg[0], [1.0, 0.0, 0.0], atol=1e-3)
    # The mean would have been dragged toward the outlier.
    _, mean = m.mean_embeddings(normalize=False)
    assert mean[0, 2] > 1.0


def test_medoid_returns_observed_sample() -> None:
    m = EmbeddingVoxelMap(voxel_size=1.0, dim=2, aggregate="medoid")
    point = np.array([[0.5, 0.5, 0.5]])
    m.insert_points(point, np.array([[1.0, 0.1]], dtype=np.float32))
    m.insert_points(point, np.array([[1.0, -0.1]], dtype=np.float32))
    m.insert_points(point, np.array([[-1.0, 0.0]], dtype=np.float32))
    _, agg = m.embeddings(normalize=False)
    # The medoid is one of the two aligned samples, never a blend.
    assert abs(agg[0, 0] - 1.0) < 1e-2
    assert abs(abs(agg[0, 1]) - 0.1) < 1e-2


def test_sample_cap_keeps_first_k() -> None:
    m = EmbeddingVoxelMap(voxel_size=1.0, dim=2, aggregate="median", max_samples=2)
    point = np.array([[0.5, 0.5, 0.5]])
    for value in (1.0, 3.0, 100.0):
        m.insert_points(point, np.array([[value, 0.0]], dtype=np.float32))
        m.reduce()
    _, agg = m.embeddings(normalize=False)
    np.testing.assert_allclose(agg[0], [2.0, 0.0], atol=1e-2)


def test_robust_save_load_freezes_aggregate(tmp_path) -> None:
    rng = np.random.default_rng(4)
    m = EmbeddingVoxelMap(voxel_size=0.2, dim=6, aggregate="median")
    for _ in range(3):
        m.insert_points(rng.uniform(-1, 1, (50, 3)), rng.normal(size=(50, 6)).astype(np.float32))
        m.reduce()
    path = str(tmp_path / "map.npz")
    m.save(path, extras={"frame_ts": np.array([1.0, 2.0])})
    loaded = EmbeddingVoxelMap.load(path)
    assert loaded.aggregate == "median"
    np.testing.assert_allclose(loaded.extras["frame_ts"], [1.0, 2.0])
    _, om = m.embeddings(normalize=False)
    _, lm = loaded.embeddings(normalize=False)
    np.testing.assert_allclose(om, lm, atol=2e-2)


def test_keys_near_mask() -> None:
    m = EmbeddingVoxelMap(voxel_size=1.0, dim=2)
    m.insert_points(
        np.array([[0.5, 0.5, 0.5], [1.5, 0.5, 0.5], [8.5, 0.5, 0.5]]),
        np.ones((3, 2), dtype=np.float32),
    )
    keys = m.voxel_keys()
    keep = EmbeddingVoxelMap(voxel_size=1.0, dim=2)
    keep_keys = keep.insert_points(np.array([[0.5, 0.5, 0.5]]), np.ones((1, 2), dtype=np.float32))
    # Radius 1 keeps the seed voxel and its direct neighbor, not the far one.
    mask = keys_near_mask(keys, keep_keys, radius=1)
    assert mask.sum() == 2
    assert keys_near_mask(keys, keep_keys, radius=0).sum() == 1
    assert keys_near_mask(keys, np.empty(0, dtype=np.int64)).sum() == 0
    assert dilate_keys(keep_keys, radius=1).size == 27


def test_input_validation() -> None:
    m = EmbeddingVoxelMap(voxel_size=0.1, dim=4)
    with pytest.raises(ValueError):
        m.insert_points(np.zeros((2, 2)), np.zeros((2, 4), dtype=np.float32))
    with pytest.raises(ValueError):
        m.insert_points(np.zeros((2, 3)), np.zeros((2, 3), dtype=np.float32))
    with pytest.raises(ValueError):
        m.query(np.zeros(3))
    m.insert_points(np.zeros((0, 3)), np.zeros((0, 4), dtype=np.float32))
    assert m.reduce() == 0
