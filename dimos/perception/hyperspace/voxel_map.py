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

"""Voxel map where every voxel accumulates a weighted-mean embedding vector."""

from __future__ import annotations

import threading

import numpy as np
from numpy.typing import NDArray

from dimos.mapping.voxels.keys import KEY_OFFSET, pack_indices, unpack_keys


class EmbeddingVoxelMap:
    """Sparse world-frame voxel grid of mean embedding vectors.

    Inserts are cheap: per-frame contributions are appended to a pending
    buffer. ``reduce()`` folds pending contributions into the canonical
    per-voxel weighted means; queries call it implicitly so results always see
    every insert. Thread-safe.
    """

    def __init__(self, voxel_size: float, dim: int) -> None:
        if voxel_size <= 0:
            raise ValueError("voxel_size must be positive")
        self.voxel_size = voxel_size
        self.dim = dim
        self._lock = threading.RLock()
        # Canonical state: sorted unique keys, per-voxel embedding sums and weights.
        self._keys: NDArray[np.int64] = np.empty(0, dtype=np.int64)
        self._sums: NDArray[np.float32] = np.empty((0, dim), dtype=np.float32)
        self._weights: NDArray[np.float32] = np.empty(0, dtype=np.float32)
        self._pending: list[tuple[NDArray[np.int64], NDArray[np.float32], NDArray[np.float32]]] = []

    def __len__(self) -> int:
        with self._lock:
            pending = {int(k) for keys, _, _ in self._pending for k in np.unique(keys)}
            return int(np.union1d(self._keys, np.fromiter(pending, np.int64, len(pending))).size)

    @property
    def voxel_count(self) -> int:
        """Number of voxels in the reduced (canonical) map."""
        with self._lock:
            return int(self._keys.size)

    @property
    def pending_count(self) -> int:
        """Number of contribution batches not yet folded by reduce()."""
        with self._lock:
            return len(self._pending)

    def insert_points(
        self,
        points: NDArray[np.floating],
        embeddings: NDArray[np.floating],
        patch_indices: NDArray[np.integer] | None = None,
    ) -> None:
        """Insert world-frame points carrying embeddings.

        Either ``embeddings`` is (N, dim) with one vector per point, or it is a
        (P, dim) palette and ``patch_indices`` (N,) selects each point's row.
        The palette form avoids materializing N x dim: contributions are
        grouped by (voxel, palette row) first, then folded with counts.
        """
        points = np.asarray(points)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"points must be (N, 3), got {points.shape}")
        if points.shape[0] == 0:
            return
        embeddings = np.asarray(embeddings, dtype=np.float32)
        if embeddings.ndim != 2 or embeddings.shape[1] != self.dim:
            raise ValueError(f"embeddings must be (_, {self.dim}), got {embeddings.shape}")

        idx = np.floor(points / self.voxel_size).astype(np.int64) + KEY_OFFSET
        keys = pack_indices(idx)

        if patch_indices is None:
            if embeddings.shape[0] != points.shape[0]:
                raise ValueError("embeddings must have one row per point")
            uniq, inverse = np.unique(keys, return_inverse=True)
            sums = np.zeros((uniq.size, self.dim), dtype=np.float32)
            np.add.at(sums, inverse, embeddings)
            weights = np.bincount(inverse, minlength=uniq.size).astype(np.float32)
        else:
            patch_indices = np.asarray(patch_indices, dtype=np.int64)
            if patch_indices.shape[0] != points.shape[0]:
                raise ValueError("patch_indices must have one entry per point")
            # Group by (voxel, palette row): each unique pair contributes
            # count * palette[row], so the N x dim intermediate never exists.
            uniq, key_ids = np.unique(keys, return_inverse=True)
            n_rows = embeddings.shape[0]
            combo = key_ids * n_rows + patch_indices
            uniq_combo, pair_counts = np.unique(combo, return_counts=True)
            pair_weights = pair_counts.astype(np.float32)
            contrib = embeddings[uniq_combo % n_rows] * pair_weights[:, None]
            inverse = uniq_combo // n_rows
            sums = np.zeros((uniq.size, self.dim), dtype=np.float32)
            np.add.at(sums, inverse, contrib)
            weights = np.zeros(uniq.size, dtype=np.float32)
            np.add.at(weights, inverse, pair_weights)

        with self._lock:
            self._pending.append((uniq, sums, weights))

    def reduce(self) -> int:
        """Fold pending contributions into per-voxel weighted means.

        Returns the number of voxels in the reduced map.
        """
        with self._lock:
            if self._pending:
                all_keys = np.concatenate([self._keys, *(p[0] for p in self._pending)])
                all_sums = np.concatenate([self._sums, *(p[1] for p in self._pending)])
                all_weights = np.concatenate([self._weights, *(p[2] for p in self._pending)])
                self._pending.clear()

                uniq, inverse = np.unique(all_keys, return_inverse=True)
                sums = np.zeros((uniq.size, self.dim), dtype=np.float32)
                np.add.at(sums, inverse, all_sums)
                weights = np.zeros(uniq.size, dtype=np.float32)
                np.add.at(weights, inverse, all_weights)

                self._keys, self._sums, self._weights = uniq, sums, weights
            return int(self._keys.size)

    def mean_embeddings(
        self, normalize: bool = True
    ) -> tuple[NDArray[np.int64], NDArray[np.float32]]:
        """(keys, means): the per-voxel mean embedding, optionally L2-normalized."""
        with self._lock:
            self.reduce()
            means = self._sums / self._weights[:, None]
            if normalize:
                norms = np.linalg.norm(means, axis=1, keepdims=True)
                means = means / np.maximum(norms, 1e-12)
            return self._keys.copy(), means

    def centers(self) -> NDArray[np.float32]:
        """(N, 3) world-frame centers of the reduced voxels, ordered like keys."""
        with self._lock:
            self.reduce()
            return ((unpack_keys(self._keys) + 0.5) * self.voxel_size).astype(np.float32)

    def query(
        self, query_vector: NDArray[np.floating]
    ) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
        """Cosine-similarity scores of every voxel against a query vector.

        Returns (centers (N, 3), scores (N,)). The query vector is normalized
        here; voxel means are normalized internally.
        """
        q = np.asarray(query_vector, dtype=np.float32).reshape(-1)
        if q.shape[0] != self.dim:
            raise ValueError(f"query vector must have dim {self.dim}, got {q.shape[0]}")
        q = q / max(float(np.linalg.norm(q)), 1e-12)
        with self._lock:
            _, means = self.mean_embeddings(normalize=True)
            return self.centers(), (means @ q).astype(np.float32)

    def clear(self) -> None:
        with self._lock:
            self._keys = np.empty(0, dtype=np.int64)
            self._sums = np.empty((0, self.dim), dtype=np.float32)
            self._weights = np.empty(0, dtype=np.float32)
            self._pending.clear()
