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

"""Voxel map where every voxel aggregates an embedding vector.

Each voxel aggregates its per-frame contributions as a weighted mean, a
per-dimension median, or a medoid. The robust modes keep up to
``max_samples`` per-frame mean vectors per voxel (float16), so one bad
frame — a reflection, a motion-blurred pass — cannot drag the voxel
toward a false match the way a mean does.
"""

from __future__ import annotations

import threading
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray

from dimos.mapping.voxels.keys import KEY_OFFSET, X_SHIFT, Y_SHIFT, pack_indices, unpack_keys

Aggregate = Literal["mean", "median", "medoid"]

#: Voxel rows processed per chunk when aggregating samples (bounds temporaries).
_AGG_CHUNK = 8192


def dilate_keys(keys: NDArray[np.int64], radius: int = 1) -> NDArray[np.int64]:
    """Sorted unique packed keys within a Chebyshev ``radius`` of ``keys``."""
    if radius <= 0:
        return np.unique(keys)
    steps = np.arange(-radius, radius + 1, dtype=np.int64)
    offsets = (
        steps[:, None, None] * (1 << X_SHIFT)
        + steps[None, :, None] * (1 << Y_SHIFT)
        + steps[None, None, :]
    ).reshape(-1)
    return np.unique((np.asarray(keys, dtype=np.int64)[:, None] + offsets[None, :]).reshape(-1))


def keys_near_mask(
    keys: NDArray[np.int64], keep_keys: NDArray[np.int64], radius: int = 1
) -> NDArray[np.bool_]:
    """Which of ``keys`` lie within ``radius`` voxels of any key in ``keep_keys``."""
    keys = np.asarray(keys, dtype=np.int64)
    if keep_keys.size == 0:
        return np.zeros(keys.shape, dtype=bool)
    dilated = dilate_keys(keep_keys, radius)
    idx = np.searchsorted(dilated, keys)
    idx = np.minimum(idx, dilated.size - 1)
    return np.asarray(dilated[idx] == keys)


def clean_mask_from_extras(voxel_map: EmbeddingVoxelMap) -> NDArray[np.bool_] | None:
    """The clean-map filter a saved map was built with; None when unsaved.

    Mirrors the live module's filter: a voxel must be near ray-observed space
    (absence means carved or unsampled) and either near healthy ray geometry
    or supported by multiple camera frames (one-frame observations off
    healthy surfaces are depth-noise shells).
    """
    extras = voxel_map.extras
    if "lidar_keys" not in extras:
        return None
    radius = int(extras.get("lidar_dilation", 2))
    keys = voxel_map.voxel_keys()
    near_observed = keys_near_mask(keys, extras["lidar_keys"].astype(np.int64), radius=radius)
    if "healthy_keys" not in extras:
        return near_observed
    near_healthy = keys_near_mask(keys, extras["healthy_keys"].astype(np.int64), radius=radius)
    counts = extras.get("sample_counts")
    if counts is None or counts.size != keys.size:
        counts = voxel_map.sample_counts()
    supported = counts.astype(np.int64) >= int(extras.get("min_frame_support", 2))
    return np.asarray(near_observed & (near_healthy | supported))


class EmbeddingVoxelMap:
    """Sparse world-frame voxel grid of aggregated embedding vectors.

    Inserts are cheap: per-frame contributions are appended to a pending
    buffer. ``reduce()`` folds pending contributions into the canonical
    per-voxel state; queries call it implicitly so results always see every
    insert. Thread-safe.

    ``aggregate`` selects how a voxel's per-frame contributions combine:

    - ``"mean"``: exact weighted mean (no per-sample storage).
    - ``"median"``: per-dimension median over the stored frame samples.
    - ``"medoid"``: the stored frame sample most aligned with the voxel's
      sample mean — always an actually-observed embedding.

    The robust modes keep the first ``max_samples`` frame means per voxel.
    """

    def __init__(
        self,
        voxel_size: float,
        dim: int,
        aggregate: Aggregate = "mean",
        max_samples: int = 8,
    ) -> None:
        if voxel_size <= 0:
            raise ValueError("voxel_size must be positive")
        if aggregate not in ("mean", "median", "medoid"):
            raise ValueError(f"aggregate must be mean, median or medoid, got {aggregate!r}")
        if max_samples < 1:
            raise ValueError("max_samples must be >= 1")
        self.voxel_size = voxel_size
        self.dim = dim
        self.aggregate: Aggregate = aggregate
        self.max_samples = max_samples
        #: Arbitrary arrays saved/loaded alongside the map (provenance, lidar keys).
        self.extras: dict[str, NDArray[Any]] = {}
        self._lock = threading.RLock()
        #: Guards only the pending list. Inserts must never wait on a fold:
        #: reduce() reindexes the whole sample store (seconds on 100k+ voxel
        #: maps) and a shared lock would stall ingest and drop frames.
        self._pending_lock = threading.Lock()
        # Canonical state: sorted unique keys, per-voxel embedding sums and weights.
        self._keys: NDArray[np.int64] = np.empty(0, dtype=np.int64)
        self._sums: NDArray[np.float32] = np.empty((0, dim), dtype=np.float32)
        self._weights: NDArray[np.float32] = np.empty(0, dtype=np.float32)
        self._pending: list[tuple[NDArray[np.int64], NDArray[np.float32], NDArray[np.float32]]] = []
        # Frame-sample store for the robust aggregation modes.
        self._track_samples = aggregate != "mean"
        self._samples: NDArray[np.float16] = np.empty((0, max_samples, dim), dtype=np.float16)
        self._sample_weights: NDArray[np.float32] = np.empty((0, max_samples), dtype=np.float32)
        self._sample_counts: NDArray[np.int32] = np.empty(0, dtype=np.int32)
        #: Cached aggregated embeddings; None = stale.
        self._agg: NDArray[np.float32] | None = None
        self._agg_normalized: NDArray[np.float32] | None = None
        #: Aggregate loaded from disk for a map whose samples were not saved.
        self._frozen_agg: NDArray[np.float32] | None = None

    def __len__(self) -> int:
        with self._lock, self._pending_lock:
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
        with self._pending_lock:
            return len(self._pending)

    def voxel_keys(self) -> NDArray[np.int64]:
        """Sorted packed keys of the reduced voxels (same order as query())."""
        with self._lock:
            self.reduce()
            return self._keys.copy()

    def sample_counts(self) -> NDArray[np.int32]:
        """Per-voxel count of contributing frames, ordered like the keys.

        Capped at ``max_samples``; all-ones for maps without a sample store
        (mean mode or legacy loads), where frame support is unknown.
        """
        with self._lock:
            self.reduce()
            if self._track_samples and self._sample_counts.size == self._keys.size:
                return self._sample_counts.copy()
            return np.ones(self._keys.size, dtype=np.int32)

    def insert_points(
        self,
        points: NDArray[np.floating],
        embeddings: NDArray[np.floating],
        patch_indices: NDArray[np.integer] | None = None,
    ) -> NDArray[np.int64]:
        """Insert world-frame points carrying embeddings.

        Either ``embeddings`` is (N, dim) with one vector per point, or it is a
        (P, dim) palette and ``patch_indices`` (N,) selects each point's row.
        The palette form avoids materializing N x dim: contributions are
        grouped by (voxel, palette row) first, then folded with counts.

        Returns the sorted unique packed keys of the voxels this batch
        touched, so callers can keep per-frame provenance.
        """
        points = np.asarray(points)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"points must be (N, 3), got {points.shape}")
        if points.shape[0] == 0:
            return np.empty(0, dtype=np.int64)
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
            # Group by (voxel, palette row): a dense (voxels, palette) count
            # matrix turns the fold into one BLAS matmul. The palette is small
            # (a patch grid), so the count matrix stays tiny, and this runs an
            # order of magnitude faster than np.add.at scatter -- insert speed
            # bounds how many camera frames survive backpressure.
            uniq, key_ids = np.unique(keys, return_inverse=True)
            n_rows = embeddings.shape[0]
            counts = np.bincount(
                key_ids * n_rows + patch_indices, minlength=uniq.size * n_rows
            ).reshape(uniq.size, n_rows)
            counts = counts.astype(np.float32)
            sums = np.asarray(counts @ embeddings, dtype=np.float32)
            weights = counts.sum(axis=1)

        with self._pending_lock:
            self._pending.append((uniq, sums, weights))
        return uniq

    def reduce(self) -> int:
        """Fold pending contributions into the canonical per-voxel state.

        Returns the number of voxels in the reduced map.
        """
        with self._lock:
            with self._pending_lock:
                pending = self._pending
                self._pending = []
            if pending:
                self._agg = None
                self._agg_normalized = None
                self._frozen_agg = None

                old_keys = self._keys
                all_keys = np.concatenate([old_keys, *(p[0] for p in pending)])
                all_sums = np.concatenate([self._sums, *(p[1] for p in pending)])
                all_weights = np.concatenate([self._weights, *(p[2] for p in pending)])

                uniq, inverse = np.unique(all_keys, return_inverse=True)
                sums = np.zeros((uniq.size, self.dim), dtype=np.float32)
                np.add.at(sums, inverse, all_sums)
                weights = np.zeros(uniq.size, dtype=np.float32)
                np.add.at(weights, inverse, all_weights)

                if self._track_samples:
                    samples = np.zeros((uniq.size, self.max_samples, self.dim), dtype=np.float16)
                    sample_weights = np.zeros((uniq.size, self.max_samples), dtype=np.float32)
                    sample_counts = np.zeros(uniq.size, dtype=np.int32)
                    if old_keys.size:
                        pos = np.searchsorted(uniq, old_keys)
                        samples[pos] = self._samples
                        sample_weights[pos] = self._sample_weights
                        sample_counts[pos] = self._sample_counts
                    self._samples = samples
                    self._sample_weights = sample_weights
                    self._sample_counts = sample_counts
                    for batch_keys, batch_sums, batch_weights in pending:
                        self._fold_samples(uniq, batch_keys, batch_sums, batch_weights)

                self._keys, self._sums, self._weights = uniq, sums, weights
            return int(self._keys.size)

    def _fold_samples(
        self,
        canonical: NDArray[np.int64],
        keys: NDArray[np.int64],
        sums: NDArray[np.float32],
        weights: NDArray[np.float32],
    ) -> None:
        """Append one batch's per-voxel means to the sample store (first-K kept)."""
        pos = np.searchsorted(canonical, keys)
        counts = self._sample_counts[pos]
        take = counts < self.max_samples
        p, c = pos[take], counts[take]
        self._samples[p, c] = (sums[take] / weights[take, None]).astype(np.float16)
        self._sample_weights[p, c] = weights[take]
        self._sample_counts[p] = c + 1

    # ------------------------------------------------------------------ aggregation

    def _aggregate_samples(self) -> NDArray[np.float32]:
        """Per-voxel median or medoid over the stored frame samples, chunked."""
        out = np.empty((self._keys.size, self.dim), dtype=np.float32)
        for start in range(0, self._keys.size, _AGG_CHUNK):
            stop = min(start + _AGG_CHUNK, self._keys.size)
            chunk = self._samples[start:stop].astype(np.float32)
            filled = self._sample_weights[start:stop] > 0
            # Voxels with no stored samples (e.g. inserted after load()) fall
            # back to their exact mean so the aggregate is always defined.
            empty = ~filled.any(axis=1)
            if empty.any():
                mean_fill = self._sums[start:stop][empty] / np.maximum(
                    self._weights[start:stop][empty, None], 1e-12
                )
                chunk[empty, 0] = mean_fill
                filled[empty, 0] = True
            if self.aggregate == "median":
                # Sort-based median: inf-pad the empty slots and pick the
                # middle of the filled prefix. Much faster than nanmedian,
                # which dominates save/query time on 300k+ voxel maps.
                counts = filled.sum(axis=1)
                chunk[~filled] = np.inf
                chunk.sort(axis=1)
                rows = np.arange(chunk.shape[0])
                lo = chunk[rows, (counts - 1) // 2]
                hi = chunk[rows, counts // 2]
                out[start:stop] = (lo + hi) * 0.5
            else:  # medoid: the sample most aligned with the voxel's sample mean
                norms = np.linalg.norm(chunk, axis=2, keepdims=True)
                unit = chunk / np.maximum(norms, 1e-12)
                unit[~filled] = 0.0
                mean = unit.sum(axis=1) / np.maximum(filled.sum(axis=1, keepdims=True), 1)
                dots = np.einsum("nkd,nd->nk", unit, mean)
                dots[~filled] = -np.inf
                best = np.argmax(dots, axis=1)
                out[start:stop] = chunk[np.arange(chunk.shape[0]), best]
        return out

    def embeddings(self, normalize: bool = True) -> tuple[NDArray[np.int64], NDArray[np.float32]]:
        """(keys, vectors): the per-voxel aggregated embedding.

        The aggregation follows ``self.aggregate``; maps loaded from disk
        without samples return the aggregate they were saved with.
        """
        with self._lock:
            self.reduce()
            if normalize and self._agg_normalized is not None:
                return self._keys.copy(), self._agg_normalized
            if self._frozen_agg is not None:
                agg = self._frozen_agg
            elif self.aggregate == "mean" or not self._track_samples:
                agg = self._sums / np.maximum(self._weights[:, None], 1e-12)
            else:
                if self._agg is None:
                    self._agg = self._aggregate_samples()
                agg = self._agg
            if normalize:
                norms = np.linalg.norm(agg, axis=1, keepdims=True)
                agg = np.asarray(agg / np.maximum(norms, 1e-12), dtype=np.float32)
                # Queries score one text vector per prompt against the same
                # matrix; renormalizing 100k+ x 1152 floats per prompt is the
                # bulk of query latency, so keep the normalized copy.
                self._agg_normalized = agg
                return self._keys.copy(), agg
            return self._keys.copy(), np.asarray(agg, dtype=np.float32)

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
        here; voxel aggregates are normalized internally.
        """
        q = np.asarray(query_vector, dtype=np.float32).reshape(-1)
        if q.shape[0] != self.dim:
            raise ValueError(f"query vector must have dim {self.dim}, got {q.shape[0]}")
        q = q / max(float(np.linalg.norm(q)), 1e-12)
        with self._lock:
            _, agg = self.embeddings(normalize=True)
            return self.centers(), (agg @ q).astype(np.float32)

    def clear(self) -> None:
        with self._lock:
            self._keys = np.empty(0, dtype=np.int64)
            self._sums = np.empty((0, self.dim), dtype=np.float32)
            self._weights = np.empty(0, dtype=np.float32)
            self._pending.clear()
            self._samples = np.empty((0, self.max_samples, self.dim), dtype=np.float16)
            self._sample_weights = np.empty((0, self.max_samples), dtype=np.float32)
            self._sample_counts = np.empty(0, dtype=np.int32)
            self._agg = None
            self._agg_normalized = None
            self._frozen_agg = None

    def smooth_scores(
        self, scores: NDArray[np.float32], iterations: int = 1
    ) -> NDArray[np.float32]:
        """Average each voxel's score with its 6-connected neighbors.

        Isolated single-voxel spikes are noise (real objects span contiguous
        voxels); a couple of iterations keep blobs and suppress speckle.
        Scores must be ordered like the reduced keys (query() output).
        """
        with self._lock:
            self.reduce()
            keys = self._keys
        offsets = (1 << X_SHIFT, -(1 << X_SHIFT), 1 << Y_SHIFT, -(1 << Y_SHIFT), 1, -1)
        smoothed = np.asarray(scores, dtype=np.float32)
        for _ in range(iterations):
            total = smoothed.copy()
            count = np.ones_like(smoothed)
            for offset in offsets:
                neighbor = keys + offset
                idx = np.searchsorted(keys, neighbor)
                idx_clipped = np.minimum(idx, keys.size - 1)
                valid = keys[idx_clipped] == neighbor
                total[valid] += smoothed[idx_clipped[valid]]
                count[valid] += 1
            smoothed = total / count
        return smoothed

    def save(self, path: str, extras: dict[str, NDArray[Any]] | None = None) -> None:
        """Persist the reduced map as an .npz archive.

        The robust modes additionally store the aggregated embedding (float16)
        so tools can re-query the saved aggregate without the sample store;
        ``extras`` arrays are saved under an ``extra_`` prefix and reappear in
        ``load(...).extras``.
        """
        with self._lock:
            self.reduce()
            arrays: dict[str, Any] = {
                "voxel_size": np.float64(self.voxel_size),
                "aggregate": np.str_(self.aggregate),
                "keys": self._keys,
                "sums": self._sums,
                "weights": self._weights,
            }
            if self._track_samples:
                _, agg = self.embeddings(normalize=False)
                arrays["agg"] = agg.astype(np.float16)
            for name, value in (extras or {}).items():
                arrays[f"extra_{name}"] = value
            # Uncompressed on purpose: zlib on ~700 MB of float noise takes
            # minutes and blows the 120 s RPC deadline on save_map.
            np.savez(path, **arrays)

    @classmethod
    def load(cls, path: str) -> EmbeddingVoxelMap:
        """Load a map previously written by save().

        Robust-mode maps come back with their saved aggregate frozen; sums
        and weights are always present, so mean queries stay exact.
        """
        data = np.load(path)
        aggregate = str(data["aggregate"]) if "aggregate" in data else "mean"
        voxel_map = cls(
            voxel_size=float(data["voxel_size"]),
            dim=int(data["sums"].shape[1]),
            aggregate=aggregate,  # type: ignore[arg-type]
        )
        voxel_map._keys = data["keys"].astype(np.int64)
        voxel_map._sums = data["sums"].astype(np.float32)
        voxel_map._weights = data["weights"].astype(np.float32)
        if "agg" in data:
            voxel_map._frozen_agg = data["agg"].astype(np.float32)
        if voxel_map._track_samples:
            n = voxel_map._keys.size
            voxel_map._samples = np.zeros(
                (n, voxel_map.max_samples, voxel_map.dim), dtype=np.float16
            )
            voxel_map._sample_weights = np.zeros((n, voxel_map.max_samples), dtype=np.float32)
            voxel_map._sample_counts = np.zeros(n, dtype=np.int32)
        voxel_map.extras = {
            name.removeprefix("extra_"): data[name]
            for name in data.files
            if name.startswith("extra_")
        }
        return voxel_map
