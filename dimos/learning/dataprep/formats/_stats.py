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

"""Streaming feature stats — shared by every format writer.

Welford for mean/std and a reservoir sample for q01/q99 over scalar /
low-dim features. Image-like (≥3D) features are subsampled and reduced
to per-channel summaries so per-pixel stats don't blow up memory.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import random
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.learning.dataprep.core import is_image_array


@dataclass
class FeatureAggregator:
    is_image: bool
    n: int = 0
    mean: NDArray[Any] | None = None
    m2: NDArray[Any] | None = None
    minv: NDArray[Any] | None = None
    maxv: NDArray[Any] | None = None
    reservoir: list[NDArray[Any]] = field(default_factory=list)
    image_seen: int = 0
    shape: tuple[int, ...] | None = None
    dtype: str | None = None


class StreamingStats:
    """Single-pass mean/std/min/max/quantile aggregator across many features."""

    def __init__(
        self, image_subsample: int = 10, quantile_reservoir: int = 10_000, seed: int = 0
    ) -> None:
        self.image_subsample = image_subsample
        self.quantile_reservoir = quantile_reservoir
        self._rng = random.Random(seed)
        self.aggs: dict[str, FeatureAggregator] = {}

    def update(self, name: str, value: NDArray[Any]) -> None:
        a = np.asarray(value)
        is_image = is_image_array(a)
        agg = self.aggs.setdefault(
            name,
            FeatureAggregator(is_image=is_image, shape=tuple(a.shape), dtype=str(a.dtype)),
        )

        if is_image:
            agg.image_seen += 1
            if (agg.image_seen - 1) % self.image_subsample != 0:
                return
            # Reduce spatial dims to a per-channel summary so stats stay small.
            if a.ndim == 2:  # grayscale (H, W) → single channel
                v = a.astype(np.float32).mean().reshape(1)
            elif a.ndim == 3:  # (H, W, C) → per-channel
                v = a.astype(np.float32).mean(axis=(0, 1))
            else:  # higher-dim image stacks → flatten
                v = a.astype(np.float32).reshape(-1)
        else:
            v = a.astype(np.float64)

        if agg.mean is None:
            agg.mean = np.zeros(v.shape, dtype=np.float64)
            agg.m2 = np.zeros(v.shape, dtype=np.float64)
            agg.minv = np.full(v.shape, np.inf, dtype=np.float64)
            agg.maxv = np.full(v.shape, -np.inf, dtype=np.float64)

        agg.n += 1
        delta = v - agg.mean
        agg.mean += delta / agg.n
        assert agg.m2 is not None
        agg.m2 += delta * (v - agg.mean)
        assert agg.minv is not None and agg.maxv is not None
        np.minimum(agg.minv, v, out=agg.minv)
        np.maximum(agg.maxv, v, out=agg.maxv)

        if not is_image:
            if len(agg.reservoir) < self.quantile_reservoir:
                agg.reservoir.append(v.copy())
            else:
                j = self._rng.randint(0, agg.n - 1)
                if j < self.quantile_reservoir:
                    agg.reservoir[j] = v.copy()

    def finalize(self) -> dict[str, dict[str, Any]]:
        out: dict[str, dict[str, Any]] = {}
        for name, agg in self.aggs.items():
            if agg.mean is None:
                continue
            n = max(1, agg.n)
            assert agg.m2 is not None
            var = agg.m2 / n if agg.n > 1 else np.zeros_like(agg.mean)
            std = np.sqrt(var)
            entry: dict[str, Any] = {
                "mean": agg.mean.tolist(),
                "std": std.tolist(),
                "min": agg.minv.tolist() if agg.minv is not None else None,
                "max": agg.maxv.tolist() if agg.maxv is not None else None,
                "count": int(agg.n),
            }
            if agg.reservoir:
                stacked = np.stack(agg.reservoir, axis=0)
                entry["q01"] = np.quantile(stacked, 0.01, axis=0).tolist()
                entry["q99"] = np.quantile(stacked, 0.99, axis=0).tolist()
            out[name] = entry
        return out


def stats_from_metadata(metadata: dict[str, Any]) -> StreamingStats:
    """Build a StreamingStats from output metadata, with shared writer defaults."""
    return StreamingStats(
        image_subsample=int(metadata.get("image_subsample", 10)),
        quantile_reservoir=int(metadata.get("quantile_reservoir", 10_000)),
        seed=int(metadata.get("stats_seed", 0)),
    )
