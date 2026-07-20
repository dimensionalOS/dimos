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

"""Final map: the mapper's output over every frame of a recording.

Not ground truth, just the most complete map the pipeline produces, so it
serves as the collision reference for returned paths. The same replay also
produces incremental checkpoints: the occupied set at chosen mid-recording
times, which is what the robot had seen by then.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import itertools
import json
from time import perf_counter
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator.recording import iter_world_frames
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator
    from pathlib import Path

    from numpy.typing import NDArray

    from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
    from dimos.navigation.nav_3d.evaluator.cases import Suite
    from dimos.navigation.nav_3d.evaluator.config import EvalConfig
    from dimos.navigation.nav_3d.evaluator.recording import Frame

logger = setup_logger()

_KEY_OFFSET = 1 << 20


def voxel_keys(points: NDArray[np.float32], voxel_size: float) -> NDArray[np.int64]:
    """Pack voxel indices into sortable int64 keys, one per point."""
    idx = np.floor(points.astype(np.float64) / voxel_size).astype(np.int64) + _KEY_OFFSET
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


def key_centers(keys: NDArray[np.int64], voxel_size: float) -> NDArray[np.float32]:
    """Voxel center positions for packed keys, the inverse of voxel_keys."""
    mask = (1 << 21) - 1
    idx = np.stack([keys >> 42, (keys >> 21) & mask, keys & mask], axis=1) - _KEY_OFFSET
    return ((idx + 0.5) * voxel_size).astype(np.float32)


def keys_contain(sorted_keys: NDArray[np.int64], query: NDArray[np.int64]) -> NDArray[np.bool_]:
    if len(sorted_keys) == 0:
        return np.zeros(len(query), dtype=bool)
    pos = np.clip(np.searchsorted(sorted_keys, query), 0, len(sorted_keys) - 1)
    return np.asarray(sorted_keys[pos] == query)


def cylinder_offsets(
    radius: float, z_lo: float, z_hi: float, voxel_size: float
) -> NDArray[np.int64]:
    """Integer voxel offsets forming a vertical cylinder."""
    r_vox = int(np.ceil(radius / voxel_size))
    span = np.arange(-r_vox, r_vox + 1)
    dx, dy = np.meshgrid(span, span, indexing="ij")
    in_disc = (dx * voxel_size) ** 2 + (dy * voxel_size) ** 2 <= radius**2
    dz = np.arange(int(np.floor(z_lo / voxel_size)), int(np.ceil(z_hi / voxel_size)) + 1)
    disc = np.stack([dx[in_disc], dy[in_disc]], axis=1)
    out = np.concatenate([np.hstack([disc, np.full((len(disc), 1), z)]) for z in dz])
    return np.asarray(out, dtype=np.int64)


def offset_keys(
    points: NDArray[np.float32], offsets: NDArray[np.int64], voxel_size: float
) -> NDArray[np.int64]:
    """Keys of every (point voxel + offset) pair, shape (P * O,)."""
    idx = np.floor(points.astype(np.float64) / voxel_size).astype(np.int64) + _KEY_OFFSET
    swept = idx[:, None, :] + offsets[None, :, :]
    return np.asarray((swept[..., 0] << 42) | (swept[..., 1] << 21) | swept[..., 2])


def densify(points: NDArray[np.float32], step: float) -> NDArray[np.float32]:
    """Resample a polyline so consecutive samples are at most step apart."""
    if len(points) < 2:
        return points.astype(np.float32)
    out = [points[:1]]
    for a, b in itertools.pairwise(points):
        seg = np.linalg.norm(b - a)
        n = max(int(np.ceil(seg / step)), 1)
        t = np.linspace(0.0, 1.0, n + 1)[1:, None]
        out.append(a[None, :] * (1 - t) + b[None, :] * t)
    return np.concatenate(out).astype(np.float32)


@dataclass
class FinalMap:
    voxel_size: float
    occupied: NDArray[np.float32]
    occupied_keys: NDArray[np.int64]
    frames: int
    add_frame_ms: dict[str, float]
    build_ms: float


@dataclass
class MapCheckpoints:
    """Map state at increasing times, delta-encoded between snapshots.

    occupied tracks the mapper's healthy voxels. observed tracks every voxel a
    raw lidar return ever landed in, mapper-independent, so it only grows.
    """

    times: NDArray[np.float64]
    added: list[NDArray[np.int64]]
    removed: list[NDArray[np.int64]]
    observed_added: list[NDArray[np.int64]]

    def iter_snapshots(self) -> Iterator[tuple[NDArray[np.int64], NDArray[np.int64]]]:
        """Yield (occupied_keys, newly_observed_keys) in time order.

        occupied comes as the full sorted set. observed only ever grows and
        gets intersected by its consumer, so it arrives as the delta since
        the previous checkpoint instead of the accumulated multi-million-key
        set.
        """
        keys = np.array([], dtype=np.int64)
        for add, rem, obs in zip(self.added, self.removed, self.observed_added, strict=True):
            keys = np.delete(keys, np.searchsorted(keys, rem))
            keys = np.insert(keys, np.searchsorted(keys, add), add)
            yield keys, obs


CACHE_VERSION = 3
CHECKPOINT_CACHE_VERSION = 2


def _cache_path(db_path: Path, params: dict[str, float | int | str]) -> Path:
    digest = hashlib.sha1(json.dumps(params, sort_keys=True).encode()).hexdigest()[:10]
    return db_path.parent / ".final" / f"{db_path.stem}.{digest}.npz"


def _final_params(suite: Suite, cfg: EvalConfig) -> dict[str, float | int | str]:
    return {
        **cfg.mapper_fingerprint(),
        "align_tol": cfg.align_tol,
        "lidar_stream": suite.lidar_stream,
        "odom_stream": suite.odom_stream,
        "cache_version": CACHE_VERSION,
    }


def replay_frames(
    frames: Iterable[Frame],
    mapper: VoxelRayMapper,
    voxel_size: float,
    times: NDArray[np.float64],
) -> tuple[FinalMap, list[NDArray[np.int64]], list[NDArray[np.int64]]]:
    """Feed frames through the mapper in order, snapshotting at each requested
    time. A snapshot holds exactly the frames with ts <= its time. Returns the
    final map, the occupied-key snapshots, and the observed-key snapshots.
    """
    snapshots: list[NDArray[np.int64]] = []
    observed_snapshots: list[NDArray[np.int64]] = []
    observed = np.array([], dtype=np.int64)
    pending: list[NDArray[np.int64]] = []

    def merged() -> NDArray[np.int64]:
        nonlocal observed
        if pending:
            observed = np.union1d(observed, np.concatenate(pending))
            pending.clear()
        return observed

    add_ms: list[float] = []
    t0 = perf_counter()
    for frame in frames:
        while len(snapshots) < len(times) and frame.ts > times[len(snapshots)]:
            snapshots.append(np.unique(voxel_keys(mapper.global_map(), voxel_size)))
            observed_snapshots.append(merged())
        t1 = perf_counter()
        mapper.add_frame(frame.points, frame.origin)
        add_ms.append((perf_counter() - t1) * 1000)
        pts = frame.points[np.isfinite(frame.points).all(axis=1)]
        pending.append(np.unique(voxel_keys(pts, voxel_size)))
        if sum(len(p) for p in pending) > 4_000_000:
            merged()
    build_ms = (perf_counter() - t0) * 1000
    occupied = mapper.global_map()
    occupied_keys = np.unique(voxel_keys(occupied, voxel_size))
    final_observed = merged()
    while len(snapshots) < len(times):
        snapshots.append(occupied_keys)
        observed_snapshots.append(final_observed)
    add_arr = np.asarray(add_ms) if add_ms else np.zeros(1)
    final = FinalMap(
        voxel_size=voxel_size,
        occupied=occupied,
        occupied_keys=occupied_keys,
        frames=len(add_ms),
        add_frame_ms={
            "p50": float(np.percentile(add_arr, 50)),
            "p95": float(np.percentile(add_arr, 95)),
            "max": float(add_arr.max()),
        },
        build_ms=build_ms,
    )
    return final, snapshots, observed_snapshots


def _save_final(cache: Path, final: FinalMap) -> None:
    cache.parent.mkdir(exist_ok=True)
    np.savez_compressed(
        cache,
        occupied=final.occupied,
        occupied_keys=final.occupied_keys,
        frames=final.frames,
        add_p50=final.add_frame_ms["p50"],
        add_p95=final.add_frame_ms["p95"],
        add_max=final.add_frame_ms["max"],
    )
    logger.info("final map cached: %s (%d voxels)", cache.name, len(final.occupied))


def load_or_build_final_map(db_path: Path, suite: Suite, cfg: EvalConfig) -> FinalMap:
    cache = _cache_path(db_path, _final_params(suite, cfg))
    if cache.exists():
        data = np.load(cache)
        return FinalMap(
            voxel_size=cfg.voxel_size,
            occupied=data["occupied"],
            occupied_keys=data["occupied_keys"],
            frames=int(data["frames"]),
            add_frame_ms={
                "p50": float(data["add_p50"]),
                "p95": float(data["add_p95"]),
                "max": float(data["add_max"]),
            },
            build_ms=0.0,
        )

    logger.info("building final map for %s (cache miss)", db_path.name)
    final, _, _ = replay_frames(
        iter_world_frames(db_path, suite.lidar_stream, suite.odom_stream, cfg.align_tol),
        cfg.make_mapper(),
        cfg.voxel_size,
        np.array([], dtype=np.float64),
    )
    _save_final(cache, final)
    return final


def encode_deltas(
    snapshots: list[NDArray[np.int64]],
) -> tuple[list[NDArray[np.int64]], list[NDArray[np.int64]]]:
    added: list[NDArray[np.int64]] = []
    removed: list[NDArray[np.int64]] = []
    prev = np.array([], dtype=np.int64)
    for keys in snapshots:
        added.append(np.setdiff1d(keys, prev, assume_unique=True))
        removed.append(np.setdiff1d(prev, keys, assume_unique=True))
        prev = keys
    return added, removed


def load_or_build_checkpoints(
    db_path: Path, suite: Suite, cfg: EvalConfig, times: NDArray[np.float64]
) -> MapCheckpoints:
    """Occupied key sets at the requested times, deduped and sorted.

    A cache miss replays the whole recording once. The replay's final state
    also fills the final cache when that is missing.
    """
    times = np.unique(np.asarray(times, dtype=np.float64))
    params: dict[str, float | int | str] = {
        **_final_params(suite, cfg),
        "kind": "checkpoints",
        "times_sha": hashlib.sha1(times.tobytes()).hexdigest()[:10],
        "checkpoint_version": CHECKPOINT_CACHE_VERSION,
    }
    cache = _cache_path(db_path, params)
    if cache.exists():
        data = np.load(cache)
        n = len(data["times"])
        return MapCheckpoints(
            times=data["times"],
            added=[data[f"add_{i}"] for i in range(n)],
            removed=[data[f"rem_{i}"] for i in range(n)],
            observed_added=[data[f"obs_{i}"] for i in range(n)],
        )

    logger.info("building %d map checkpoints for %s (cache miss)", len(times), db_path.name)
    final, snapshots, observed = replay_frames(
        iter_world_frames(db_path, suite.lidar_stream, suite.odom_stream, cfg.align_tol),
        cfg.make_mapper(),
        cfg.voxel_size,
        times,
    )
    final_cache = _cache_path(db_path, _final_params(suite, cfg))
    if not final_cache.exists():
        _save_final(final_cache, final)
    added, removed = encode_deltas(snapshots)
    observed_added, _ = encode_deltas(observed)
    arrays: dict[str, NDArray[np.int64] | NDArray[np.float64]] = {"times": times}
    arrays |= {f"add_{i}": a for i, a in enumerate(added)}
    arrays |= {f"rem_{i}": r for i, r in enumerate(removed)}
    arrays |= {f"obs_{i}": o for i, o in enumerate(observed_added)}
    cache.parent.mkdir(exist_ok=True)
    np.savez_compressed(cache, **arrays)  # type: ignore[arg-type]
    logger.info("checkpoints cached: %s", cache.name)
    return MapCheckpoints(times=times, added=added, removed=removed, observed_added=observed_added)
