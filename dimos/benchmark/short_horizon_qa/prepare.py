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

"""Prepare reusable runtime-map snapshots for frozen Memory2 cutoffs."""

from __future__ import annotations

from collections.abc import Callable
import json
import math
import os
from pathlib import Path
import sqlite3
import tempfile
from typing import Any, NamedTuple

from dimos.benchmark.short_horizon_qa.models import (
    CutoffRecord,
    FrozenMemoryManifest,
    MapperSettings,
    StreamBoundary,
)
from dimos.mapping.voxels.module import VoxelMapTransformer
from dimos.memory2.cli.dataset import resolve_dataset
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

MANIFEST_NAME = "manifest.v1.json"
DERIVED_NAME = "derived.db"


def prepare_bundle(
    recording: str | Path,
    cutoff_seconds: list[float] | None,
    output: Path,
    *,
    progress: list[float] | None = None,
    mapper: MapperSettings = MapperSettings(),
    map_progress: Callable[[int, int], None] | None = None,
) -> FrozenMemoryManifest:
    """Build one derived map sidecar without copying the source recording."""
    cutoffs = _validate_seconds(cutoff_seconds or [])
    progresses = _validate_progress(progress or [])
    if not cutoffs and not progresses:
        raise ValueError("At least one cutoff in seconds or normalized progress is required")
    if output.exists():
        raise FileExistsError(f"Output already exists: {output}")

    source_path = resolve_dataset(recording).resolve()
    if not source_path.is_file():
        raise FileNotFoundError(source_path)
    wal_path = Path(f"{source_path}-wal")
    if wal_path.exists() and wal_path.stat().st_size > 0:
        raise ValueError("Source recording has an active WAL and is not immutable")

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f".{output.name}-", dir=output.parent) as temporary:
        temporary_path = Path(temporary)
        manifest = _prepare_into(
            source_path,
            str(recording),
            cutoffs,
            progresses,
            temporary_path,
            mapper,
            map_progress,
        )
        os.replace(temporary_path, output)
        return manifest


def _prepare_into(
    source_path: Path,
    source_identity: str,
    cutoffs: list[float],
    progresses: list[float],
    output: Path,
    mapper: MapperSettings,
    map_progress: Callable[[int, int], None] | None,
) -> FrozenMemoryManifest:
    derived_path = output / DERIVED_NAME
    with SqliteStore(path=str(source_path), must_exist=True, read_only=True) as source:
        if "global_map" in source.list_streams():
            raise ValueError("Source recording already contains a global_map stream")
        ranges = _stream_ranges(source)
        if not ranges:
            raise ValueError("Source recording contains no observations")
        recording_start = min(item[1] for item in ranges.values())
        recording_end = max(item[2] for item in ranges.values())
        duration = recording_end - recording_start
        selections = [_CutoffSelection(seconds=value, progress=None) for value in cutoffs] + [
            _CutoffSelection(
                seconds=resolve_progress(value, recording_start, recording_end) - recording_start,
                progress=value,
            )
            for value in progresses
        ]
        selections.sort(key=lambda item: (item.seconds, item.progress is None))
        absolute_cutoffs = [recording_start + item.seconds for item in selections]
        if absolute_cutoffs[-1] > recording_end:
            raise ValueError(
                f"Cutoff {selections[-1].seconds}s exceeds recording duration {duration:.3f}s"
            )
        cutoff_maps = _write_maps(
            source,
            derived_path,
            absolute_cutoffs,
            mapper,
            map_progress,
        )
        records = tuple(
            CutoffRecord(
                cutoff_seconds=selection.seconds,
                cutoff_timestamp=absolute,
                normalized_progress=selection.progress,
                stream_boundaries=_stream_boundaries(source, absolute),
                map_observation_id=map_obs.id,
                map_timestamp=map_obs.ts,
                map_frame_count=int(map_obs.tags["frame_count"]),
            )
            for selection, absolute, map_obs in zip(
                selections, absolute_cutoffs, cutoff_maps, strict=True
            )
        )

    _seal_sqlite(derived_path)
    manifest = FrozenMemoryManifest(
        source_identity=source_identity,
        source_path=str(source_path),
        source_size_bytes=source_path.stat().st_size,
        source_mtime_ns=source_path.stat().st_mtime_ns,
        recording_start_timestamp=recording_start,
        recording_end_timestamp=recording_end,
        mapper=mapper,
        cutoffs=records,
    )
    (output / MANIFEST_NAME).write_text(
        json.dumps(manifest.model_dump(mode="json"), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest


class _CutoffSelection(NamedTuple):
    seconds: float
    progress: float | None


def resolve_progress(progress: float, recording_start: float, recording_end: float) -> float:
    """Resolve normalized progress to an exact timestamp over a sealed range."""
    if not math.isfinite(progress) or not 0 <= progress <= 1:
        raise ValueError("Normalized progress must be finite and within [0, 1]")
    if recording_end < recording_start:
        raise ValueError("Recording end precedes recording start")
    if progress == 0:
        return recording_start
    if progress == 1:
        return recording_end
    return recording_start + progress * (recording_end - recording_start)


def _validate_seconds(values: list[float]) -> list[float]:
    if any(not math.isfinite(value) or value < 0 for value in values):
        raise ValueError("Cutoffs must be finite and non-negative")
    return sorted(set(values))


def _validate_progress(values: list[float]) -> list[float]:
    for value in values:
        resolve_progress(value, 0.0, 1.0)
    return sorted(set(values))


def _stream_ranges(source: SqliteStore) -> dict[str, tuple[int, float, float]]:
    result: dict[str, tuple[int, float, float]] = {}
    for name in source.list_streams():
        stream: Stream[Any] = source.stream(name)
        count = stream.count()
        if count:
            start, end = stream.get_time_range()
            result[name] = (count, start, end)
    return result


def _stream_boundaries(source: SqliteStore, cutoff: float) -> tuple[StreamBoundary, ...]:
    boundaries: list[StreamBoundary] = []
    for name in source.list_streams():
        bounded: Stream[Any] = source.stream(name).time_range(-math.inf, cutoff)
        count = bounded.count()
        last = bounded.last() if count else None
        boundaries.append(
            StreamBoundary(
                name=name,
                count=count,
                last_observation_id=last.id if last is not None else None,
                last_timestamp=last.ts if last is not None else None,
            )
        )
    return tuple(sorted(boundaries, key=lambda item: item.name))


def _write_maps(
    source: SqliteStore,
    derived_path: Path,
    cutoffs: list[float],
    mapper: MapperSettings,
    map_progress: Callable[[int, int], None] | None,
) -> list[Any]:
    if "lidar" not in source.list_streams():
        raise ValueError("Source recording has no lidar stream")
    lidar = source.stream("lidar", PointCloud2)
    total_frames = lidar.count()
    first = next(iter(lidar), None)
    if first is None:
        raise ValueError("No lidar observations exist before the final cutoff")
    if first.data.frame_id != mapper.frame_id:
        raise ValueError(
            f"LiDAR frame {first.data.frame_id!r} does not match mapper frame {mapper.frame_id!r}"
        )
    transformer = VoxelMapTransformer(
        emit_every=mapper.emit_every,
        voxel_size=mapper.voxel_size_m,
        block_count=mapper.block_count,
        device=mapper.device,
        carve_columns=mapper.carve_columns,
        frame_id=mapper.frame_id,
        show_startup_log=False,
    )
    emissions = iter(lidar.transform(transformer))
    last_reported = 0

    def next_emission() -> Any:
        nonlocal last_reported
        observation = next(emissions, None)
        if observation is not None and map_progress is not None:
            frame_count = int(observation.tags["frame_count"])
            if frame_count == total_frames or frame_count - last_reported >= 250:
                map_progress(frame_count, total_frames)
                last_reported = frame_count
        return observation

    try:
        latest = next_emission()
        if latest is None:
            raise ValueError("Mapper produced no global map")

        selected: list[Any] = []
        with SqliteStore(path=str(derived_path)) as derived:
            target = derived.stream("global_map", PointCloud2)
            stored_by_source_id: dict[int, Any] = {}
            following = next_emission()
            for cutoff in cutoffs:
                while following is not None and following.ts <= cutoff:
                    latest = following
                    following = next_emission()
                if latest.ts > cutoff:
                    raise ValueError(f"No runtime map was emitted by cutoff {cutoff}")
                source_key = latest.id
                stored = stored_by_source_id.get(source_key)
                if stored is None:
                    stored = target.append(
                        latest.data,
                        ts=latest.ts,
                        tags={**latest.tags, "source_observation_id": source_key},
                    )
                    stored_by_source_id[source_key] = stored
                selected.append(stored)
        return selected
    finally:
        close = getattr(emissions, "close", None)
        if close is not None:
            close()


def _seal_sqlite(path: Path) -> None:
    with sqlite3.connect(path) as connection:
        connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
        connection.execute("PRAGMA journal_mode=DELETE")
