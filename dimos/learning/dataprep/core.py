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

"""Dataset-shape types + pure helpers.

Sub-configs (StreamField, SyncConfig, OutputConfig, EpisodeExtractor) and
data records (Episode, Sample) live here. So do the stateless functions
that walk samples — `resolve_field`, `extract_episodes`,
`iter_episode_samples`. Pure and side-effect-free; importable without
booting a Module.

The impure orchestration that composes these (opening the store, driving
the writer, writing files) lives in `build.py`.
"""

from __future__ import annotations

import bisect
from collections.abc import Callable, Iterator
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, ConfigDict, Field

from dimos.constants import STATE_DIR
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.memory2.stream import Stream

# Each `formats/<name>/` package's writer/reader expose these, via get_writer/get_inspector.
Writer = Callable[[Iterator["Sample"], "OutputConfig"], Path]
Inspector = Callable[[Path], dict[str, Any]]

DEFAULT_FPS = 30.0  # resample rate == written video/timestamp rate


# ─────────────────────────────────────────────────────────────────────────────
# Sub-configs
# ─────────────────────────────────────────────────────────────────────────────


class EpisodeExtractor(BaseConfig):
    extractor: Literal["episode_status", "ranges"] = "episode_status"
    # Recorded stream name for EpisodeStatus events. Must match the recorder's
    # `status` In port (CollectionRecorder records it as "status").
    status_stream: str = "status"
    ranges: list[tuple[float, float]] | None = None


class StreamField(BaseConfig):
    stream: str
    field: str | None = None


class SyncConfig(BaseConfig):
    anchor: str
    rate_hz: float
    tolerance_ms: float
    # TODO: add "interp" — do it per-stream (lerp low-dim vectors, force nearest
    # for ndim>=3 images, can't blend frames). Only "nearest" is wired today.
    strategy: Literal["nearest"] = "nearest"
    action_shift: int = 1


class OutputConfig(BaseConfig):
    format: Literal["lerobot", "hdf5"] = "lerobot"
    path: Path
    metadata: dict[str, Any] = Field(default_factory=dict)


class DataPrepConfig(BaseConfig):
    """Everything needed to turn a recording into a dataset.

    `source` is a recording `.db`; `observation`/`action` map dataset feature
    names to recorded streams; `sync` resamples them onto a common timeline;
    `output` selects format + path. Consumed by `build.run_dataprep`.
    """

    source: str = ""
    episodes: EpisodeExtractor = EpisodeExtractor()
    observation: dict[str, StreamField] = Field(default_factory=dict)
    action: dict[str, StreamField] = Field(default_factory=dict)
    sync: SyncConfig = SyncConfig(anchor="image", rate_hz=DEFAULT_FPS, tolerance_ms=50.0)
    output: OutputConfig = OutputConfig(format="lerobot", path=STATE_DIR / "datasets" / "default")


# ─────────────────────────────────────────────────────────────────────────────
# Data records
# ─────────────────────────────────────────────────────────────────────────────


class Episode(BaseModel):
    id: str
    start_ts: float
    end_ts: float
    task_label: str | None = None
    success: bool = True
    metadata: dict[str, Any] = Field(default_factory=dict)


class Sample(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    ts: float
    episode_id: str
    observation: dict[str, NDArray[Any]]
    action: dict[str, NDArray[Any]]
    task_label: str | None = None  # carried from the episode for multi-task datasets


# ─────────────────────────────────────────────────────────────────────────────
# Pure helpers — used by format writers and run_dataprep
# ─────────────────────────────────────────────────────────────────────────────


def resolve_field(msg: Any, ref: StreamField) -> NDArray[Any]:
    """Project `msg` through `ref` (attribute access) and coerce to ndarray.

    Single source of truth for obs/action construction across train and
    live inference. Behavior:
      - `ref.field is None`: best-effort coerce the whole message
        (Image → `.data`, ndarray pass-through, list/tuple → asarray).
      - `ref.field` set: `getattr(msg, ref.field)` (or `msg[ref.field]`
        for dict payloads) then coerce.
    """
    if ref.field is None:
        value: Any = msg
    elif isinstance(msg, dict):
        value = msg[ref.field]
    else:
        value = getattr(msg, ref.field)

    if isinstance(value, np.ndarray):
        return value
    if hasattr(value, "data") and isinstance(value.data, np.ndarray):
        # e.g. Image → use its underlying ndarray
        return value.data
    return np.asarray(value)


def is_image_array(arr: NDArray[Any]) -> bool:
    """Image-like per-frame array (→ video) vs low-dim feature (→ parquet).

    3D+ is always an image; 2D is a grayscale frame only if integer — float 2D
    is a low-dim matrix (pose/rotation/jacobian) and stays in the parquet.
    (Writers on time-stacked (T, …) arrays use ``ndim >= 3`` directly.)
    """
    if arr.ndim >= 3:
        return True
    if arr.ndim == 2:
        return np.issubdtype(arr.dtype, np.integer)
    return False


def extract_episodes(store: SqliteStore, cfg: EpisodeExtractor) -> list[Episode]:
    """Walk recorded events into Episodes per the configured strategy.

    EPISODE_STATUS: scan `cfg.status_stream` for state transitions emitted
        by `EpisodeMonitorModule`. State machine (mirrors the live monitor):
            ev.last_event == "start":   begin (auto-commit any prior pending)
            ev.last_event == "save":    commit (success=True)
            ev.last_event == "discard": drop (success=False)
            end of stream with pending: dropped (matches live spec)

    RANGES: emit one Episode per (start, end) tuple in `cfg.ranges`.
    """
    if cfg.extractor == "ranges":
        if not cfg.ranges:
            return []
        return [
            Episode(id=f"ep_{i:06d}", start_ts=t0, end_ts=t1)
            for i, (t0, t1) in enumerate(cfg.ranges)
        ]

    # episode_status (default)
    status_stream: Stream[Any, Any] = store.stream(cfg.status_stream)
    events = list(status_stream)  # observations in storage order

    episodes: list[Episode] = []
    pending_start_ts: float | None = None
    pending_label: str | None = None
    counter = 0

    def _commit(end_ts: float, success: bool, label: str | None) -> None:
        nonlocal counter, pending_start_ts, pending_label
        if pending_start_ts is None:
            return
        episodes.append(
            Episode(
                id=f"ep_{counter:06d}",
                start_ts=pending_start_ts,
                end_ts=end_ts,
                task_label=label,
                success=success,
            )
        )
        counter += 1
        pending_start_ts = None
        pending_label = None

    for obs in events:
        ev = obs.data
        last_event = getattr(ev, "last_event", None)
        ts = obs.ts
        label = getattr(ev, "task_label", None)

        if last_event == "start":
            # Auto-commit any prior pending episode (success=True per state-machine spec).
            _commit(ts, success=True, label=pending_label)
            # obs.ts is the press time — the recorder stamps EpisodeStatus from
            # its own `.ts` field (set at the button press, not at record time).
            pending_start_ts = ts
            pending_label = label
        elif last_event == "save":
            _commit(ts, success=True, label=pending_label or label)
        elif last_event == "discard":
            _commit(ts, success=False, label=pending_label or label)
        # "init" and unknown events are no-ops.

    # Anything still pending at end-of-stream is dropped (state-machine spec).
    return episodes


def iter_episode_samples(
    store: SqliteStore,
    episode: Episode,
    streams: dict[str, StreamField],  # observation ∪ action
    sync: SyncConfig,
    obs_keys: set[str] | None = None,
    action_keys: set[str] | None = None,
) -> Iterator[Sample]:
    """Yield synced (obs, action) Samples for one episode.

    Walks the anchor stream at `sync.rate_hz` between `episode.start_ts` and
    `episode.end_ts`. For each anchor timestamp, picks the nearest sample
    from each configured stream within `sync.tolerance_ms`. Skips frames
    where any required stream lacks a nearby sample.

    `obs_keys` / `action_keys` partition `streams` into observation vs
    action. If omitted, every key is treated as observation (used by
    callers that only need raw aligned data).

    With `sync.action_shift > 0` (default 1), each frame's action is taken
    `action_shift` frames later (next-state target); the tail is dropped.
    """
    if sync.anchor not in streams:
        raise ValueError(f"sync.anchor {sync.anchor!r} not in streams: {sorted(streams)}")

    obs_keys = obs_keys if obs_keys is not None else set(streams)
    action_keys = action_keys if action_keys is not None else set()

    tolerance_s = sync.tolerance_ms / 1000.0

    # Materialize each stream's (timestamps, messages) once per episode.
    cached: dict[str, tuple[list[float], list[Any]]] = {}
    for key, ref in streams.items():
        sub: Stream[Any, Any] = store.stream(ref.stream).time_range(
            episode.start_ts, episode.end_ts
        )
        ts_list: list[float] = []
        msg_list: list[Any] = []
        for obs in sub:
            ts_list.append(obs.ts)
            msg_list.append(obs.data)
        # Keep them sorted by time — query order is usually already sorted, but be safe.
        if ts_list and any(ts_list[i] > ts_list[i + 1] for i in range(len(ts_list) - 1)):
            order = sorted(range(len(ts_list)), key=ts_list.__getitem__)
            ts_list = [ts_list[i] for i in order]
            msg_list = [msg_list[i] for i in order]
        cached[key] = (ts_list, msg_list)

    anchor_ts, _ = cached[sync.anchor]
    if not anchor_ts:
        return

    # Build the sequence of target timestamps for this episode.
    if sync.rate_hz > 0:
        # Uniform 1/rate_hz grid, phase-locked to the first anchor sample —
        # what LeRobot expects (it assumes contiguous fixed-fps frames).
        period = 1.0 / sync.rate_hz
        targets: list[float] = []
        t = anchor_ts[0]
        end = anchor_ts[-1]
        while t <= end:
            targets.append(t)
            t += period
    else:
        # rate_hz=0: follow the anchor's own timestamps (no image resampling).
        # dt is irregular if the camera jitters — fine for hdf5/custom trainers,
        # but not LeRobot-uniform.
        targets = list(anchor_ts)

    def _nearest(key: str, t: float) -> Any | None:
        ts_list, msg_list = cached[key]
        if not ts_list:
            return None
        # Nearest is i (first sample ≥ t) or i-1 (last sample < t).
        i = bisect.bisect_left(ts_list, t)
        if i == 0:
            best = 0
        elif i == len(ts_list):
            best = i - 1
        else:
            best = i if (ts_list[i] - t) < (t - ts_list[i - 1]) else i - 1
        return msg_list[best] if abs(ts_list[best] - t) <= tolerance_s else None

    def _build_frames() -> Iterator[Sample]:
        for t in targets:
            obs_dict: dict[str, NDArray[Any]] = {}
            act_dict: dict[str, NDArray[Any]] = {}
            skip = False
            for key, ref in streams.items():
                msg = _nearest(key, t)
                if msg is None:
                    skip = True
                    break
                arr = resolve_field(msg, ref)
                if not is_image_array(arr):
                    arr = arr.astype(np.float32, copy=False)
                if key in action_keys:
                    act_dict[key] = arr
                elif key in obs_keys:
                    obs_dict[key] = arr
            if skip:
                continue
            yield Sample(
                ts=t,
                episode_id=episode.id,
                observation=obs_dict,
                action=act_dict,
                task_label=episode.task_label,
            )

    shift = max(0, sync.action_shift)
    if shift == 0 or not action_keys:
        yield from _build_frames()
        return

    # frame i keeps its obs but takes frame i+shift's action; tail dropped.
    frames = list(_build_frames())
    for i in range(len(frames) - shift):
        cur = frames[i]
        nxt = frames[i + shift]
        yield Sample(
            ts=cur.ts,
            episode_id=cur.episode_id,
            observation=cur.observation,
            action=nxt.action,
            task_label=cur.task_label,
        )


def get_writer(format_name: str) -> Writer:
    """Lazy-import the format writer's `write` function."""
    if format_name == "lerobot":
        from dimos.learning.dataprep.formats.lerobot.writer import write
    elif format_name == "hdf5":
        from dimos.learning.dataprep.formats.hdf5.writer import write
    else:
        raise ValueError(f"Unknown format: {format_name!r}")
    return write


def get_inspector(format_name: str) -> Inspector:
    """Lazy-import the format reader's `inspect` function."""
    if format_name == "lerobot":
        from dimos.learning.dataprep.formats.lerobot.reader import inspect
    elif format_name == "hdf5":
        from dimos.learning.dataprep.formats.hdf5.reader import inspect
    else:
        raise ValueError(f"Unknown format: {format_name!r}")
    return inspect


def summarize_lengths(lengths: list[int]) -> dict[str, Any]:
    """Min/max/mean of per-episode frame counts + whether they're all equal."""
    if not lengths:
        return {"min": 0, "max": 0, "mean": 0.0, "uniform": True}
    return {
        "min": min(lengths),
        "max": max(lengths),
        "mean": sum(lengths) / len(lengths),
        "uniform": min(lengths) == max(lengths),
    }
