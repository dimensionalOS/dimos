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
that walk samples — `resolve_field`, `compute_stats`, `extract_episodes`,
`iter_episode_samples`. Importable without booting a Module.

`DataPrepModule` (in `dataprep_module.py`) is a thin wrapper that runs
these helpers on a thread.
"""

from __future__ import annotations

import bisect
from collections.abc import Callable, Iterator
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from dimos.protocol.service.spec import BaseConfig

Writer = Callable[[Iterator["Sample"], "OutputConfig"], Path]

if TYPE_CHECKING:
    from dimos.memory2.store.sqlite import SqliteStore


# ─────────────────────────────────────────────────────────────────────────────
# Sub-configs
# ─────────────────────────────────────────────────────────────────────────────


class EpisodeExtractor(BaseConfig):
    extractor: Literal["episode_status", "ranges", "whole_session"] = "episode_status"
    status_stream: str = "episode_status"
    ranges: list[tuple[float, float]] | None = None


class StreamField(BaseConfig):
    stream: str
    field: str | None = None


class SyncConfig(BaseConfig):
    anchor: str
    rate_hz: float
    tolerance_ms: float
    strategy: Literal["nearest", "interp"] = "nearest"


class OutputConfig(BaseConfig):
    format: Literal["lerobot", "hdf5"] = "lerobot"
    path: Path
    metadata: dict[str, Any] = {}


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

    @property
    def duration(self) -> float:
        return self.end_ts - self.start_ts


class Sample(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    ts: float
    episode_id: str
    observation: dict[str, np.ndarray]
    action: dict[str, np.ndarray]


# ─────────────────────────────────────────────────────────────────────────────
# Pure helpers — used by format writers, DataPrepModule
# ─────────────────────────────────────────────────────────────────────────────


def resolve_field(msg: Any, ref: StreamField) -> np.ndarray:
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


def extract_episodes(store: SqliteStore, cfg: EpisodeExtractor) -> list[Episode]:
    """Walk recorded events into Episodes per the configured strategy.

    EPISODE_STATUS: scan `cfg.status_stream` for state transitions emitted
        by `EpisodeMonitorModule`. State machine (mirrors the live monitor):
            ev.last_event == "start":   begin (auto-commit any prior pending)
            ev.last_event == "save":    commit (success=True)
            ev.last_event == "discard": drop (success=False)
            end of stream with pending: dropped (matches live spec)

    RANGES: emit one Episode per (start, end) tuple in `cfg.ranges`.

    WHOLE_SESSION: one Episode covering the full time range of every stream.
    """
    if cfg.extractor == "ranges":
        if not cfg.ranges:
            return []
        return [
            Episode(id=f"ep_{i:06d}", start_ts=t0, end_ts=t1)
            for i, (t0, t1) in enumerate(cfg.ranges)
        ]

    if cfg.extractor == "whole_session":
        # Span every stream's time range.
        names = store.list_streams()
        if not names:
            return []
        starts: list[float] = []
        ends: list[float] = []
        for name in names:
            try:
                stream = store.stream(name)
                t0, t1 = stream.get_time_range()
                starts.append(t0)
                ends.append(t1)
            except Exception:
                continue
        if not starts:
            return []
        return [Episode(id="ep_000000", start_ts=min(starts), end_ts=max(ends))]

    # episode_status (default)
    status_stream = store.stream(cfg.status_stream)
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
            pending_start_ts = getattr(ev, "current_episode_start_ts", None) or ts
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
    """
    if sync.anchor not in streams:
        raise ValueError(f"sync.anchor {sync.anchor!r} not in streams: {sorted(streams)}")

    obs_keys = obs_keys if obs_keys is not None else set(streams)
    action_keys = action_keys if action_keys is not None else set()

    tolerance_s = sync.tolerance_ms / 1000.0

    # Materialize each stream's (timestamps, messages) once per episode.
    cached: dict[str, tuple[list[float], list[Any]]] = {}
    for key, ref in streams.items():
        sub = store.stream(ref.stream).time_range(episode.start_ts, episode.end_ts)
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
        period = 1.0 / sync.rate_hz
        targets: list[float] = []
        t = anchor_ts[0]
        end = anchor_ts[-1]
        while t <= end:
            targets.append(t)
            t += period
    else:
        targets = list(anchor_ts)

    def _nearest(key: str, t: float) -> Any | None:
        ts_list, msg_list = cached[key]
        if not ts_list:
            return None
        i = bisect.bisect_left(ts_list, t)
        candidates: list[int] = []
        if i < len(ts_list):
            candidates.append(i)
        if i > 0:
            candidates.append(i - 1)
        best: int | None = None
        best_dt = float("inf")
        for c in candidates:
            dt = abs(ts_list[c] - t)
            if dt < best_dt:
                best = c
                best_dt = dt
        if best is None or best_dt > tolerance_s:
            return None
        return msg_list[best]

    for t in targets:
        obs_dict: dict[str, np.ndarray] = {}
        act_dict: dict[str, np.ndarray] = {}
        skip = False
        for key, ref in streams.items():
            msg = _nearest(key, t)
            if msg is None:
                skip = True
                break
            arr = resolve_field(msg, ref)
            if key in action_keys:
                act_dict[key] = arr
            elif key in obs_keys:
                obs_dict[key] = arr
        if skip:
            continue
        yield Sample(ts=t, episode_id=episode.id, observation=obs_dict, action=act_dict)


def compute_stats(
    samples: Iterator[Sample],
    image_subsample: int = 10,
    quantile_reservoir: int = 10_000,
    seed: int = 0,
) -> dict[str, Any]:
    """Single-pass per-feature stats over a Sample iterator.

    Output schema matches LeRobot v2 ``stats.json``::

        { "observation.<key>": {"mean", "std", "min", "max", "q01", "q99"},
          "action.<key>":      {...} }

    Thin wrapper over :class:`StreamingStats` so format writers and
    ad-hoc callers share the exact same accumulator.
    """
    from dimos.learning.formats._stats import StreamingStats

    s = StreamingStats(image_subsample=image_subsample,
                       quantile_reservoir=quantile_reservoir, seed=seed)
    for sample in samples:
        for k, v in sample.observation.items():
            s.update(f"observation.{k}", np.asarray(v))
        for k, v in sample.action.items():
            s.update(f"action.{k}", np.asarray(v))
    return s.finalize()


def get_writer(format_name: str) -> Writer:
    """Lazy-import the format writer's `write` function."""
    if format_name == "lerobot":
        from dimos.learning.formats.lerobot import write
    elif format_name == "hdf5":
        from dimos.learning.formats.hdf5 import write
    else:
        raise ValueError(f"Unknown format: {format_name!r}")
    return write
