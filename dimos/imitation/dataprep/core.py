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

Sub-configs (FeatureSpec, SyncConfig, QualityConfig, OutputConfig, EpisodeExtractor) and
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
from itertools import pairwise
import math
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, ConfigDict, Field, model_validator

from dimos.constants import STATE_DIR
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream

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


class FeatureSpec(BaseConfig):
    """Explicit dataset feature and its recorded source."""

    stream: str
    field: str | None = None
    dtype: str
    shape: tuple[int, ...]
    names: list[str]

    @model_validator(mode="after")
    def validate_schema(self) -> FeatureSpec:
        if not self.shape or any(value <= 0 for value in self.shape):
            raise ValueError("feature shape must contain positive dimensions")
        if self.dtype == "video":
            if len(self.names) != len(self.shape):
                raise ValueError("video feature names must name every axis")
        else:
            try:
                np.dtype(self.dtype)
            except TypeError as error:
                raise ValueError(f"unsupported feature dtype {self.dtype!r}") from error
            if len(self.shape) == 1 and len(self.names) != self.shape[0]:
                raise ValueError("vector feature names must match its length")
        if any(not name.strip() for name in self.names):
            raise ValueError("feature names must not be empty")
        return self


class SyncConfig(BaseConfig):
    anchor: str
    rate_hz: float
    tolerance_ms: float
    # TODO: add "interp" — do it per-stream (lerp low-dim vectors, force nearest
    # for ndim>=3 images, can't blend frames). Only "nearest" is wired today.
    strategy: Literal["nearest"] = "nearest"


class QualityConfig(BaseConfig):
    mode: Literal["strict", "fill"] = "strict"
    min_source_rate_ratio: float = 0.95
    max_camera_gap_ms: float = 100.0
    max_alignment_error_ms: float = 20.0


class OutputConfig(BaseConfig):
    format: Literal["lerobot", "hdf5"] = "lerobot"
    path: Path
    metadata: dict[str, Any] = Field(default_factory=dict)


class DataPrepConfig(BaseConfig):
    """Everything needed to turn a recording into a dataset.

    `source` is a recording `.db` or `.mcap`; `observation`/`action` map dataset feature
    names to recorded streams; `sync` resamples them onto a common timeline;
    `output` selects format + path. Consumed by `build.run_dataprep`.
    """

    source: str = ""
    episodes: EpisodeExtractor = EpisodeExtractor()
    observation: dict[str, FeatureSpec] = Field(default_factory=dict)
    action: dict[str, FeatureSpec] = Field(default_factory=dict)
    sync: SyncConfig = SyncConfig(anchor="image", rate_hz=DEFAULT_FPS, tolerance_ms=50.0)
    quality: QualityConfig = QualityConfig()
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


class IncompleteEpisode(BaseModel):
    start_ts: float
    task_label: str | None = None


class EpisodeReport(BaseModel):
    episodes: list[Episode] = Field(default_factory=list)
    incomplete: list[IncompleteEpisode] = Field(default_factory=list)


class Sample(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    ts: float
    episode_id: str
    observation: dict[str, NDArray[Any]]
    action: dict[str, NDArray[Any]]
    task_label: str | None = None  # carried from the episode for multi-task datasets
    complementary_info: dict[str, NDArray[Any]] = Field(default_factory=dict)


class EpisodeQualityReport(BaseModel):
    episode_id: str
    valid: bool
    mode: Literal["strict", "fill"]
    expected_frames: int = 0
    emitted_frames: int = 0
    filled_frames: int = 0
    source_rates_hz: dict[str, float] = Field(default_factory=dict)
    max_gaps_ms: dict[str, float] = Field(default_factory=dict)
    max_alignment_error_ms: float = 0.0
    rejection_reasons: list[str] = Field(default_factory=list)


# ─────────────────────────────────────────────────────────────────────────────
# Pure helpers — used by format writers and run_dataprep
# ─────────────────────────────────────────────────────────────────────────────


def resolve_field(msg: Any, ref: FeatureSpec) -> NDArray[Any]:
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


def extract_episodes(store: Store, cfg: EpisodeExtractor) -> list[Episode]:
    """Walk recorded events into Episodes per the configured strategy.

    EPISODE_STATUS: scan `cfg.status_stream` for state transitions emitted
        by `EpisodeMonitorModule`. State machine (mirrors the live monitor):
            ev.last_event == "start":   begin (auto-commit any prior pending)
            ev.last_event == "save":    commit (success=True)
            ev.last_event == "discard": drop (success=False)
            end of stream with pending: dropped (matches live spec)

    RANGES: emit one Episode per (start, end) tuple in `cfg.ranges`.
    """
    return inspect_episodes(store, cfg).episodes


def inspect_episodes(store: Store, cfg: EpisodeExtractor) -> EpisodeReport:
    """Extract completed episodes and retain any recording left open at EOF."""
    if cfg.extractor == "ranges":
        if not cfg.ranges:
            return EpisodeReport()
        return EpisodeReport(
            episodes=[
                Episode(id=f"ep_{i:06d}", start_ts=t0, end_ts=t1)
                for i, (t0, t1) in enumerate(cfg.ranges)
            ]
        )

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

    incomplete = (
        [IncompleteEpisode(start_ts=pending_start_ts, task_label=pending_label)]
        if pending_start_ts is not None
        else []
    )
    return EpisodeReport(episodes=episodes, incomplete=incomplete)


def iter_episode_samples(
    store: Store,
    episode: Episode,
    streams: dict[str, FeatureSpec],  # observation ∪ action
    sync: SyncConfig,
    quality: QualityConfig,
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

    In strict mode callers validate every target before iterating. In fill mode,
    a target without a nearby value uses the last value at or before the target
    and marks the frame as filled. Leading targets without a complete causal
    sample are trimmed rather than fabricated.
    """
    if sync.anchor not in streams:
        raise ValueError(f"sync.anchor {sync.anchor!r} not in streams: {sorted(streams)}")

    obs_keys = obs_keys if obs_keys is not None else set(streams)
    action_keys = action_keys if action_keys is not None else set()

    tolerance_s = min(sync.tolerance_ms, quality.max_alignment_error_ms) / 1000.0

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

    def _previous(key: str, t: float) -> Any | None:
        ts_list, msg_list = cached[key]
        i = bisect.bisect_right(ts_list, t) - 1
        return msg_list[i] if i >= 0 else None

    def _build_frames() -> Iterator[Sample]:
        for t in targets:
            obs_dict: dict[str, NDArray[Any]] = {}
            act_dict: dict[str, NDArray[Any]] = {}
            skip = False
            filled = False
            for key, ref in streams.items():
                msg = _nearest(key, t)
                if msg is None:
                    if quality.mode == "fill":
                        msg = _previous(key, t)
                        filled = msg is not None
                    if msg is None:
                        skip = True
                        break
                arr = resolve_field(msg, ref)
                if not is_image_array(arr):
                    arr = arr.astype(np.dtype(ref.dtype), copy=False)
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
                complementary_info={"is_filled": np.asarray([filled], dtype=np.bool_)},
            )

    yield from _build_frames()


def _feature_error(msg: Any, key: str, spec: FeatureSpec) -> str | None:
    """Return a schema error for one source message, if any."""
    try:
        value = resolve_field(msg, spec)
    except (AttributeError, KeyError, TypeError) as error:
        return f"{key}: cannot resolve {spec.stream}.{spec.field}: {error}"
    if tuple(value.shape) != spec.shape:
        return f"{key}: expected shape {spec.shape}, got {tuple(value.shape)}"
    if spec.dtype == "video":
        if not is_image_array(value) or value.dtype != np.uint8:
            return f"{key}: video source must be a uint8 image, got {value.dtype} {value.shape}"
    else:
        try:
            numeric = value.astype(np.dtype(spec.dtype), copy=False)
        except (TypeError, ValueError) as error:
            return f"{key}: cannot convert to {spec.dtype}: {error}"
        if np.issubdtype(numeric.dtype, np.number) and not np.isfinite(numeric).all():
            return f"{key}: contains non-finite values"
    source_names = getattr(msg, "name", None)
    if spec.field in {"position", "velocity", "effort"} and source_names is not None:
        if list(source_names) != spec.names:
            return f"{key}: expected names {spec.names}, got {list(source_names)}"
    return None


def inspect_episode_quality(
    store: Store,
    episode: Episode,
    streams: dict[str, FeatureSpec],
    sync: SyncConfig,
    quality: QualityConfig,
) -> EpisodeQualityReport:
    """Validate one saved episode without retaining payloads in memory."""
    report = EpisodeQualityReport(episode_id=episode.id, valid=True, mode=quality.mode)
    if sync.anchor not in streams:
        report.rejection_reasons.append(
            f"sync.anchor {sync.anchor!r} not in features {sorted(streams)}"
        )
        report.valid = False
        return report

    timestamps: dict[str, list[float]] = {}
    for key, spec in streams.items():
        values: list[float] = []
        schema_error: str | None = None
        source_stream: Stream[Any, Any] = store.stream(spec.stream)
        for observation in source_stream.time_range(episode.start_ts, episode.end_ts):
            values.append(observation.ts)
            if schema_error is None:
                schema_error = _feature_error(observation.data, key, spec)
        values.sort()
        timestamps[key] = values
        if not values:
            report.rejection_reasons.append(f"{key}: stream {spec.stream!r} has no episode data")
        if schema_error is not None:
            report.rejection_reasons.append(schema_error)

        if spec.dtype == "video":
            if len(values) > 1:
                duration = values[-1] - values[0]
                rate = (len(values) - 1) / duration if duration > 0 else 0.0
                gap_ms = max(b - a for a, b in pairwise(values)) * 1000.0
            else:
                rate = 0.0
                gap_ms = math.inf
            report.source_rates_hz[key] = rate
            report.max_gaps_ms[key] = gap_ms
            if quality.mode == "strict" and rate < sync.rate_hz * quality.min_source_rate_ratio:
                report.rejection_reasons.append(
                    f"{key}: source rate {rate:.2f}Hz is below "
                    f"{sync.rate_hz * quality.min_source_rate_ratio:.2f}Hz"
                )
            if quality.mode == "strict" and gap_ms > quality.max_camera_gap_ms:
                report.rejection_reasons.append(
                    f"{key}: maximum source gap {gap_ms:.1f}ms exceeds "
                    f"{quality.max_camera_gap_ms:.1f}ms"
                )

    anchor = timestamps.get(sync.anchor, [])
    if anchor and sync.rate_hz > 0:
        period = 1.0 / sync.rate_hz
        targets: list[float] = []
        target = anchor[0]
        while target <= anchor[-1]:
            targets.append(target)
            target += period
    else:
        targets = list(anchor)
    report.expected_frames = len(targets)

    tolerance_s = min(sync.tolerance_ms, quality.max_alignment_error_ms) / 1000.0
    filled = 0
    emitted = 0
    max_error_s = 0.0
    for target in targets:
        target_filled = False
        complete = True
        for values in timestamps.values():
            if not values:
                complete = False
                break
            index = bisect.bisect_left(values, target)
            candidates = values[max(0, index - 1) : min(len(values), index + 1)]
            error = min(abs(value - target) for value in candidates)
            max_error_s = max(max_error_s, error)
            if error > tolerance_s:
                if quality.mode == "fill" and bisect.bisect_right(values, target) > 0:
                    target_filled = True
                else:
                    complete = False
                    break
        if complete:
            emitted += 1
            filled += int(target_filled)
        elif quality.mode == "strict":
            report.rejection_reasons.append(
                f"fixed-rate target at {target:.6f} has no complete aligned sample"
            )
            break

    report.emitted_frames = emitted
    report.filled_frames = filled
    report.max_alignment_error_ms = max_error_s * 1000.0
    if not targets:
        report.rejection_reasons.append("anchor stream has no episode data")
    if emitted == 0:
        report.rejection_reasons.append("episode has no complete output frames")
    report.valid = not report.rejection_reasons
    return report


def get_writer(format_name: str) -> Writer:
    """Lazy-import the format writer's `write` function."""
    if format_name == "lerobot":
        raise RuntimeError(
            "LeRobot conversion requires its isolated environment; "
            "run it through `dimos dataprep build`"
        )
    elif format_name == "hdf5":
        from dimos.imitation.dataprep.formats.hdf5.writer import write
    else:
        raise ValueError(f"Unknown format: {format_name!r}")
    return write


def get_inspector(format_name: str) -> Inspector:
    """Lazy-import the format reader's `inspect` function."""
    if format_name == "lerobot":
        from dimos.imitation.dataprep.formats.lerobot.reader import inspect
    elif format_name == "hdf5":
        from dimos.imitation.dataprep.formats.hdf5.reader import inspect
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
