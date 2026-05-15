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

from __future__ import annotations

from abc import ABC, abstractmethod
import collections
import inspect
import math
import time
from typing import TYPE_CHECKING, Any, Generic, Literal, TypeVar, cast

from dimos.memory2.utils.formatting import FilterRepr

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator, Sequence

    from dimos.memory2.stream import Stream
    from dimos.memory2.type.observation import Observation

T = TypeVar("T")
R = TypeVar("R")


class Transformer(FilterRepr, ABC, Generic[T, R]):
    """Transforms a stream of observations lazily via iterator -> iterator.

    Pull from upstream, yield transformed observations. Naturally supports
    batching, windowing, fan-out. The generator cleans
    up when upstream exhausts.
    """

    @abstractmethod
    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[R]]: ...

    def __str__(self) -> str:
        parts: list[str] = []
        for name in inspect.signature(self.__init__).parameters:  # type: ignore[misc]
            for attr in (name, f"_{name}"):
                if hasattr(self, attr):
                    val = getattr(self, attr)
                    if callable(val):
                        parts.append(f"{name}={getattr(val, '__name__', '...')}")
                    else:
                        parts.append(f"{name}={val!r}")
                    break
        return f"{self.__class__.__name__}({', '.join(parts)})"


class FnTransformer(Transformer[T, R]):
    """Wraps a callable that receives an Observation and returns a new one (or None to skip)."""

    def __init__(self, fn: Callable[[Observation[T]], Observation[R] | None]) -> None:
        self._fn = fn

    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[R]]:
        fn = self._fn
        for obs in upstream:
            result = fn(obs)
            if result is not None:
                yield result


class FnIterTransformer(Transformer[T, R]):
    """Wraps a bare ``Iterator → Iterator`` callable (e.g. a generator function)."""

    def __init__(self, fn: Callable[[Iterator[Observation[T]]], Iterator[Observation[R]]]) -> None:
        self._fn = fn

    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[R]]:
        return self._fn(upstream)


class Batch(Transformer[T, R]):
    """Batched transform: collects observations, applies a batch function, derives new data.

    The ``fn`` receives a list of data items and returns a list of results,
    one per input (e.g. ``model.caption_batch``, ``model.embed``).
    """

    def __init__(self, fn: Callable[[list[T]], Sequence[R]], batch_size: int = 16) -> None:
        self._fn = fn
        self._batch_size = batch_size

    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[R]]:
        fn = self._fn
        batch: list[Observation[T]] = []
        for obs in upstream:
            batch.append(obs)
            if len(batch) >= self._batch_size:
                results = fn([o.data for o in batch])
                for o, r in zip(batch, results, strict=True):
                    yield o.derive(data=r)
                batch = []
        if batch:
            results = fn([o.data for o in batch])
            for o, r in zip(batch, results, strict=True):
                yield o.derive(data=r)


def downsample(n: int) -> FnIterTransformer[T, T]:
    """Yield every *n*-th observation, skipping the rest."""
    if n < 1:
        raise ValueError(f"downsample(n) requires n >= 1, got {n}")

    def _downsample(upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        for i, obs in enumerate(upstream):
            if i % n == 0:
                yield obs

    return FnIterTransformer(_downsample)


def throttle(interval: float) -> FnIterTransformer[T, T]:
    """Yield at most one observation per *interval* seconds."""
    if interval <= 0:
        raise ValueError(f"throttle(interval) requires interval > 0, got {interval}")

    def _throttle(upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        last_ts: float | None = None
        for obs in upstream:
            if last_ts is None or obs.ts - last_ts >= interval:
                last_ts = obs.ts
                yield obs

    return FnIterTransformer(_throttle)


def measure_time(out: Stream[float]) -> FnIterTransformer[T, T]:
    """Returns a transformer that records per-frame downstream cost (ms) into *out*."""

    def _xf(upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        for obs in upstream:
            start = time.perf_counter()
            yield obs
            out.append((time.perf_counter() - start) * 1000, ts=obs.ts)

    return FnIterTransformer(_xf)


def measure_gpu_mem(out: Stream[float], device: int = 0) -> FnIterTransformer[T, T]:
    """Returns a transformer that records device-level GPU VRAM used (MB) per frame into *out*.

    Reads ``torch.cuda.mem_get_info`` *after* each yield (and ``synchronize()``
    so async kernels submitted by the downstream consumer have completed),
    capturing memory used by every allocator on the device — Open3D, torch,
    other processes — not just the current process.
    """
    import open3d.core as o3c  # type: ignore[import-untyped]
    import torch

    if not o3c.cuda.is_available():
        raise RuntimeError("measure_gpu_mem requires a CUDA-capable GPU")

    def _xf(upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        for obs in upstream:
            yield obs
            torch.cuda.synchronize(device)
            free, total = torch.cuda.mem_get_info(device)
            out.append((total - free) / 1_000_000, ts=obs.ts)

    return FnIterTransformer(_xf)


def speed() -> FnIterTransformer[Any, float]:
    """Compute speed (m/s) between consecutive observations from their poses."""

    def _speed(upstream: Iterator[Observation[Any]]) -> Iterator[Observation[float]]:
        prev: Observation[Any] | None = None
        for obs in upstream:
            if prev is not None and obs.pose is not None and prev.pose is not None:
                dx = obs.pose[0] - prev.pose[0]
                dy = obs.pose[1] - prev.pose[1]
                dz = obs.pose[2] - prev.pose[2]
                dt = obs.ts - prev.ts
                v = math.sqrt(dx * dx + dy * dy + dz * dz) / dt if dt > 0 else 0.0
                yield obs.derive(data=v)
            prev = obs

    return FnIterTransformer(_speed)


def smooth(window: int) -> FnIterTransformer[float, float]:
    """Sliding window average over obs.data (must be numeric)."""

    def _smooth(upstream: Iterator[Observation[float]]) -> Iterator[Observation[float]]:
        buf: collections.deque[float] = collections.deque(maxlen=window)
        for obs in upstream:
            buf.append(obs.data)
            yield obs.derive(data=sum(buf) / len(buf))

    return FnIterTransformer(_smooth)


def peaks(
    prominence: float = 0.02,
    distance: float = 5.0,
    width: float | None = 0.5,
    key: Callable[[Observation[T]], float] | None = None,
) -> FnIterTransformer[T, T]:
    """Yield only the local-maximum observations, gated by peak shape.

    Runs scipy.signal.find_peaks on a scalar extracted from each observation
    and emits the qualifying observations in timestamp order. Each yielded
    observation gets its peak's prominence stashed on ``tags["peak_prominence"]``.

    All parameters are in the natural units of the stream (seconds and
    data-range units), not sample counts. Time-based parameters are
    converted to sample counts internally using the median sample spacing.

    - ``prominence``: minimum topological prominence to keep. Assumes the
      upstream data is roughly normalized to [0, 1]; with default 0.1 a peak
      has to stick up at least 10% of the range above its surroundings.
      Pass 0.0 to return *every* local maximum with its prominence attached
      — useful for plotting the distribution and picking a threshold.
    - ``distance``: minimum time in seconds between detected peaks.
    - ``width``: minimum peak width in seconds at 50% prominence. Filters
      sub-second noise spikes. Pass ``None`` to disable.
    - ``key``: callable that extracts the scalar signal from an observation.
      Defaults to ``obs.data``. Use this when ``obs.data`` isn't the scalar
      you want to detect peaks on (e.g. image observations with a
      ``similarity`` metadata field).
    """
    from scipy.signal import find_peaks

    key_fn: Callable[[Observation[T]], float] = (
        key if key is not None else cast("Callable[[Observation[T]], float]", lambda obs: obs.data)
    )

    def _peaks(upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        items = list(upstream)
        if len(items) < 3:
            return
        values = [key_fn(obs) for obs in items]

        # Median sample spacing — used to convert seconds → samples
        # consistently for both `distance` and `width`.
        spacings = sorted(items[i + 1].ts - items[i].ts for i in range(len(items) - 1))
        median_spacing = spacings[len(spacings) // 2] if spacings else 0.0

        def seconds_to_samples(seconds: float | None) -> int | None:
            if seconds is None or median_spacing <= 0:
                return None
            return max(1, round(seconds / median_spacing))

        # Always pass a numeric `prominence` so scipy populates props["prominences"].
        # Passing None would skip the computation, leaving tags empty.
        idx, props = find_peaks(
            values,
            prominence=prominence,
            distance=seconds_to_samples(distance),
            width=seconds_to_samples(width),
        )
        proms = props["prominences"]

        for i, prom in zip(idx, proms, strict=True):
            yield items[int(i)].tag(peak_prominence=float(prom))

    return FnIterTransformer(_peaks)


def _median(sorted_vals: list[float]) -> float:
    n = len(sorted_vals)
    return sorted_vals[n // 2] if n % 2 else 0.5 * (sorted_vals[n // 2 - 1] + sorted_vals[n // 2])


def _mad_threshold(values: list[float], k: float) -> tuple[float, float, float]:
    """Returns (threshold, median, scale) where scale = MAD * 1.4826."""
    median = _median(sorted(values))
    scale = _median(sorted(abs(v - median) for v in values)) * 1.4826
    return median + k * scale, median, scale


def _otsu_threshold(values: list[float]) -> float:
    """1D Otsu threshold: maximizes between-class variance over the value list."""
    sorted_vals = sorted(values)
    n = len(sorted_vals)
    total = sum(sorted_vals)
    best_var, best_thresh = -1.0, sorted_vals[-1]
    cum = 0.0
    for i in range(n - 1):
        cum += sorted_vals[i]
        count = i + 1
        w0, w1 = count / n, (n - count) / n
        m0, m1 = cum / count, (total - cum) / (n - count)
        var = w0 * w1 * (m0 - m1) ** 2
        if var > best_var:
            best_var, best_thresh = var, 0.5 * (sorted_vals[i] + sorted_vals[i + 1])
    return best_thresh


def _gap_threshold(values: list[float]) -> float:
    """Largest log-ratio gap between consecutive sorted values."""
    sorted_vals = sorted(v for v in values if v > 0)
    n = len(sorted_vals)
    if n < 2:
        return sorted(values)[len(values) // 2] if values else 0.0
    best_ratio, best_idx = 0.0, n - 1
    for i in range(n - 1):
        ratio = sorted_vals[i + 1] / sorted_vals[i]
        if ratio > best_ratio:
            best_ratio, best_idx = ratio, i
    return 0.5 * (sorted_vals[best_idx] + sorted_vals[best_idx + 1])


def significant(
    method: Literal["mad", "otsu", "gap"] = "mad",
    k: float = 3.0,
    tag: str = "peak_prominence",
) -> FnIterTransformer[T, T]:
    """Keep observations whose ``tags[tag]`` is an outlier in its own distribution.

    Designed to chain after :func:`peaks` so the cutoff is *derived from the
    prominence distribution itself*, invariant to overall signal range. The
    upstream :func:`peaks` call still does the shape gating (``distance``,
    ``width``, and a small ``prominence`` floor to reject obvious noise);
    :func:`significant` then picks a statistical cutoff from what survives.

    Each surviving observation gets ``tags["significance"]`` attached.

    - ``method``:
        - ``"mad"``: keep values above ``median + k * 1.4826 * MAD``. Robust
          default; assumes most upstream values are noise. ``significance``
          is the resulting (value - median) / scale, i.e. a robust z-score.
        - ``"otsu"``: 1D Otsu — picks the threshold maximizing between-class
          variance over the value distribution. Parameter-free; works when
          the distribution is roughly bimodal. ``significance`` is value /
          threshold.
        - ``"gap"``: largest ratio gap between consecutive sorted values.
          Crisp when peaks are well separated from noise, brittle otherwise
          (a single tiny value at the bottom of the list can dominate).
          ``significance`` is value / threshold.
    - ``k``: only used by ``"mad"`` (≈3 ≙ 3-sigma equivalent).
    - ``tag``: which tag holds the scalar to threshold on. Defaults to
      ``peak_prominence`` (set by :func:`peaks`).
    """
    if method not in ("mad", "otsu", "gap"):
        raise ValueError(f"unknown method {method!r}; expected 'mad', 'otsu', or 'gap'")

    def _significant(upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        items = list(upstream)
        if len(items) < 2:
            return
        try:
            values = [float(o.tags[tag]) for o in items]
        except KeyError as e:
            raise ValueError(
                f"significant() requires upstream observations to be tagged with {tag!r}; "
                f"chain after peaks() or set tag= to a tag that exists"
            ) from e

        if method == "mad":
            threshold, median, scale = _mad_threshold(values, k)
            for obs, val in zip(items, values, strict=True):
                if val >= threshold and scale > 0:
                    yield obs.tag(significance=(val - median) / scale)
        else:
            threshold = _otsu_threshold(values) if method == "otsu" else _gap_threshold(values)
            for obs, val in zip(items, values, strict=True):
                if val >= threshold:
                    yield obs.tag(significance=val / threshold if threshold > 0 else 0.0)

    return FnIterTransformer(_significant)


def smooth_time(seconds: float) -> FnIterTransformer[float, float]:
    """Sliding window average over obs.data, by time.

    Averages all observations whose timestamp is within ``seconds`` of the
    current observation's timestamp. Unlike ``smooth(window)`` (which uses a
    fixed sample count and so depends on sampling rate), the effective window
    here adapts: dense regions average more samples, sparse regions average
    fewer.
    """
    if seconds <= 0:
        raise ValueError(f"smooth_time(seconds) requires seconds > 0, got {seconds}")

    def _smooth(upstream: Iterator[Observation[float]]) -> Iterator[Observation[float]]:
        buf: collections.deque[Observation[float]] = collections.deque()
        for obs in upstream:
            buf.append(obs)
            while buf and obs.ts - buf[0].ts > seconds:
                buf.popleft()
            yield obs.derive(data=sum(o.data for o in buf) / len(buf))

    return FnIterTransformer(_smooth)


def normalize() -> FnIterTransformer[float, float]:
    """Normalize obs.data to [0, 1] range across all observations."""

    def _normalize(upstream: Iterator[Observation[float]]) -> Iterator[Observation[float]]:
        items = list(upstream)
        if not items:
            return
        values = [obs.data for obs in items]
        lo, hi = min(values), max(values)
        for obs in items:
            t = (obs.data - lo) / (hi - lo) if hi != lo else 0.5
            yield obs.derive(data=t)

    return FnIterTransformer(_normalize)


class QualityWindow(Transformer[T, T]):
    """Keeps the highest-quality item per time window.

    Emits the best observation when the window advances. The last window
    is emitted when the upstream iterator exhausts — no flush needed.
    """

    def __init__(self, quality_fn: Callable[[Any], float], window: float) -> None:
        self._quality_fn = quality_fn
        self._window = window

    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        quality_fn = self._quality_fn
        window = self._window
        best: Observation[T] | None = None
        best_score: float = -1.0
        window_start: float | None = None

        for obs in upstream:
            if window_start is not None and (obs.ts - window_start) >= window:
                if best is not None:
                    yield best
                best = None
                best_score = -1.0
                window_start = obs.ts

            score = quality_fn(obs.data)
            if score > best_score:
                best = obs
                best_score = score
            if window_start is None:
                window_start = obs.ts

        if best is not None:
            yield best
