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

"""Tick assembly — align N observation streams into per-tick input rows.

A :class:`~dimos.memory2.puremodule.PureModule` declares *when* it runs
(one ``tick()`` input) and *how* every other input is sampled at that
moment (``latest()``, ``interpolate()``, ``window()``). This module holds
the samplers and the :class:`TickMachine` that does the alignment.

The machine is a plain event-in/rows-out state machine — no threads, no
streams — so the exact same code drives both execution modes:

- **offline** (stored streams): events arrive in ascending-``ts`` order
  via a heap-merge; alignment is exact and deterministic.
- **live** (pubsub ports): events arrive in wall-clock order from a
  queue; alignment is best-effort under arrival jitter.

Samplers answer "what is the value of this input at time t?":

- ``tick()`` — the input that *defines* t; each observation fires a tick.
- ``latest(max_age=None)`` — newest observation with ``ts <= t``
  (zero-order hold). Never delays a tick.
- ``interpolate(tolerance=0.5)`` — bracket t between the surrounding
  observations and interpolate (lerp for numbers, lerp+slerp for poses).
  Needs one observation with ``ts >= t``, so a live tick waits for the
  next sample on this input (~one sample period of latency). When no
  bracket is possible (stream ended / not yet started), falls back to the
  nearest observation within ``tolerance`` seconds.
- ``window(seconds)`` — every observation with ``t - seconds < ts <= t``,
  as a list (e.g. all IMU samples since 0.1s before the frame).

A missing value (e.g. ``latest`` older than ``max_age``) becomes ``None``
when the step parameter is typed ``X | None``, otherwise the tick is
dropped.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import bisect
from dataclasses import dataclass, field
import heapq
import logging
from typing import TYPE_CHECKING, Any

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator

    from dimos.memory2.type.observation import Observation

logger = logging.getLogger(__name__)


class _Blocked:
    """Sentinel: this tick must wait for a future observation (live mode)."""

    __slots__ = ()

    def __repr__(self) -> str:
        return "<blocked>"


class _Missing:
    """Sentinel: no value available for this input at this tick."""

    __slots__ = ()

    def __repr__(self) -> str:
        return "<missing>"


BLOCKED = _Blocked()
MISSING = _Missing()


# -- Interpolation ----------------------------------------------------------


def _lerp(a: float, b: float, alpha: float) -> float:
    return a + (b - a) * alpha


def _slerp(
    qa: tuple[float, float, float, float],
    qb: tuple[float, float, float, float],
    alpha: float,
) -> tuple[float, float, float, float]:
    """Spherical lerp between two (x, y, z, w) quaternions."""
    import math

    ax, ay, az, aw = qa
    bx, by, bz, bw = qb
    dot = ax * bx + ay * by + az * bz + aw * bw
    if dot < 0.0:  # take the short path
        bx, by, bz, bw = -bx, -by, -bz, -bw
        dot = -dot
    if dot > 0.9995:  # nearly parallel — nlerp to avoid div-by-~0
        x, y, z, w = (
            _lerp(ax, bx, alpha),
            _lerp(ay, by, alpha),
            _lerp(az, bz, alpha),
            _lerp(aw, bw, alpha),
        )
        n = math.sqrt(x * x + y * y + z * z + w * w)
        return (x / n, y / n, z / n, w / n)
    theta = math.acos(min(1.0, dot))
    s = math.sin(theta)
    wa = math.sin((1.0 - alpha) * theta) / s
    wb = math.sin(alpha * theta) / s
    return (
        ax * wa + bx * wb,
        ay * wa + by * wb,
        az * wa + bz * wb,
        aw * wa + bw * wb,
    )


def _interp_pose_tuple(a: Any, b: Any, alpha: float) -> tuple[float, ...]:
    """Interpolate two pose-like objects (with .position/.orientation) to a 7-tuple."""
    pa, pb = a.position, b.position
    return (
        _lerp(pa.x, pb.x, alpha),
        _lerp(pa.y, pb.y, alpha),
        _lerp(pa.z, pb.z, alpha),
        *_slerp(
            (a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w),
            (b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w),
            alpha,
        ),
    )


def interp_data(a: Any, b: Any, alpha: float, t: float) -> Any:
    """Interpolate two data values; falls back to nearest-neighbor for unknown types.

    Supported: int/float (lerp), :class:`Pose` (lerp + slerp),
    :class:`PoseStamped` (lerp + slerp, ``ts=t``). Anything else returns
    the temporally nearer of the two.
    """
    if isinstance(a, bool) or isinstance(b, bool):  # bool is int — don't lerp it
        return a if alpha < 0.5 else b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return _lerp(float(a), float(b), alpha)
    if isinstance(a, PoseStamped) and isinstance(b, PoseStamped):
        x, y, z, qx, qy, qz, qw = _interp_pose_tuple(a, b, alpha)
        return PoseStamped(ts=t, position=(x, y, z), orientation=(qx, qy, qz, qw))
    if isinstance(a, Pose) and isinstance(b, Pose):
        return Pose(*_interp_pose_tuple(a, b, alpha))
    return a if alpha < 0.5 else b


# -- Samplers ----------------------------------------------------------------


class Sampler(ABC):
    """How to read one input's value at a tick time *t* from its buffer.

    Samplers also carry the input's *declared health contracts*
    (``expect_hz``, ``max_missing_ratio``) — class-level defaults that
    deployment config may override per key. Alignment ignores them; the
    health monitor reads them off the plan.
    """

    expect_hz: float | None = None
    max_missing_ratio: float | None = None

    def _init_contracts(self, expect_hz: float | None, max_missing_ratio: float | None) -> None:
        if expect_hz is not None and expect_hz <= 0:
            raise ValueError(f"expect_hz must be > 0, got {expect_hz}")
        if max_missing_ratio is not None and not (0.0 < max_missing_ratio <= 1.0):
            raise ValueError(f"max_missing_ratio must be in (0, 1], got {max_missing_ratio}")
        self.expect_hz = expect_hz
        self.max_missing_ratio = max_missing_ratio

    @abstractmethod
    def sample(self, buf: list[Observation[Any]], t: float, exhausted: bool) -> Any:
        """Return an Observation (or list), MISSING, or BLOCKED.

        ``buf`` is in ascending-``ts`` order. ``exhausted`` means no more
        observations will ever arrive on this input (offline end-of-stream
        or module shutdown) — samplers must not return BLOCKED then.
        """

    def min_keep_ts(self, bound: float) -> float:
        """Observations with ``ts`` strictly below this can be pruned,
        given no future tick will be earlier than *bound*."""
        return bound

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}()"


def _ts_list(buf: list[Observation[Any]]) -> list[float]:
    return [o.ts for o in buf]


class Tick(Sampler):
    """Marks the input whose observations fire the ticks."""

    def __init__(self, expect_hz: float | None = None) -> None:
        self._init_contracts(expect_hz, None)

    def sample(self, buf: list[Observation[Any]], t: float, exhausted: bool) -> Any:
        raise RuntimeError("Tick sampler is never sampled — it drives the clock")


class Latest(Sampler):
    """Newest observation with ``ts <= t``; MISSING if none (or older than max_age)."""

    def __init__(
        self,
        max_age: float | None = None,
        expect_hz: float | None = None,
        max_missing_ratio: float | None = None,
    ) -> None:
        if max_age is not None and max_age <= 0:
            raise ValueError(f"latest(max_age) requires max_age > 0, got {max_age}")
        self.max_age = max_age
        self._init_contracts(expect_hz, max_missing_ratio)

    def sample(self, buf: list[Observation[Any]], t: float, exhausted: bool) -> Any:
        i = bisect.bisect_right(_ts_list(buf), t) - 1
        if i < 0:
            return MISSING
        obs = buf[i]
        if self.max_age is not None and t - obs.ts > self.max_age:
            return MISSING
        return obs

    def __repr__(self) -> str:
        return f"Latest(max_age={self.max_age})"


class Interpolate(Sampler):
    """Bracket *t* between surrounding observations and interpolate the data.

    Live, a tick waits (BLOCKED) until this input produces an observation
    with ``ts >= t`` — one sample period of latency. With no bracket
    available (input ended, or t precedes its first observation) the
    nearest observation within ``tolerance`` seconds is used as-is.
    """

    def __init__(
        self,
        tolerance: float = 0.5,
        expect_hz: float | None = None,
        max_missing_ratio: float | None = None,
    ) -> None:
        if tolerance <= 0:
            raise ValueError(f"interpolate(tolerance) requires tolerance > 0, got {tolerance}")
        self.tolerance = tolerance
        self._init_contracts(expect_hz, max_missing_ratio)

    def sample(self, buf: list[Observation[Any]], t: float, exhausted: bool) -> Any:
        ts = _ts_list(buf)
        right_i = bisect.bisect_left(ts, t)
        left_i = right_i - 1
        right = buf[right_i] if right_i < len(buf) else None
        left = buf[left_i] if left_i >= 0 else None

        if right is None and not exhausted:
            return BLOCKED  # the bracketing observation may still arrive
        if left is not None and right is not None:
            dt = right.ts - left.ts
            alpha = (t - left.ts) / dt if dt > 0 else 0.0
            data = interp_data(left.data, right.data, alpha, t)
            pose: Any = None
            if left.pose is not None and right.pose is not None:
                pose = _interp_pose_tuple(left.pose, right.pose, alpha)
            elif left.pose is not None:
                pose = left.pose_tuple
            return left.derive(data=data, ts=t, pose_tuple=tuple(pose) if pose else None)
        nearest = left if left is not None else right
        if nearest is not None and abs(nearest.ts - t) <= self.tolerance:
            return nearest
        return MISSING

    def __repr__(self) -> str:
        return f"Interpolate(tolerance={self.tolerance})"


class Window(Sampler):
    """All observations with ``t - seconds < ts <= t``, as a list (may be empty)."""

    def __init__(
        self,
        seconds: float,
        expect_hz: float | None = None,
    ) -> None:
        if seconds <= 0:
            raise ValueError(f"window(seconds) requires seconds > 0, got {seconds}")
        self.seconds = seconds
        self._init_contracts(expect_hz, None)

    def sample(self, buf: list[Observation[Any]], t: float, exhausted: bool) -> Any:
        ts = _ts_list(buf)
        lo = bisect.bisect_right(ts, t - self.seconds)
        hi = bisect.bisect_right(ts, t)
        return list(buf[lo:hi])

    def min_keep_ts(self, bound: float) -> float:
        return bound - self.seconds

    def __repr__(self) -> str:
        return f"Window(seconds={self.seconds})"


def tick(expect_hz: float | None = None) -> Any:
    """This input fires the ticks — the module steps once per observation.

    Declared as a port default: ``image: In[Image] = tick()``. Exactly one
    input must be the tick. ``expect_hz`` declares the input's expected
    arrival rate as a class-level health contract (deployment config
    overrides). (Typed ``Any`` so it can sit on an ``In[X]`` annotation,
    pydantic-``Field()`` style.)
    """
    return Tick(expect_hz=expect_hz)


def latest(
    max_age: float | None = None,
    expect_hz: float | None = None,
    max_missing_ratio: float | None = None,
) -> Any:
    """Sample this input as the newest observation at the tick time (hold).

    ``expect_hz``/``max_missing_ratio`` declare class-level health
    contracts for this input (deployment config overrides per key).
    """
    return Latest(max_age, expect_hz=expect_hz, max_missing_ratio=max_missing_ratio)


def interpolate(
    tolerance: float = 0.5,
    expect_hz: float | None = None,
    max_missing_ratio: float | None = None,
) -> Any:
    """Sample this input by interpolating to the tick time (lerp/slerp)."""
    return Interpolate(tolerance, expect_hz=expect_hz, max_missing_ratio=max_missing_ratio)


def window(seconds: float, expect_hz: float | None = None) -> Any:
    """Sample this input as the list of observations in the trailing window."""
    return Window(seconds, expect_hz=expect_hz)


# -- The machine -------------------------------------------------------------


@dataclass
class _FieldState:
    sampler: Sampler
    buf: list[Observation[Any]] = field(default_factory=list)
    exhausted: bool = False


class TickMachine:
    """Aligns observation events from N named inputs into per-tick rows.

    Feed events with :meth:`process`; finish with :meth:`flush`. Each
    returned row is ``(trigger_obs, {input_name: Observation | list |
    MISSING})`` — policy (optional vs drop, data vs obs) is the caller's.

    Ticks resolve in order: the oldest pending tick blocks the rest, so
    output order always matches trigger order.
    """

    def __init__(
        self, trigger: str, samplers: dict[str, Sampler], max_pending: int | None = None
    ) -> None:
        """``max_pending`` caps ticks waiting for interpolation brackets.

        When a live ``interpolate()`` input dies, every trigger would
        otherwise queue forever. With the cap, the oldest pending tick is
        dropped (counted in :attr:`blocked_dropped`) — controller
        semantics. Pass ``None`` offline, where exactness matters and the
        ts-ordered merge bounds pending naturally.
        """
        self.trigger = trigger
        self.fields = {name: _FieldState(s) for name, s in samplers.items()}
        self.pending: list[Observation[Any]] = []
        self.max_pending = max_pending
        self.blocked_dropped = 0  # ticks evicted by the max_pending cap
        self._last_t = float("-inf")  # ts of the newest resolved tick

    def process(
        self, name: str, obs: Observation[Any]
    ) -> list[tuple[Observation[Any], dict[str, Any]]]:
        """Feed one observation event; return any ticks it resolved."""
        if name == self.trigger:
            self.pending.append(obs)
            if self.max_pending is not None and len(self.pending) > self.max_pending:
                del self.pending[0]
                self.blocked_dropped += 1
        else:
            fs = self.fields[name]
            # Live arrival can be slightly out of order across sources;
            # keep the buffer ts-sorted so samplers can bisect.
            if fs.buf and obs.ts < fs.buf[-1].ts:
                bisect.insort(fs.buf, obs, key=lambda o: o.ts)
            else:
                fs.buf.append(obs)
        return self._resolve()

    def end_of_stream(self, name: str) -> list[tuple[Observation[Any], dict[str, Any]]]:
        """Mark one input as finished (no more observations will arrive)."""
        if name != self.trigger:
            self.fields[name].exhausted = True
        return self._resolve()

    def flush(self) -> list[tuple[Observation[Any], dict[str, Any]]]:
        """Mark every input finished and resolve all pending ticks."""
        for fs in self.fields.values():
            fs.exhausted = True
        return self._resolve()

    def _resolve(self) -> list[tuple[Observation[Any], dict[str, Any]]]:
        rows: list[tuple[Observation[Any], dict[str, Any]]] = []
        while self.pending:
            tobs = self.pending[0]
            row: dict[str, Any] = {}
            blocked = False
            for name, fs in self.fields.items():
                val = fs.sampler.sample(fs.buf, tobs.ts, fs.exhausted)
                if val is BLOCKED:
                    blocked = True
                    break
                row[name] = val
            if blocked:
                break
            self.pending.pop(0)
            self._last_t = tobs.ts
            rows.append((tobs, row))
        self._prune()
        return rows

    def _prune(self) -> None:
        """Drop buffer entries no future tick can need."""
        for fs in self.fields.values():
            if self.pending:
                bound = self.pending[0].ts
            elif fs.buf:
                # No tick waiting — the next one can't be much older than
                # what we've already seen (1s slack for live arrival jitter).
                bound = max(self._last_t, fs.buf[-1].ts - 1.0)
            else:
                continue
            keep_from = fs.sampler.min_keep_ts(bound)
            ts = _ts_list(fs.buf)
            # Keep the newest obs at-or-before keep_from (it's the next
            # tick's "latest"/left bracket), drop everything older.
            i = bisect.bisect_right(ts, keep_from) - 1
            if i > 0:
                del fs.buf[:i]


# -- Offline driver -----------------------------------------------------------


def merge_events(
    trigger_name: str,
    trigger_iter: Iterator[Observation[Any]],
    streams: dict[str, Iterable[Observation[Any]]],
) -> Iterator[tuple[str, Observation[Any]]]:
    """K-way merge of observation iterators into one ascending-``ts`` event feed.

    At equal ``ts``, non-trigger observations sort first so a tick sees
    same-timestamp data from other inputs. Assumes each input iterates in
    ascending ``ts`` (true for stored streams in insertion order — prepend
    ``.order_by("ts")`` otherwise).
    """

    def tag(name: str, it: Iterable[Observation[Any]], prio: int) -> Iterator[tuple[Any, ...]]:
        return ((obs.ts, prio, name, obs) for obs in it)

    feeds = [tag(name, it, 0) for name, it in streams.items()]
    feeds.append(tag(trigger_name, trigger_iter, 1))
    for _ts, _prio, name, obs in heapq.merge(*feeds):
        yield name, obs
