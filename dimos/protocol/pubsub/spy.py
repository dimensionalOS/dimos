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

"""Transport-agnostic pubsub spy: topic discovery, rates, sizes, liveness.

Design doc: docs/usage/transports/spy.md. Task spec: TASK.md (branch agent/spy-architect).

HARD CONSTRAINT: the spy never decodes message payloads. Sources tap the
raw-bytes pubsub layer (LCMPubSubBase, ZenohPubSubBase — beneath the encoder
mixins), so the hot path per message is (topic string, payload length,
timestamp) and nothing else. Message *types* are still visible because they
are embedded in the topic string ("/cmd_vel#geometry_msgs.Twist").

Decoding is a separate, per-topic, opt-in concern: SpySource.subscribe_decoded
is the spec'd hook, not implemented in v1.
"""

from __future__ import annotations

from collections import deque
import contextlib
from dataclasses import dataclass
import sys
import threading
import time
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

    # The entire hot-path event: (topic string incl. '#type' suffix, wire payload length).
    TapCallback = Callable[[str, int], None]


@dataclass(frozen=True, slots=True)
class SpyKey:
    """Identity of one spied topic: which transport saw it + its raw topic string."""

    transport: str  # SpySource.name, e.g. "lcm", "zenoh"
    topic: str  # raw transport topic, e.g. "/cmd_vel#geometry_msgs.Twist"


def split_type_suffix(topic: str) -> tuple[str, str | None]:
    """Split a spied topic string into (base_topic, msg_type_name or None).

    Both LCM and zenoh sources deliver topics in the uniform str(Topic) form
    "base#pkg.Msg" (zenoh keys are converted back by _key_expr_to_topic).
    Render-time helper — never called on the hot path.

    >>> split_type_suffix("/cmd_vel#geometry_msgs.Twist")
    ('/cmd_vel', 'geometry_msgs.Twist')
    >>> split_type_suffix("/plain")
    ('/plain', None)
    """
    base, sep, suffix = topic.partition("#")
    return (base, suffix) if sep else (topic, None)


class TopicStats:
    """Sliding-window traffic statistics for one spied topic.

    Records only (timestamp, nbytes) pairs — no payloads are retained.
    Timestamps are passed in explicitly (callers use time.time(); tests inject
    values), so all stats are deterministic functions of recorded data.

    Thread-safety: record() may be called from transport threads while readers
    query from a UI thread.
    """

    total_bytes: int
    total_msgs: int
    last_seen: float | None  # timestamp of newest recorded message, None if none yet

    def __init__(self, history_window: float = 60.0) -> None:
        """history_window: seconds of per-message history kept for windowed stats.

        total_bytes/total_msgs/last_seen survive eviction; only windowed stats
        (freq/bytes_per_sec/avg_size) forget evicted messages.
        """
        self.history_window = history_window
        self._history: deque[tuple[float, int]] = deque()  # (timestamp, nbytes)
        self._lock = threading.Lock()
        self.total_bytes = 0
        self.total_msgs = 0
        self.last_seen = None

    def record(self, nbytes: int, timestamp: float) -> None:
        """Hot path: O(1) amortized append + eviction of entries older than history_window."""
        with self._lock:
            self._history.append((timestamp, nbytes))
            self.total_bytes += nbytes
            self.total_msgs += 1
            self.last_seen = timestamp
            cutoff = timestamp - self.history_window
            while self._history and self._history[0][0] < cutoff:
                self._history.popleft()

    def _in_window(self, window: float, now: float) -> list[tuple[float, int]]:
        cutoff = now - window
        with self._lock:
            return [(ts, n) for ts, n in self._history if ts >= cutoff]

    def freq(self, window: float, now: float) -> float:
        """Messages per second over [now - window, now]. 0.0 if none."""
        msgs = self._in_window(window, now)
        return len(msgs) / window if msgs else 0.0

    def bytes_per_sec(self, window: float, now: float) -> float:
        """Payload bytes per second over [now - window, now]. 0.0 if none."""
        msgs = self._in_window(window, now)
        return sum(n for _, n in msgs) / window if msgs else 0.0

    def avg_size(self, window: float, now: float) -> float:
        """Mean payload size in bytes over [now - window, now]. 0.0 if none."""
        msgs = self._in_window(window, now)
        return sum(n for _, n in msgs) / len(msgs) if msgs else 0.0


@runtime_checkable
class SpySource(Protocol):
    """One transport's raw firehose feeding the spy.

    Invariants for implementations:
    - tap() rides the raw-bytes bus's subscribe_all; delivery scope and
      conflation follow that transport's semantics (LCM delivers every
      message; zenoh's subscribe_all is latest-per-topic over dimos/**).
    - The tap callback receives (topic_str, nbytes) where topic_str is the
      uniform str(Topic) form and nbytes is the wire payload length.
    - Never decodes payloads, never retains them past the callback.
    - start() is required before tap() delivers; stop() releases the bus.
    """

    name: str

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def tap(self, callback: Callable[[str, int], None]) -> Callable[[], None]: ...

    def subscribe_decoded(
        self, topic: str, callback: Callable[[Any], None]
    ) -> Callable[[], None]: ...


class LCMSpySource:
    """Spy source over LCM, via LCMPubSubBase.subscribe_all (raw regex '.*').

    Delivers raw channel strings incl. the '#pkg.Msg' suffix. Payloads are the
    LCM-encoded bytes; nbytes = len(payload).
    """

    name = "lcm"

    def __init__(self, **lcm_kwargs: Any) -> None:
        """lcm_kwargs forwarded to LCMPubSubBase (e.g. lcm_url)."""
        from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase

        self._bus = LCMPubSubBase(**lcm_kwargs)

    def start(self) -> None:
        self._bus.start()

    def stop(self) -> None:
        self._bus.stop()

    def tap(self, callback: Callable[[str, int], None]) -> Callable[[], None]:
        return self._bus.subscribe_all(lambda msg, topic: callback(str(topic), len(msg)))

    def subscribe_decoded(self, topic: str, callback: Callable[[Any], None]) -> Callable[[], None]:
        """Opt-in per-topic decoded tap — OFF the spy hot path. Not implemented in v1."""
        raise NotImplementedError


class ZenohSpySource:
    """Spy source over zenoh, via ZenohPubSubBase.subscribe_all.

    zenoh's subscribe_all is latest-per-topic over dimos/** (best-effort), so
    same-topic bursts between drains conflate away. nbytes = payload length;
    topics arrive as str(Topic) with the type suffix reconstructed from the key.
    """

    name = "zenoh"

    def __init__(self, **zenoh_kwargs: Any) -> None:
        """zenoh_kwargs forwarded to ZenohPubSubBase (e.g. mode/connect/listen, session_pool)."""
        from dimos.protocol.pubsub.impl.zenohpubsub import ZenohPubSubBase

        self._bus = ZenohPubSubBase(**zenoh_kwargs)

    def start(self) -> None:
        self._bus.start()

    def stop(self) -> None:
        self._bus.stop()

    def tap(self, callback: Callable[[str, int], None]) -> Callable[[], None]:
        return self._bus.subscribe_all(lambda msg, topic: callback(str(topic), len(msg)))

    def subscribe_decoded(self, topic: str, callback: Callable[[Any], None]) -> Callable[[], None]:
        """Opt-in per-topic decoded tap — OFF the spy hot path. Not implemented in v1."""
        raise NotImplementedError


class TransportSpy:
    """Aggregates SpySources into per-(transport, topic) stats plus global totals.

    Owns the sources' lifecycle: start() starts every source and taps it;
    stop() untaps and stops them. Stats rows appear lazily as topics are first
    seen (a topic with no traffic since start is invisible — the spy observes,
    it does not enumerate).

    Thread-safety: tap callbacks arrive on transport threads; snapshot() may be
    called from any thread and returns a consistent view for rendering.
    """

    totals: TopicStats  # all messages across all sources

    def __init__(
        self, sources: Sequence[SpySource] | None = None, history_window: float = 60.0
    ) -> None:
        """sources=None means default_sources()."""
        self._sources = list(sources) if sources is not None else default_sources()
        self._history_window = history_window
        self._stats: dict[SpyKey, TopicStats] = {}
        self._lock = threading.Lock()
        self._untaps: list[Callable[[], None]] = []
        self._live: list[SpySource] = []  # sources currently started + tapped
        self.totals = TopicStats(history_window=history_window)

    def _tap_callback(self, transport: str) -> Callable[[str, int], None]:
        def on_message(topic: str, nbytes: int) -> None:
            now = time.time()
            key = SpyKey(transport, topic)
            with self._lock:
                stats = self._stats.get(key)
                if stats is None:
                    stats = TopicStats(history_window=self._history_window)
                    self._stats[key] = stats
            stats.record(nbytes, now)
            self.totals.record(nbytes, now)

        return on_message

    def _start_one(self, source: SpySource) -> None:
        """Start + tap one source; if tap fails, stop it before propagating."""
        source.start()
        try:
            untap = source.tap(self._tap_callback(source.name))
        except BaseException:
            with contextlib.suppress(Exception):
                source.stop()
            raise
        self._live.append(source)
        self._untaps.append(untap)

    def start(self, best_effort: bool = False) -> None:
        """Start and tap every source.

        Strict (default): all-or-nothing — if any source fails to start or tap,
        roll back the ones already started and re-raise.
        best_effort: a source that fails start()/tap() is warned and skipped
        (its own partial start is undone); the survivors keep running. Raises
        only if no source starts at all.
        """
        try:
            for source in self._sources:
                try:
                    self._start_one(source)
                except Exception as exc:
                    if not best_effort:
                        raise
                    print(f"spy: skipping transport {source.name!r}: {exc}", file=sys.stderr)
        except BaseException:
            self.stop()  # roll back everything started so far, then propagate
            raise
        if best_effort and not self._live:
            raise RuntimeError(
                f"no spy transports could start (tried: {', '.join(s.name for s in self._sources)})"
            )

    def stop(self) -> None:
        for untap in self._untaps:
            untap()
        self._untaps.clear()
        for source in self._live:
            source.stop()
        self._live.clear()

    def snapshot(self) -> dict[SpyKey, TopicStats]:
        """Current per-topic stats, safe to iterate while messages keep arriving."""
        with self._lock:
            return dict(self._stats)


SOURCE_FACTORIES: dict[str, Callable[[], SpySource]] = {
    LCMSpySource.name: LCMSpySource,
    ZenohSpySource.name: ZenohSpySource,
}
"""Known transports by name; each factory constructs its source only when called.

v1: lcm + zenoh. SHM/ROS/DDS/Redis are future sources.
"""


def default_sources() -> list[SpySource]:
    """The spy observes ALL transports simultaneously, regardless of DIMOS_TRANSPORT.

    A backend that fails to construct (missing import, native init error, …) is
    skipped with a warning, so the default spy degrades to whatever transports
    are available. Requesting a transport explicitly (constructing its
    SOURCE_FACTORIES entry) keeps the hard error.
    """
    sources: list[SpySource] = []
    for name, factory in SOURCE_FACTORIES.items():
        try:
            sources.append(factory())
        except Exception as exc:  # degrade on any backend init failure, not just imports
            print(f"spy: skipping unavailable transport {name!r}: {exc}", file=sys.stderr)
    if not sources:
        raise RuntimeError(f"no spy transports available (tried: {', '.join(SOURCE_FACTORIES)})")
    return sources
