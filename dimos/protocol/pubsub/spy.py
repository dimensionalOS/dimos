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

from dataclasses import dataclass
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
    raise NotImplementedError


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
        raise NotImplementedError

    def record(self, nbytes: int, timestamp: float) -> None:
        """Hot path: O(1) amortized append + eviction of entries older than history_window."""
        raise NotImplementedError

    def freq(self, window: float, now: float) -> float:
        """Messages per second over [now - window, now]. 0.0 if none."""
        raise NotImplementedError

    def bytes_per_sec(self, window: float, now: float) -> float:
        """Payload bytes per second over [now - window, now]. 0.0 if none."""
        raise NotImplementedError

    def avg_size(self, window: float, now: float) -> float:
        """Mean payload size in bytes over [now - window, now]. 0.0 if none."""
        raise NotImplementedError


@runtime_checkable
class SpySource(Protocol):
    """One transport's raw firehose feeding the spy.

    Invariants for implementations:
    - tap() delivers EVERY observable message exactly once per callback
      (non-conflating; built on the raw-bytes bus's subscribe_all).
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
        raise NotImplementedError

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def tap(self, callback: Callable[[str, int], None]) -> Callable[[], None]:
        raise NotImplementedError

    def subscribe_decoded(self, topic: str, callback: Callable[[Any], None]) -> Callable[[], None]:
        """Opt-in per-topic decoded tap — OFF the spy hot path. Not implemented in v1."""
        raise NotImplementedError


class ZenohSpySource:
    """Spy source over zenoh, via ZenohPubSubBase.subscribe_all (all keys, '**').

    Depends on the subscribe_all non-conflation fix (see TASK.md): every sample
    must reach the tap. nbytes = payload length; topics arrive as str(Topic)
    with the type suffix reconstructed from the key expression.
    """

    name = "zenoh"

    def __init__(self, **zenoh_kwargs: Any) -> None:
        """zenoh_kwargs forwarded to ZenohPubSubBase (e.g. mode/connect/listen, session_pool)."""
        raise NotImplementedError

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def tap(self, callback: Callable[[str, int], None]) -> Callable[[], None]:
        raise NotImplementedError

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
        raise NotImplementedError

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def snapshot(self) -> dict[SpyKey, TopicStats]:
        """Current per-topic stats, safe to iterate while messages keep arriving."""
        raise NotImplementedError


def default_sources() -> list[SpySource]:
    """The spy observes ALL transports simultaneously, regardless of DIMOS_TRANSPORT.

    v1: [LCMSpySource(), ZenohSpySource()]. SHM/ROS/DDS/Redis are future sources.
    """
    raise NotImplementedError
