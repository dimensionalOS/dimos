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

"""TF service backed by a recorded ``tf`` stream."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, cast

from dimos.memory2.stream import Stream
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer, TFConfig, TFSpec

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.protocol.tf.tf import TFLookup


class StreamTFConfig(TFConfig):
    stream: Stream[TFMessage] | None = (
        None  # Required field but needs default for config inheritance
    )
    # Prefetch span (s) cached past a missed query window, so chronological
    # replay costs one db query per cache_span of progress. Also the cache
    # size bound: a miss evicts everything before re-caching.
    cache_span: float = 300.0


class StreamTF(MultiTBuffer, TFSpec):
    """A tf service whose backend is a recorded memory2 ``tf`` stream.

    The read-side mirror of :class:`~dimos.protocol.tf.tf.PubSubTF`: the same
    :class:`MultiTBuffer` cache and lookup API, but ingestion pulls windows
    from the stream on demand instead of receiving pushed messages.

    Lookups reach as far as they would against the live service: with no
    explicit ``time_tolerance`` a query window spans ``buffer_size`` seconds
    backward (what a live buffer would still hold) plus ``forward_tolerance``
    ahead — the recorded-time analog of the live wall-clock wait for future
    transforms, which is why lookups here never block. A cache miss fetches
    the window plus ``cache_span`` beyond it in one query. The cache pins the
    underlying :class:`MultiTBuffer` to infinite retention: insert-time
    pruning would silently delete data the cache still claims to hold, so
    eviction is explicit (miss → full clear → re-cache).
    """

    config: StreamTFConfig

    def __init__(self, stream: Stream[TFMessage] | None = None, **kwargs: Any) -> None:
        if stream is not None:
            kwargs["stream"] = stream
        TFSpec.__init__(self, **kwargs)
        MultiTBuffer.__init__(self, buffer_size=math.inf)

        if self.config.stream is None:
            raise ValueError("Stream configuration is missing")
        self.stream = self.config.stream

        self._covered: tuple[float, float] | None = None

    @classmethod
    def from_store(cls, store: Any, stream: str = "tf") -> StreamTF | None:
        if stream not in store.list_streams():
            return None
        return cls(store.stream(stream, TFMessage))

    def publish(self, *args: Transform) -> None:
        raise NotImplementedError("StreamTF is a read-only replay service.")

    def publish_static(self, *args: Transform) -> None:
        raise NotImplementedError("StreamTF is a read-only replay service.")

    def _load(self, lo: float, hi: float) -> None:
        # at() windows are boundary-inclusive; from/to_timestamp are strict and
        # would skip messages stamped exactly at the stream's first timestamp.
        for obs in self.stream.at((lo + hi) / 2, (hi - lo) / 2):
            self.receive_transform(*obs.data.transforms)

    def _ensure(self, lo: float, hi: float) -> None:
        """Serve ``[lo, hi]`` from the cache, else re-cache ``[lo, hi + cache_span]``.

        The prefetch past ``hi`` makes chronological replay cost one db query
        per ``cache_span`` of progress. A miss evicts everything first — a full
        clear (not partial pruning) keeps ``_covered`` truthful.
        """
        if self._covered is not None:
            clo, chi = self._covered
            if clo <= lo and hi <= chi:
                return
            with self._cv:
                self.buffers.clear()
        self._load(lo, hi + self.config.cache_span)
        self._covered = (lo, hi + self.config.cache_span)

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
        *,
        forward_tolerance: float = 0.0,
    ) -> Transform | None:
        tp = time_point
        if tp is None:
            last = next(iter(self.stream.order_by("ts", desc=True).limit(1)), None)
            tp = last.ts if last is not None else None
        if tp is not None:
            back = time_tolerance if time_tolerance is not None else self.config.buffer_size
            fwd = time_tolerance if time_tolerance is not None else forward_tolerance
            self._ensure(tp - back, tp + fwd)
        # The recorded-time lookahead above stands in for the live wall-clock
        # wait, so the base lookup must never block.
        return super().get(
            parent_frame,
            child_frame,
            time_point,
            time_tolerance,
            forward_tolerance=0.0,
        )


if TYPE_CHECKING:
    # mypy conformance check: StreamTF satisfies the read-side tf protocol.
    _lookup_impl: TFLookup = cast("StreamTF", None)
