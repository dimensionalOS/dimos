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

"""TF lookups backed by a recorded ``tf`` stream."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, cast

from dimos.memory.stream import Stream
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.protocol.tf.tf import TFLookup


def tf_stream(store: Any, stream: str = "tf") -> Stream[TFMessage] | None:
    """The recording's tf stream, or None if it has none."""
    # check if it's there first so we don't create one
    if stream not in store.list_streams():
        return None
    return cast("Stream[TFMessage]", store.stream(stream, TFMessage))


class StreamTF(MultiTBuffer):
    def __init__(
        self,
        stream: Stream[TFMessage] | None = None,
        cache_span: float = 300.0,
        default_tolerance: float = 10.0,
    ) -> None:
        MultiTBuffer.__init__(self, buffer_size=math.inf)

        if stream is None:
            raise ValueError("Stream configuration is missing")
        self.stream = stream
        self.cache_span = cache_span
        self.default_tolerance = default_tolerance

        self._covered: tuple[float, float] | None = None

    @classmethod
    def from_store(cls, store: Any, stream: str = "tf") -> StreamTF | None:
        recorded = tf_stream(store, stream)
        return None if recorded is None else cls(recorded)

    def publish(self, *args: Transform) -> None:
        raise NotImplementedError("StreamTF is a read-only replay service.")

    def _load(self, lo: float, hi: float) -> None:
        for obs in self.stream.at((lo + hi) / 2, (hi - lo) / 2):
            self.receive_transform(*obs.data.transforms)
        self._covered = (lo, hi)

    def _ensure(self, lo: float, hi: float) -> None:
        """Serve ``[lo, hi]`` from the cache, else re-cache ``[lo, hi + cache_span]``."""
        with self._cv:
            if self._covered is not None:
                clo, chi = self._covered
                if clo <= lo and hi <= chi:
                    return
                self.buffers.clear()
                self._covered = None
            self._load(lo, hi + self.cache_span)

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
            back = time_tolerance if time_tolerance is not None else self.default_tolerance
            fwd = time_tolerance if time_tolerance is not None else forward_tolerance
            self._ensure(tp - back, tp + fwd)

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
