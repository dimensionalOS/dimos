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

from typing import TYPE_CHECKING

from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.msgs.geometry_msgs.Transform import Transform

WORLD_FRAME = "world"
TF_STREAM = "tf"


class DbTf:
    """Read-only TF tree backed by a recording's ``tf`` stream.

    Loads every recorded :class:`TFMessage` into a :class:`MultiTBuffer` (with
    an unbounded time window so lookups work at any point across the whole
    recording) and exposes :meth:`get` for ``parent <- child`` lookups. Use it
    to register frame-tagged observations into ``world`` at replay time instead
    of trusting a baked-in per-observation pose.

    Lookups use an unbounded time tolerance by default: static transforms
    (mounts, ``world <- map``) are published once at the start of a recording,
    so a fixed tolerance would drop them for every later observation. Densely
    sampled dynamic transforms still resolve to their nearest sample.
    """

    def __init__(self, buffer: MultiTBuffer) -> None:
        self._buffer = buffer

    @classmethod
    def from_store(cls, store: Store, stream_name: str = TF_STREAM) -> DbTf | None:
        """Build a :class:`DbTf` from ``store``'s tf stream, or ``None`` if absent."""
        if stream_name not in store.list_streams():
            return None
        buffer = MultiTBuffer(buffer_size=float("inf"))
        for observation in store.stream(stream_name, TFMessage):
            buffer.receive_transform(*observation.data.transforms)
        return cls(buffer)

    def get(
        self,
        target_frame: str,
        source_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        """Transform mapping a point in ``source_frame`` into ``target_frame`` at
        ``time_point`` (``None`` if no chain connects them)."""
        return self._buffer.get(
            target_frame, source_frame, time_point=time_point, time_tolerance=time_tolerance
        )

    @property
    def frames(self) -> set[str]:
        return self._buffer.get_frames()
