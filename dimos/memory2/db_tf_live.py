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

"""In-RAM transform lookups — the default for any :class:`Store`."""

from __future__ import annotations

import math
import threading
from typing import TYPE_CHECKING

from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.msgs.geometry_msgs.Transform import Transform

# The transform streams the buffer loads (dynamic + latched static).
TF_STREAMS = ("tf", "tf_static")


class DbTfLive:
    """Transform lookups by loading the ``tf``/``tf_static`` streams into an in-RAM
    :class:`MultiTBuffer`. The default for any :class:`Store` — backend-agnostic (uses
    only the generic stream API). Holds the full history in memory, loaded once."""

    def __init__(self, store: Store) -> None:
        self._store = store
        self._lock = threading.Lock()
        self._buffer: MultiTBuffer | None = None

    def _ensure_loaded(self) -> MultiTBuffer:
        if self._buffer is not None:
            return self._buffer
        with self._lock:
            if self._buffer is not None:
                return self._buffer
            buffer = MultiTBuffer(buffer_size=math.inf)
            available = set(self._store.list_streams())
            for name in TF_STREAMS:
                if name not in available:
                    continue
                for observation in self._store.stream(name, TFMessage):
                    transforms = getattr(observation.data, "transforms", None) or [observation.data]
                    buffer.receive_transform(*transforms)
            self._buffer = buffer
            return buffer

    def has_transforms(self) -> bool:
        return bool(self._ensure_loaded().buffers)

    def get(
        self,
        target_frame: str,
        source_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        return self._ensure_loaded().lookup(target_frame, source_frame, time_point, time_tolerance)
