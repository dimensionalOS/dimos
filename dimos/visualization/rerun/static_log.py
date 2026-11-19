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

"""Static logging that outlives the replay buffer of Rerun's gRPC proxy."""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING

import psutil
from pydantic import ByteSize, TypeAdapter

if TYPE_CHECKING:
    from rerun import AsComponents

# What the proxy charges for a message on top of its Arrow data. Measured on
# Rerun 0.32 at 1.1 kB for one component and 1.7 kB for three.
MESSAGE_OVERHEAD = 2048


def memory_limit_bytes(limit: str) -> int:
    """A Rerun memory limit in bytes, from a size ("1GiB") or a share of RAM ("25%")."""
    if limit.endswith("%"):
        return int(psutil.virtual_memory().total * float(limit[:-1]) / 100)
    return int(TypeAdapter(ByteSize).validate_python(limit))


class StaticLog:
    """Log static data, repeating what is logged only once before the proxy drops it.

    Rerun's gRPC proxy replays its buffer to a viewer that connects late. It
    drops static messages last, but it never merges them. Once static data alone
    exceeds the server memory limit, the oldest static messages go.

    Static topic data is published again by its source. What the bridge logs
    only once is not, so it is kept here and logged again each time half a
    buffer of static data has gone by.
    """

    def __init__(self, server_memory_limit: str) -> None:
        self._replay_after = memory_limit_bytes(server_memory_limit) // 2
        self._once: dict[str, dict[type, AsComponents]] = {}
        self._pending = 0
        self._lock = threading.Lock()

    def log(self, path: str, *archetypes: AsComponents, once: bool = False) -> None:
        """Log static data. Pass once=True when nothing will log it again."""
        # Rerun is heavy, so only the bridge's worker imports it.
        import rerun as rr

        # The lock also keeps a replay from logging an older value after a newer one.
        with self._lock:
            if once:
                self._once.setdefault(path, {}).update(
                    {type(archetype): archetype for archetype in archetypes}
                )
            rr.log(path, *archetypes, static=True)
            self._pending += MESSAGE_OVERHEAD + sum(
                batch.as_arrow_array().nbytes
                for archetype in archetypes
                for batch in archetype.as_component_batches()
            )
            if self._pending < self._replay_after:
                return
            self._pending = 0
            for once_path, kept in self._once.items():
                rr.log(once_path, *kept.values(), static=True)
