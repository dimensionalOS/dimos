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
    from collections.abc import Collection

    from rerun import AsComponents, ComponentDescriptor, DescribedComponentBatch

# What the proxy charges for a message on top of its Arrow data. Measured on
# Rerun 0.32 at 1.1 kB for one component and 1.7 kB for three.
MESSAGE_OVERHEAD = 2048


def memory_limit_bytes(limit: str) -> int | None:
    """Parse a Rerun proxy budget. None means unlimited."""
    if limit in {"unlimited", "none", "max", "∞"}:
        return None
    if limit.endswith("%"):
        return int(psutil.virtual_memory().total * float(limit[:-1]) / 100)
    return int(TypeAdapter(ByteSize).validate_python(limit))


def _message_size(batches: Collection[DescribedComponentBatch]) -> int:
    return MESSAGE_OVERHEAD + sum(int(batch.as_arrow_array().nbytes) for batch in batches)


class StaticLog:
    """Refresh one-time static components before other writes age them out of the proxy."""

    def __init__(self, server_memory_limit: str) -> None:
        self._limit = memory_limit_bytes(server_memory_limit)
        self._remaining = (self._limit or 0) // 2
        self._once: dict[
            tuple[str, type], list[dict[ComponentDescriptor, DescribedComponentBatch]]
        ] = {}
        self._lock = threading.Lock()

    def log(self, path: str, *archetypes: AsComponents, once: bool = False) -> None:
        """Log static data. Pass once=True to retain its components for late viewers."""
        # Rerun is heavy, so only the bridge's worker imports it.
        import rerun as rr

        if not self._limit:  # Zero disables history. Unlimited history needs no refresh.
            rr.log(path, *archetypes, static=True)
            return

        with self._lock:
            rr.log(path, *archetypes, static=True)
            for archetype in archetypes:
                batches = archetype.as_component_batches()
                self._remaining -= _message_size(batches)
                key = (path.lstrip("/") or "/", type(archetype))
                if once or key in self._once:
                    rows = self._once.setdefault(key, [])
                    update = {batch.component_descriptor(): batch for batch in batches}
                    for row in rows:
                        for descriptor in update:
                            row.pop(descriptor, None)
                    # Keep row order. Rerun resolves a Transform3D from a single row,
                    # while Points3D can take positions and colors from different rows.
                    self._once[key] = [row for row in rows if row] + [update]

            if self._remaining > 0:
                return

            # Two copies of a large asset in one SDK chunk are evicted together.
            recording = rr.get_data_recording()
            if recording is not None:
                recording.flush(timeout_sec=0)
            headroom = self._limit
            for (path, _), rows in self._once.items():
                for row in rows:
                    rr.log(path, row.values(), static=True)
                    headroom -= _message_size(row.values())
            # If the retained data alone exceeds the budget, replaying on every
            # write would flood live viewers without making the snapshot fit.
            self._remaining = (headroom if headroom > 0 else self._limit) // 2
