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

"""Current scene state that outlives Rerun's bounded message history."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import pyarrow as pa
    from rerun import AsComponents, ComponentDescriptor, DescribedComponentBatch


@dataclass
class _ArrowBatch:
    values: pa.Array

    def as_arrow_array(self) -> pa.Array:
        return self.values


@dataclass
class _Component:
    batch: DescribedComponentBatch
    sent_at: float


class Scene:
    """Keep one value per retained component and refresh values no longer being logged.

    Rerun 0.32's gRPC proxy retains messages, not the latest scene: even static
    data can be evicted. Periodic refresh also repairs viewers already connected
    to a proxy whose history is too small to hold the complete scene.
    """

    def __init__(self) -> None:
        self._components: dict[tuple[str, bool], dict[ComponentDescriptor, _Component]] = {}
        self._lock = threading.Lock()
        self._stopped = False

    def log(
        self,
        path: str,
        *entities: AsComponents,
        static: bool = False,
        retain: bool = False,
    ) -> None:
        # Rerun is optional/heavy; load it only in the visualization worker.
        import pyarrow as pa
        import rerun as rr

        with self._lock:
            if self._stopped:
                return
            now = time.monotonic()
            key = (path, static)
            for entity in entities:
                if isinstance(entity, rr.Clear):
                    if entity.is_recursive is None:
                        continue
                    recursive_values = entity.is_recursive.as_arrow_array()
                    if not len(recursive_values):
                        continue
                    recursive = bool(recursive_values[0].as_py())
                    for (cached_path, cached_static), components in self._components.items():
                        if cached_static == static and (
                            cached_path == path
                            or (recursive and cached_path.startswith(path.rstrip("/") + "/"))
                        ):
                            for descriptor, component in components.items():
                                component.batch = rr.DescribedComponentBatch(
                                    _ArrowBatch(
                                        pa.array([], type=component.batch.as_arrow_array().type)
                                    ),
                                    descriptor,
                                )
                                component.sent_at = now
                elif static or retain or key in self._components:
                    components = self._components.setdefault(key, {})
                    for batch in entity.as_component_batches():
                        descriptor = batch.component_descriptor()
                        if static or retain or descriptor in components:
                            components[descriptor] = _Component(batch, now)
            rr.log(path, *entities, static=static)

    def refresh(self, interval: float) -> None:
        # Rerun is optional/heavy; load it only in the visualization worker.
        import rerun as rr

        with self._lock:
            if self._stopped:
                return
            now = time.monotonic()
            for (path, static), components in self._components.items():
                stale = [value for value in components.values() if now - value.sent_at >= interval]
                if stale:
                    rr.log(path, [value.batch for value in stale], static=static)
                    for value in stale:
                        value.sent_at = now

    def stop(self) -> None:
        with self._lock:
            self._stopped = True
            self._components.clear()
