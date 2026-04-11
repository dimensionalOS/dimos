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

from typing import Any

from dimos.memory2.backend import Backend
from dimos.memory2.observationstore.memory import ListObservationStore
from dimos.memory2.store.base import Store, StoreConfig


class MemoryStoreConfig(StoreConfig):
    max_size: int | None = None


class MemoryStore(Store):
    """In-memory store for experimentation.

    ``max_size`` controls how many observations each stream retains:
    - ``None`` (default) — keep all (unbounded).
    - ``N`` — rolling window of the most recent N observations.
    - ``0`` — discard immediately (live-only, no history).
    """

    config: MemoryStoreConfig

    def _create_backend(
        self, name: str, payload_type: type[Any] | None = None, **config: Any
    ) -> Backend[Any]:
        if "observation_store" not in config and self.config.observation_store is None:
            obs: ListObservationStore[Any] = ListObservationStore(
                name=name, max_size=self.config.max_size
            )
            config["observation_store"] = obs
        return super()._create_backend(name, payload_type, **config)
