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

from abc import abstractmethod
from typing import Any

from dimos.core.resource import CompositeResource
from dimos.memory2.registry import qual
from dimos.protocol.service.spec import BaseConfig, Configurable


class BlobStoreConfig(BaseConfig):
    pass


class BlobStore(Configurable, CompositeResource):
    """Persistent storage for encoded payload blobs.

    Separates payload data from metadata indexing so that large blobs
    (images, point clouds) don't penalize metadata queries.
    """

    config: BlobStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        Configurable.__init__(self, **kwargs)
        CompositeResource.__init__(self)

    @abstractmethod
    def put(self, stream_name: str, key: int, data: bytes) -> None:
        """Store a blob for the given stream and observation id."""
        ...

    @abstractmethod
    def get(self, stream_name: str, key: int) -> bytes:
        """Retrieve a blob by stream name and observation id."""
        ...

    @abstractmethod
    def delete(self, stream_name: str, key: int) -> None:
        """Delete a blob by stream name and observation id."""
        ...

    def serialize(self) -> dict[str, Any]:
        return {"class": qual(type(self)), "config": self.config.model_dump()}
