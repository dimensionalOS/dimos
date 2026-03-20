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

from pathlib import Path
from typing import Any

from dimos.memory2.blobstore.base import BlobStore, BlobStoreConfig
from dimos.memory2.utils.validation import validate_identifier


class FileBlobStoreConfig(BlobStoreConfig):
    root: str


class FileBlobStore(BlobStore):
    """Stores blobs as files on disk, one directory per stream.

    Layout::

        {root}/{stream}/{key}.bin
    """

    default_config = FileBlobStoreConfig
    config: FileBlobStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._root = Path(self.config.root)

    def _path(self, stream_name: str, key: int) -> Path:
        validate_identifier(stream_name)
        return self._root / stream_name / f"{key}.bin"

    def start(self) -> None:
        self._root.mkdir(parents=True, exist_ok=True)

    def stop(self) -> None:
        pass

    def put(self, stream_name: str, key: int, data: bytes) -> None:
        p = self._path(stream_name, key)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)

    def get(self, stream_name: str, key: int) -> bytes:
        p = self._path(stream_name, key)
        try:
            return p.read_bytes()
        except FileNotFoundError:
            raise KeyError(f"No blob for stream={stream_name!r}, key={key}") from None

    def delete(self, stream_name: str, key: int) -> None:
        p = self._path(stream_name, key)
        try:
            p.unlink()
        except FileNotFoundError:
            raise KeyError(f"No blob for stream={stream_name!r}, key={key}") from None
