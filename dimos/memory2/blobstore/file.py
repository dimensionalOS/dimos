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
from typing import TYPE_CHECKING

from dimos.memory2.backend import BlobStore

if TYPE_CHECKING:
    import os


class FileBlobStore(BlobStore):
    """Stores blobs as files on disk, one directory per stream.

    Layout::

        {root}/{stream}/{key}.bin
    """

    def __init__(self, root: str | os.PathLike[str]) -> None:
        self._root = Path(root)

    def _path(self, stream: str, key: int) -> Path:
        return self._root / stream / f"{key}.bin"

    # ── Resource lifecycle ────────────────────────────────────────

    def start(self) -> None:
        self._root.mkdir(parents=True, exist_ok=True)

    def stop(self) -> None:
        pass

    # ── BlobStore interface ───────────────────────────────────────

    def put(self, stream: str, key: int, data: bytes) -> None:
        p = self._path(stream, key)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)

    def get(self, stream: str, key: int) -> bytes:
        p = self._path(stream, key)
        try:
            return p.read_bytes()
        except FileNotFoundError:
            raise KeyError(f"No blob for stream={stream!r}, key={key}") from None

    def delete(self, stream: str, key: int) -> None:
        p = self._path(stream, key)
        p.unlink(missing_ok=True)
