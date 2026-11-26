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

"""A file named by its URI, with its sha256 and mtime when known, and its bytes unless it is only a header.

A request is a FileInfo with no content, carrying the sha256 the requester already has (if any);
the reply is header only when that sha256 is current, else the whole file.
"""

from __future__ import annotations

import struct
import time

from dimos.types.timestamped import Timestamped

_TS_FLAGS = struct.Struct("<dB")
_LENGTH = struct.Struct("<I")
_HASH_LENGTH = struct.Struct("<H")
_MTIME = struct.Struct("<d")

_HAS_SHA256 = 1
_HAS_MTIME = 2
_HAS_CONTENT = 4


class FileInfo(Timestamped):
    msg_name = "files.FileInfo"

    def __init__(
        self,
        uri: str,
        sha256: str | None = None,
        mtime: float | None = None,
        content: bytes | None = None,
        ts: float | None = None,
    ) -> None:
        self.uri = uri
        # "sha256:<64 hex>", as an asset's assetInfo version is written.
        self.sha256 = sha256
        # Seconds since the epoch.
        self.mtime = mtime
        # None is a header only; b"" is an empty file.
        self.content = content
        self.ts = time.time() if ts is None else ts

    def header(self) -> FileInfo:
        """The same file without its content."""
        return FileInfo(self.uri, self.sha256, self.mtime, None, self.ts)

    def lcm_encode(self) -> bytes:
        flags = (
            (_HAS_SHA256 if self.sha256 is not None else 0)
            | (_HAS_MTIME if self.mtime is not None else 0)
            | (_HAS_CONTENT if self.content is not None else 0)
        )
        uri = self.uri.encode()
        parts = [_TS_FLAGS.pack(self.ts, flags), _LENGTH.pack(len(uri)), uri]
        if self.sha256 is not None:
            digest = self.sha256.encode()
            parts += [_HASH_LENGTH.pack(len(digest)), digest]
        if self.mtime is not None:
            parts.append(_MTIME.pack(self.mtime))
        if self.content is not None:
            parts.append(self.content)
        return b"".join(parts)

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs: object) -> FileInfo:
        ts, flags = _TS_FLAGS.unpack_from(data)
        at = _TS_FLAGS.size
        (uri_length,) = _LENGTH.unpack_from(data, at)
        at += _LENGTH.size
        uri = bytes(data[at : at + uri_length]).decode()
        at += uri_length
        sha256 = None
        if flags & _HAS_SHA256:
            (hash_length,) = _HASH_LENGTH.unpack_from(data, at)
            at += _HASH_LENGTH.size
            sha256 = bytes(data[at : at + hash_length]).decode()
            at += hash_length
        mtime = None
        if flags & _HAS_MTIME:
            (mtime,) = _MTIME.unpack_from(data, at)
            at += _MTIME.size
        content = bytes(data[at:]) if flags & _HAS_CONTENT else None
        return cls(uri, sha256, mtime, content, ts)

    def __repr__(self) -> str:
        size = "header only" if self.content is None else f"{len(self.content)} bytes"
        return f"FileInfo({self.uri!r}, {self.sha256!r}, mtime={self.mtime!r}, {size})"
