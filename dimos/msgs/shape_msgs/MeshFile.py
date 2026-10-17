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

"""One mesh file's bytes as found on disk (STL, DAE, OBJ, ...), named by the URI a URDF uses for it."""

from __future__ import annotations

import struct
import time

from dimos.types.timestamped import Timestamped

_HEADER = struct.Struct("<dI")


class MeshFile(Timestamped):
    msg_name = "shape_msgs.MeshFile"

    def __init__(self, uri: str, data: bytes, ts: float | None = None) -> None:
        self.uri = uri
        self.data = data
        self.ts = time.time() if ts is None else ts

    def lcm_encode(self) -> bytes:
        uri = self.uri.encode()
        return _HEADER.pack(self.ts, len(uri)) + uri + self.data

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs: object) -> MeshFile:
        ts, uri_length = _HEADER.unpack_from(data)
        start = _HEADER.size + uri_length
        return cls(data[_HEADER.size : start].decode(), bytes(data[start:]), ts)
