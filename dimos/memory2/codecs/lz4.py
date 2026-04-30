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

from typing import TYPE_CHECKING, Any

import lz4.frame  # type: ignore[import-untyped]

if TYPE_CHECKING:
    from dimos.memory2.codecs.base import Codec


class Lz4Codec:
    """Wraps another codec and applies LZ4 frame compression to the output.

    Works with any inner codec — compresses the bytes produced by
    ``inner.encode()`` and decompresses before ``inner.decode()``.
    """

    def __init__(self, inner: Codec[Any], compression_level: int = 0) -> None:
        self._inner = inner
        self._compression_level = compression_level

    def encode(self, value: Any) -> bytes:
        raw = self._inner.encode(value)
        return bytes(lz4.frame.compress(raw, compression_level=self._compression_level))

    def decode(self, data: bytes) -> Any:
        raw: bytes = lz4.frame.decompress(data)
        return self._inner.decode(raw)
