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

import importlib
from typing import Any, Protocol, TypeVar, runtime_checkable

T = TypeVar("T")


@runtime_checkable
class Codec(Protocol[T]):
    """Encode/decode payloads for storage."""

    def encode(self, value: T) -> bytes: ...
    def decode(self, data: bytes) -> T: ...


def codec_for(payload_type: type[Any] | None = None) -> Codec[Any]:
    """Auto-select codec based on payload type."""
    from dimos.memory2.codecs.pickle import PickleCodec

    if payload_type is not None:
        from dimos.msgs.sensor_msgs.Image import Image

        if issubclass(payload_type, Image):
            from dimos.memory2.codecs.jpeg import JpegCodec

            return JpegCodec()
        if hasattr(payload_type, "lcm_encode") and hasattr(payload_type, "lcm_decode"):
            from dimos.memory2.codecs.lcm import LcmCodec

            return LcmCodec(payload_type)
    return PickleCodec()


def codec_id(codec: Codec[Any]) -> str:
    """Derive a string ID from a codec instance, e.g. ``'lz4+lcm'``.

    Walks the ``_inner`` chain for wrapper codecs, joining with ``+``.
    Uses the naming convention ``FooCodec`` → ``'foo'``.
    """
    parts: list[str] = []
    c: Any = codec
    while hasattr(c, "_inner"):
        parts.append(_class_to_id(c))
        c = c._inner
    parts.append(_class_to_id(c))
    return "+".join(parts)


def codec_from_id(codec_id_str: str, payload_module: str) -> Codec[Any]:
    """Reconstruct a codec chain from its string ID (e.g. ``'lz4+lcm'``).

    Builds inside-out: the rightmost segment is the innermost (base) codec.
    """
    parts = codec_id_str.split("+")
    # Innermost first
    result = _make_one(parts[-1], payload_module)
    for name in reversed(parts[:-1]):
        result = _make_one(name, payload_module, inner=result)
    return result


def _class_to_id(codec: Any) -> str:
    name = type(codec).__name__
    if name.endswith("Codec"):
        return name[:-5].lower()
    return name.lower()


def _resolve_payload_type(payload_module: str) -> type[Any]:
    parts = payload_module.rsplit(".", 1)
    if len(parts) != 2:
        raise ValueError(f"Cannot resolve payload type from {payload_module!r}")
    mod = importlib.import_module(parts[0])
    return getattr(mod, parts[1])  # type: ignore[no-any-return]


def _make_one(name: str, payload_module: str, inner: Codec[Any] | None = None) -> Codec[Any]:
    """Instantiate a single codec by its short name."""
    if name == "lz4":
        from dimos.memory2.codecs.lz4 import Lz4Codec

        if inner is None:
            raise ValueError("lz4 is a wrapper codec — must have an inner codec")
        return Lz4Codec(inner)
    if name == "jpeg":
        from dimos.memory2.codecs.jpeg import JpegCodec

        return JpegCodec()
    if name == "lcm":
        from dimos.memory2.codecs.lcm import LcmCodec

        return LcmCodec(_resolve_payload_type(payload_module))
    if name == "pickle":
        from dimos.memory2.codecs.pickle import PickleCodec

        return PickleCodec()
    raise ValueError(f"Unknown codec: {name!r}")
