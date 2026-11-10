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


def codec_for(payload_type: type[Any] | None = None, sample: Any = None) -> Codec[Any]:
    """Auto-select codec based on payload type.

    ``sample`` is a first payload, when one is at hand. ``Image`` covers depth as
    well as colour, and JPEG would destroy depth values, so a depth or 16-bit
    sample selects lossless ``lz4+lcm`` instead.
    """
    from dimos.memory.codecs.pickle import PickleCodec

    if payload_type is not None and hasattr(payload_type, "__msgtype__"):
        from dimos.memory.codecs.cdr import CdrCodec

        return CdrCodec(payload_type)
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


_HISTORICAL_ALIASES = {
    "nav_msgs/msg/LineSegments3D": "dimos_msgs/msg/LineSegments3D",
    "geometry_msgs/msg/Transform": "geometry_msgs/msg/TransformStamped",
}


def legacy_payload_type(payload_module: str) -> type[Any] | None:
    """Resolve a retired ``dimos.msgs.<pkg>.<Msg>.<Msg>`` name to its generated type.

    Only the historical built-in namespace, in its recorded ``<pkg>.<Msg>.<Msg>``
    spelling, is accepted; nothing here can import an arbitrary Python module.
    Returns ``None`` for anything that does not name a registered message, so
    callers can leave the channel undecoded (raw bytes) instead of guessing.
    """
    parts = payload_module.split(".")
    if len(parts) < 5 or parts[0] != "dimos" or parts[1] != "msgs":
        return None
    package, message = parts[2], parts[3]
    if not package.endswith("_msgs") or parts[4:] != [message]:
        return None
    name = _HISTORICAL_ALIASES.get(f"{package}/msg/{message}", f"{package}/msg/{message}")
    from dimos.protocol.pubsub.message_types import resolve_message_type

    return resolve_message_type(name)


def recorded_payload_type(payload_module: str) -> type[Any] | None:
    """Resolve a recorded ``dimos.payload_type`` spelling to its generated type.

    Recordings name a native channel in one of three ways: the canonical
    registered name (``std_msgs/msg/String``), the generated module qualname
    (``dimos_messages.std_msgs.msg.String``), or the retired historical form
    (``dimos.msgs.std_msgs.String.String``). All three resolve here, through the
    message registry only — an arbitrary Python module is never imported — and
    anything unrecognised returns ``None`` so the channel stays raw bytes.
    """
    if "/msg/" in payload_module:
        from dimos.protocol.pubsub.message_types import resolve_message_type

        return resolve_message_type(payload_module)
    parts = payload_module.split(".")
    if len(parts) == 5 and parts[:2] == ["dimos", "messages"] and parts[3] == "msg":
        from dimos.protocol.pubsub.message_types import resolve_message_type

        return resolve_message_type(f"{parts[2]}/msg/{parts[4]}")
    if len(parts) > 1 and "__msg__" in parts[-1]:
        # Generated classes may be usertype-backed, so their recorded qualname is
        # e.g. ``rosbags.usertypes.std_msgs__msg__String``. Accept it only when it
        # is exactly the qualname of the class the registry resolves to.
        package, _, message = parts[-1].partition("__msg__")
        if package and message:
            from dimos.protocol.pubsub.message_types import resolve_message_type

            kind = resolve_message_type(f"{package}/msg/{message}")
            if kind is not None and f"{kind.__module__}.{kind.__qualname__}" == payload_module:
                return kind
        return None
    return legacy_payload_type(payload_module)


def resolve_payload_type(payload_module: str) -> type[Any]:
    if "/msg/" in payload_module:
        from dimos.protocol.pubsub.message_types import resolve_message_type

        resolved = resolve_message_type(payload_module)
        if resolved is None:
            raise ValueError(f"Unregistered payload type {payload_module!r}")
        return resolved
    if payload_module.startswith("dimos.msgs."):
        resolved = legacy_payload_type(payload_module)
        if resolved is None:
            raise ValueError(f"Unknown historical payload type {payload_module!r}")
        return resolved
    parts = payload_module.rsplit(".", 1)
    if len(parts) != 2:
        raise ValueError(f"Cannot resolve payload type from {payload_module!r}")
    mod = importlib.import_module(parts[0])
    return getattr(mod, parts[1])  # type: ignore[no-any-return]


def _make_one(name: str, payload_module: str, inner: Codec[Any] | None = None) -> Codec[Any]:
    """Instantiate a single codec by its short name."""
    if name == "lz4":
        from dimos.memory.codecs.lz4 import Lz4Codec

        if inner is None:
            raise ValueError("lz4 is a wrapper codec — must have an inner codec")
        return Lz4Codec(inner)
    if name == "cdrjpeg":
        from dimos.memory.codecs.cdrjpeg import CdrjpegCodec

        return CdrjpegCodec()
    if name == "jpeg":
        from dimos.memory.codecs.jpeg import JpegCodec

        return JpegCodec()
    if name == "cdr":
        from dimos.memory.codecs.cdr import CdrCodec

        return CdrCodec(resolve_payload_type(payload_module))
    if name == "lcm":
        from dimos.memory.codecs.lcm import LcmCodec

        return LcmCodec(resolve_payload_type(payload_module))
    if name == "json":
        from dimos.memory.codecs.json import JsonCodec

        expected = JsonCodec.payload_type
        if resolve_payload_type(payload_module) is not expected:
            raise TypeError("JSON codec requires std_msgs.String")
        return JsonCodec()
    if name == "pickle":
        from dimos.memory.codecs.pickle import PickleCodec

        return PickleCodec()
    raise ValueError(f"Unknown codec: {name!r}")
