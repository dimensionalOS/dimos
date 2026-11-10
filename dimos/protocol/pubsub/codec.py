# Copyright 2025-2026 Dimensional Inc.
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

"""Message codecs selected once from the declared message type.

Generated types registered by ``dimos_cdrgen`` use raw CDR. Legacy
``dimos.msgs`` classes keep their LCM encoding, so both coexist while consumers
migrate opt-in.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Generic, Literal, Protocol, TypeVar

from dimos_cdrgen.registry import decode, encode, message_types

T = TypeVar("T")


class MessageCodec(Protocol[T]):
    def encode(self, message: T) -> bytes: ...

    def decode(self, payload: bytes) -> T: ...


@dataclass(frozen=True)
class CDRCodec(Generic[T]):
    message_type: type[T]

    def encode(self, message: T) -> bytes:
        if type(message) is not self.message_type:
            raise TypeError("Message does not match the declared type")
        return encode(message)

    def decode(self, payload: bytes) -> T:
        if len(payload) < 4 or payload[:2] not in (b"\x00\x00", b"\x00\x01"):
            raise ValueError("Expected standalone XCDR1 encapsulation")
        return decode(payload, self.message_type)


@dataclass(frozen=True)
class LCMCodec(Generic[T]):
    """Legacy ``dimos.msgs`` wire encoding, unchanged by the CDR rollout."""

    message_type: type[T]

    def encode(self, message: T) -> bytes:
        if type(message) is not self.message_type:
            raise TypeError("Message does not match the declared type")
        return message.lcm_encode()  # type: ignore[attr-defined,no-any-return]

    def decode(self, payload: bytes) -> T:
        try:
            return self.message_type.lcm_decode(payload)  # type: ignore[attr-defined,no-any-return]
        except AttributeError as error:
            # Publish-only doubles (e.g. tests) may implement just lcm_encode.
            raise ValueError(f"{self.message_type!r} has no lcm_decode") from error


@dataclass(frozen=True)
class MessageDescriptor(Generic[T]):
    message_type: type[T]
    encoding: Literal["lcm", "cdr"]
    name: str
    codec: MessageCodec[T]


def _generated_name(message_type: type[Any]) -> str | None:
    names = [name for name, cls in message_types().items() if cls is message_type]
    if len(names) > 1:
        raise TypeError(f"Ambiguous generated message type: {message_type!r}")
    return names[0] if names else None


def describe_message(message_type: type[T]) -> MessageDescriptor[T]:
    """Describe *message_type* with the codec its class family defines.

    Generated (installed) types are CDR. Legacy ``dimos.msgs`` classes, which
    still implement ``lcm_encode``/``lcm_decode``, keep LCM so existing modules
    are unaffected until they are migrated.
    """
    name = _generated_name(message_type)
    if name is not None:
        return MessageDescriptor(message_type, "cdr", name, CDRCodec(message_type))
    if hasattr(message_type, "lcm_encode") or hasattr(message_type, "lcm_decode"):
        legacy_name = getattr(message_type, "msg_name", None)
        if not isinstance(legacy_name, str) or not legacy_name:
            legacy_name = f"{message_type.__module__}.{message_type.__name__}"
        return MessageDescriptor(message_type, "lcm", legacy_name, LCMCodec(message_type))
    raise TypeError(f"Unsupported or ambiguous message type: {message_type!r}")


def message_encoding(message_type: type[Any]) -> Literal["lcm", "cdr"] | None:
    """Encoding of *message_type*, or ``None`` while it is not describable.

    Legacy channel helpers use this to keep their historical tolerance for
    classes that are not bound to a codec yet instead of raising at parse time.
    """
    try:
        return describe_message(message_type).encoding
    except TypeError:
        return None
