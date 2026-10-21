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

"""Message codecs selected once from the declared message type."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Generic, Literal, Protocol, TypeVar, cast

from dimos_cdrgen.registry import decode, encode, message_types

from dimos.msgs.protocol import DimosMsg

T = TypeVar("T")


class MessageCodec(Protocol[T]):
    def encode(self, message: T) -> bytes: ...

    def decode(self, payload: bytes) -> T: ...


@dataclass(frozen=True)
class LCMCodec(Generic[T]):
    message_type: type[T]

    def encode(self, message: T) -> bytes:
        if not isinstance(message, self.message_type):
            raise TypeError("Message does not match the declared type")
        return cast("DimosMsg", message).lcm_encode()

    def decode(self, payload: bytes) -> T:
        return cast("T", cast("type[DimosMsg]", self.message_type).lcm_decode(payload))


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
class MessageDescriptor(Generic[T]):
    message_type: type[T]
    encoding: Literal["lcm", "cdr"]
    name: str
    codec: MessageCodec[T]


def describe_message(message_type: type[T]) -> MessageDescriptor[T]:
    legacy = all(
        callable(getattr(message_type, method, None)) for method in ("lcm_encode", "lcm_decode")
    )
    if legacy and not hasattr(message_type, "__msgtype__"):
        name = getattr(message_type, "msg_name", None)
        if not isinstance(name, str):
            raise TypeError("Legacy message type must declare msg_name")
        return MessageDescriptor(message_type, "lcm", name, LCMCodec(message_type))
    if not hasattr(message_type, "__msgtype__"):
        raise TypeError(f"Unsupported message type: {message_type!r}")
    names = [name for name, cls in message_types().items() if cls is message_type]
    if (names and legacy) or len(names) > 1:
        raise TypeError("Ambiguous message encoding")
    if names:
        return MessageDescriptor(message_type, "cdr", names[0], CDRCodec(message_type))
    if legacy and isinstance(getattr(message_type, "msg_name", None), str):
        return MessageDescriptor(
            message_type, "lcm", cast("Any", message_type).msg_name, LCMCodec(message_type)
        )
    raise TypeError(f"Unsupported message type: {message_type!r}")
