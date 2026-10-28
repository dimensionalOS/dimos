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
from typing import Generic, Literal, Protocol, TypeVar

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
class MessageDescriptor(Generic[T]):
    message_type: type[T]
    encoding: Literal["lcm", "cdr"]
    name: str
    codec: MessageCodec[T]


def describe_message(message_type: type[T]) -> MessageDescriptor[T]:
    names = [name for name, cls in message_types().items() if cls is message_type]
    if len(names) != 1:
        raise TypeError(f"Unsupported or ambiguous generated message type: {message_type!r}")
    return MessageDescriptor(message_type, "cdr", names[0], CDRCodec(message_type))
