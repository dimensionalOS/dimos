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

"""CDR storage for installed generated values; independent of legacy codecs."""

from __future__ import annotations

from typing import Generic, TypeVar

from dimos.protocol.pubsub.codec import describe_message

T = TypeVar("T")


class CdrCodec(Generic[T]):
    def __init__(self, msg_type: type[T]) -> None:
        descriptor = describe_message(msg_type)
        if descriptor.encoding != "cdr":
            raise TypeError(f"{msg_type!r} is not an installed generated CDR message type")
        self._msg_type = msg_type
        self._codec = descriptor.codec

    @property
    def payload_type(self) -> type[T]:
        return self._msg_type

    def encode(self, value: T) -> bytes:
        return self._codec.encode(value)

    def decode(self, data: bytes) -> T:
        return self._codec.decode(data)
