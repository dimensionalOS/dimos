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

from dimos_cdrgen.registry import decode, encode, message_types

T = TypeVar("T")


class CdrCodec(Generic[T]):
    def __init__(self, msg_type: type[T]) -> None:
        name = getattr(msg_type, "__msgtype__", None)
        if not isinstance(name, str) or message_types().get(name) is not msg_type:
            raise TypeError(f"{msg_type!r} is not an installed generated CDR message type")
        self._msg_type = msg_type

    @property
    def payload_type(self) -> type[T]:
        return self._msg_type

    def encode(self, value: T) -> bytes:
        if not isinstance(value, self._msg_type):
            raise TypeError(f"Expected {self._msg_type.__name__}, got {type(value).__name__}")
        return encode(value)

    def decode(self, data: bytes) -> T:
        return decode(data, self._msg_type)
