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

from typing import Protocol, TypeVar, runtime_checkable

from dimos_cdrgen.registry import decode as _registry_decode

_Message = TypeVar("_Message")


def validate_cdr_payload(data: bytes) -> None:
    """Require a standalone plain XCDR1 representation in either byte order."""
    if len(data) < 4 or data[:2] not in (b"\x00\x00", b"\x00\x01"):
        raise ValueError("Expected standalone XCDR1 encapsulation")


def decode_cdr(data: bytes, message_type: type[_Message]) -> _Message:
    """Validate encapsulation, then leave field decoding to the installed codec.

    The installed serializer treats the endian marker permissively; arbitrary
    leading bytes must not become valid values at runtime boundaries.
    """
    validate_cdr_payload(data)
    return _registry_decode(data, message_type)


@runtime_checkable
class DimosMsg(Protocol):
    """Protocol for dimos message types (LCM-based messages from dimos.msgs)."""

    msg_name: str

    @classmethod
    def lcm_decode(cls, data: bytes) -> "DimosMsg":
        """Decode bytes into a message instance."""
        ...

    def lcm_encode(self) -> bytes:
        """Encode this message instance into bytes."""
        ...
