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

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.msgs.protocol import DimosMsg


class LcmCodec:
    """Codec for DimosMsg types — uses lcm_encode/lcm_decode."""

    def __init__(self, msg_type: type[DimosMsg]) -> None:
        self._msg_type = msg_type

    def encode(self, value: DimosMsg) -> bytes:
        return value.lcm_encode()

    def decode(self, data: bytes) -> DimosMsg:
        return self._msg_type.lcm_decode(data)
