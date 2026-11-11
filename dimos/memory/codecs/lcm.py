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

from typing import Any

from dimos.memory.codecs.legacy import decode_lcm


class LcmCodec:
    """Read historical LCM envelopes directly into generated messages."""

    def __init__(self, msg_type: type) -> None:
        self._msg_type = msg_type

    @property
    def payload_type(self) -> type:
        return self._msg_type

    def encode(self, value: Any) -> bytes:
        raise TypeError("Historical LCM codecs are read-only; use CDR for new recordings")

    def decode(self, data: bytes) -> Any:
        return decode_lcm(data, self._msg_type)
