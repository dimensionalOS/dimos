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
    """Round-trip historical LCM envelopes, or lift them into the generated family.

    A legacy ``dimos.msgs`` target keeps the class that wrote the artifact, so
    historical recordings decode (and re-encode) exactly as they did before CDR:
    the recorded values are retained. A generated target is the canonical family
    a migrated consumer reads: the same envelope decodes into it, read-only —
    new recordings use ``CdrCodec``.
    """

    def __init__(self, msg_type: type) -> None:
        self._msg_type = msg_type

    @property
    def payload_type(self) -> type:
        return self._msg_type

    def _generated(self) -> bool:
        return getattr(self._msg_type, "__msgtype__", None) is not None

    def encode(self, value: Any) -> bytes:
        if self._generated():
            raise TypeError("Historical LCM codecs are read-only; use CDR for new recordings")
        return value.lcm_encode()  # type: ignore[no-any-return]

    def decode(self, data: bytes) -> Any:
        if self._generated():
            return decode_lcm(data, self._msg_type)
        return self._msg_type.lcm_decode(data)  # type: ignore[attr-defined,no-any-return]
