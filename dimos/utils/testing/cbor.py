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

"""Round-trip a value through the CBOR codec, for tests that check it crosses processes."""

from typing import Any

from dimos.protocol.encode.cbor import decode, encode


def roundtrip(value: Any) -> Any:
    """Encode ``value`` and decode the bytes, as the RPC layer would."""
    return decode(encode(value))
