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

"""Store a String's JSON document as UTF-8, without its transport envelope."""

import json

from dimos_generated.std_msgs.msg import String


def _invalid_constant(value: str) -> None:
    raise ValueError(f"Invalid JSON constant: {value}")


class JsonCodec:
    payload_type = String

    def encode(self, value: String) -> bytes:
        json.loads(value.data, parse_constant=_invalid_constant)
        return value.data.encode("utf-8")

    def decode(self, data: bytes) -> String:
        value = String(data=data.decode("utf-8"))
        json.loads(value.data, parse_constant=_invalid_constant)
        return value
