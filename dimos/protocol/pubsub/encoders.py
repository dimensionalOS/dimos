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

from collections.abc import Callable
import pickle
from typing import Any, TypeAlias

Encoder: TypeAlias = Callable[[Any, Any], bytes]
Decoder: TypeAlias = Callable[[bytes, Any], Any]
Codec: TypeAlias = tuple[Encoder, Decoder]


class DecodingError(Exception):
    pass


def raw(message: bytes, _: Any) -> bytes:
    return message


def pickle_encode(message: Any, _: Any) -> bytes:
    return pickle.dumps(message)


def pickle_decode(message: bytes, _: Any) -> Any:
    return pickle.loads(message)


def lcm_encode(message: Any, _: Any) -> bytes:
    return message if isinstance(message, bytes) else message.lcm_encode()


def lcm_decode(message: bytes, topic: Any) -> Any:
    if topic.lcm_type is None:
        raise DecodingError
    return topic.lcm_type.lcm_decode(message)


RAW: Codec = (raw, raw)
PICKLE: Codec = (pickle_encode, pickle_decode)
LCM: Codec = (lcm_encode, lcm_decode)
