# Copyright 2025 Dimensional Inc.
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

import hashlib
import json
import os
import string
from typing import Any
import uuid


def truncate_display_string(arg: Any, max: int | None = None) -> str:
    """
    If we print strings that are too long that potentially obscures more important logs.

    Use this function to truncate it to a reasonable length (configurable from the env).
    """
    string = str(arg)

    if max is not None:
        max_chars = max
    else:
        max_chars = int(os.getenv("TRUNCATE_MAX", "2000"))

    if max_chars == 0 or len(string) <= max_chars:
        return string

    return string[:max_chars] + "...(truncated)..."


def extract_json_from_llm_response(response: str) -> Any:
    start_idx = response.find("{")
    end_idx = response.rfind("}") + 1

    if start_idx >= 0 and end_idx > start_idx:
        json_str = response[start_idx:end_idx]
        try:
            return json.loads(json_str)
        except Exception:
            pass

    return None


def short_id(from_string: str | None = None) -> str:
    alphabet = string.digits + string.ascii_letters
    base = len(alphabet)

    if from_string is None:
        num = uuid.uuid4().int
    else:
        hash_bytes = hashlib.sha1(from_string.encode()).digest()[:16]
        num = int.from_bytes(hash_bytes, "big")

    min_chars = 18

    chars: list[str] = []
    while num > 0 or len(chars) < min_chars:
        num, rem = divmod(num, base)
        chars.append(alphabet[rem])

    return "".join(reversed(chars))[:min_chars]


class classproperty(property):
    def __get__(self, obj, cls):  # type: ignore[no-untyped-def, override]
        return self.fget(cls)  # type: ignore[misc]
