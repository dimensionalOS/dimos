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

"""Resolve schema metadata once when registering a raw CDR recording stream."""

from typing import Any

from dimos_cdrgen.registry import message_types, schema


def cdr_name(message_type: type[Any]) -> str | None:
    if not hasattr(message_type, "__msgtype__"):
        return None
    return next((name for name, cls in message_types().items() if cls is message_type), None)


def recording_schema(message_type: type[Any]) -> dict[str, Any]:
    name = cdr_name(message_type)
    if name is None:
        raise TypeError("CDR recording requires an installed, registry-owned message type")
    return {"name": name, "data": schema(name)}
