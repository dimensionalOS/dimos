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

"""Resolve CDR recording metadata without generating or duplicating message types."""

from __future__ import annotations

from typing import Any

from dimos_cdrgen.definitions import Definitions, FieldType, Message
from dimos_cdrgen.registry import message_types, schema, schema_roots

SCALAR_SIZES = {
    "bool": 1,
    "byte": 1,
    "char": 1,
    "int8": 1,
    "uint8": 1,
    "int16": 2,
    "uint16": 2,
    "int32": 4,
    "uint32": 4,
    "int64": 8,
    "uint64": 8,
    "float32": 4,
    "float64": 8,
}


def cdr_name(message_type: type[Any]) -> str | None:
    return next((name for name, cls in message_types().items() if cls is message_type), None)


def timestamp_layout(definitions: Definitions, name: str) -> dict[str, Any] | None:
    """Project the conventional header.stamp using the public parsed definitions.

    Only fields preceding the stamp are described. Rust uses re_cdr's runtime
    reader to traverse them; recording keeps the original entire payload.
    """
    messages = {message.name: message for message in definitions.resolve([name])}

    def skip(field: FieldType) -> dict[str, Any]:
        element: dict[str, Any]
        if field.nested:
            element = {
                "kind": "struct",
                "fields": [skip(f.type) for f in messages[field.name].fields],
            }
        elif field.name == "string":
            element = {"kind": "string"}
        elif field.name in SCALAR_SIZES:
            size = SCALAR_SIZES[field.name]
            element = {"kind": "atom", "alignment": size, "bytes": size}
        else:
            raise ValueError(f"Cannot project a CDR timestamp through {field.name}")
        if field.sequence:
            return {"kind": "sequence", "element": element}
        if field.is_array:
            return {"kind": "array", "length": field.array_size, "element": element}
        return element

    def prefix(message: Message, path: tuple[str, ...]) -> dict[str, Any] | None:
        target = next((field for field in message.fields if field.name == path[0]), None)
        if target is None or not target.type.nested or target.type.is_array:
            return None
        nested: dict[str, Any] | None
        if len(path) == 1:
            if target.type.name != "builtin_interfaces/msg/Time":
                return None
            nested = {"kind": "stamp"}
        else:
            nested = prefix(messages[target.type.name], path[1:])
            if nested is None:
                return None
        fields = []
        for field in message.fields:
            if field.name == path[0]:
                fields.append(nested)
                return {"kind": "struct", "fields": fields}
            fields.append(skip(field.type))
        raise AssertionError("Resolved target disappeared")

    header = prefix(messages[name], ("header", "stamp"))
    if header is not None:
        return header
    scalar_names = {
        "bool": "bool",
        "byte": "u8",
        "char": "u8",
        "int8": "i8",
        "uint8": "u8",
        "int16": "i16",
        "uint16": "u16",
        "int32": "i32",
        "uint32": "u32",
        "int64": "i64",
        "uint64": "u64",
        "float32": "f32",
        "float64": "f64",
    }
    fields = messages[name].fields
    target = next((field for field in fields if field.name == "ts"), None)
    if target is None or target.type.is_array or target.type.name not in scalar_names:
        return None
    preceding = []
    for field in fields:
        if field.name == "ts":
            preceding.append({"kind": "seconds", "scalar": scalar_names[field.type.name]})
            return {"kind": "struct", "fields": preceding}
        preceding.append(skip(field.type))
    raise AssertionError("Resolved timestamp field disappeared")


def recording_schema(message_type: type[Any]) -> dict[str, Any]:
    name = cdr_name(message_type)
    if name is None:
        raise TypeError("CDR recording requires an installed, registry-owned message type")
    definitions = Definitions(schema_roots())
    return {
        "name": name,
        "data": schema(name),
        "timestamp_prefix": timestamp_layout(definitions, name),
    }
