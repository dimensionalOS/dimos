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

"""Generate native C++ value types and Fast CDR customization functions."""

from __future__ import annotations

from dataclasses import asdict
from hashlib import sha256
import json
import math
from typing import Any

from dimos.message_codegen.definitions import FieldType, Message

PRIMITIVES = {
    "bool": "bool",
    "byte": "uint8_t",
    "char": "uint8_t",
    "int8": "int8_t",
    "uint8": "uint8_t",
    "int16": "int16_t",
    "uint16": "uint16_t",
    "int32": "int32_t",
    "uint32": "uint32_t",
    "int64": "int64_t",
    "uint64": "uint64_t",
    "float32": "float",
    "float64": "double",
    "string": "std::string",
    "wstring": "std::wstring",
}
KEYWORDS = frozenset(
    [
        "alignas",
        "alignof",
        "and",
        "and_eq",
        "asm",
        "auto",
        "bitand",
        "bitor",
        "bool",
        "break",
        "case",
        "catch",
        "char",
        "char8_t",
        "char16_t",
        "char32_t",
        "class",
        "compl",
        "concept",
        "const",
        "consteval",
        "constexpr",
        "constinit",
        "const_cast",
        "continue",
        "co_await",
        "co_return",
        "co_yield",
        "decltype",
        "default",
        "delete",
        "do",
        "double",
        "dynamic_cast",
        "else",
        "enum",
        "explicit",
        "export",
        "extern",
        "false",
        "float",
        "for",
        "friend",
        "goto",
        "if",
        "inline",
        "int",
        "long",
        "mutable",
        "namespace",
        "new",
        "noexcept",
        "not",
        "not_eq",
        "nullptr",
        "operator",
        "or",
        "or_eq",
        "private",
        "protected",
        "public",
        "register",
        "reinterpret_cast",
        "requires",
        "return",
        "short",
        "signed",
        "sizeof",
        "static",
        "static_assert",
        "static_cast",
        "struct",
        "switch",
        "template",
        "this",
        "thread_local",
        "throw",
        "true",
        "try",
        "typedef",
        "typeid",
        "typename",
        "union",
        "unsigned",
        "using",
        "virtual",
        "void",
        "volatile",
        "wchar_t",
        "while",
        "xor",
        "xor_eq",
    ]
)


def identifier(name: str) -> str:
    return name + "_" if name in KEYWORDS else name


def qualified(name: str) -> str:
    return "::".join(identifier(part) for part in name.split("/"))


def type_name(type_: FieldType) -> str:
    scalar = qualified(type_.name) if type_.nested else PRIMITIVES[type_.name]
    if type_.sequence:
        return f"std::vector<{scalar}>"
    if type_.is_array:
        return f"std::array<{scalar}, {type_.array_size}>"
    return scalar


def literal(value: Any, type_name_: str) -> str:
    if isinstance(value, tuple):
        return "{" + ", ".join(literal(item, type_name_) for item in value) + "}"
    if isinstance(value, str):
        return ("L" if type_name_ == "wstring" else "") + json.dumps(value, ensure_ascii=True)
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        native = PRIMITIVES[type_name_]
        if math.isnan(value):
            return f"std::numeric_limits<{native}>::quiet_NaN()"
        if math.isinf(value):
            sign = "-" if value < 0 else ""
            return f"{sign}std::numeric_limits<{native}>::infinity()"
        return repr(value) + ("f" if type_name_ == "float32" else "")
    if type_name_ == "uint64":
        return f"{value}ULL"
    if type_name_ == "int64" and value == -(2**63):
        return "(-9223372036854775807LL - 1)"
    return str(value)


def validation(message: Message) -> list[str]:
    lines = []
    for field in message.fields:
        member = identifier(field.name)
        type_ = field.type
        if type_.array_bounded:
            lines.append(
                f'if ({member}.size() > {type_.array_size}) throw std::length_error("{field.name} exceeds sequence bound");'
            )
        value = "item" if type_.is_array else member
        checks = []
        if type_.string_bound is not None:
            checks.append(
                f'if ({value}.size() > {type_.string_bound}) throw std::length_error("{field.name} exceeds string bound");'
            )
        if type_.nested:
            checks.append(f"{value}.validate();")
        if checks and type_.is_array:
            lines.append(f"for (const auto& item : {member}) {{ {' '.join(checks)} }}")
        else:
            lines.extend(checks)
    return lines


def generate(messages: tuple[Message, ...]) -> str:
    guards = {
        message.name: "DIMOS_MESSAGE_"
        + sha256(
            json.dumps(
                [
                    message.name,
                    [asdict(field) for field in message.fields],
                    [asdict(constant) for constant in message.constants],
                ],
                sort_keys=True,
            ).encode()
        )
        .hexdigest()
        .upper()
        for message in messages
    }
    lines = [
        "// Generated from ROS2 .msg definitions. Do not edit.",
        "#pragma once",
        "#include <array>",
        "#include <cstdint>",
        "#include <limits>",
        "#include <stdexcept>",
        "#include <string>",
        "#include <vector>",
        "#include <fastcdr/Cdr.h>",
        "#include <fastcdr/CdrSizeCalculator.hpp>",
        '#include "dimos_cdr.hpp"',
    ]
    for message in messages:
        guard = guards[message.name] + "_TYPE"
        lines.extend([f"#ifndef {guard}", f"#define {guard}"])
        namespace = qualified(message.name.rsplit("/", 1)[0])
        name = identifier(message.short_name)
        lines.extend([f"namespace {namespace} {{", f"struct {name} {{"])
        for constant in message.constants:
            native = PRIMITIVES[constant.type]
            if constant.type in {"string", "wstring"}:
                native = "const wchar_t*" if constant.type == "wstring" else "const char*"
            lines.append(
                f"static constexpr {native} {identifier(constant.name)} = {literal(constant.value, constant.type)};"
            )
        for field in message.fields:
            if field.default is None:
                default = "{}"
            elif isinstance(field.default, tuple):
                default = literal(field.default, field.type.name)
            else:
                default = "{" + literal(field.default, field.type.name) + "}"
            lines.append(f"{type_name(field.type)} {identifier(field.name)}{default};")
        equal = (
            " && ".join(
                f"this->{identifier(field.name)} == other.{identifier(field.name)}"
                for field in message.fields
            )
            or "true"
        )
        other = " other" if message.fields else ""
        lines.append(f"bool operator==(const {name}&{other}) const {{ return {equal}; }}")
        lines.append(f"bool operator!=(const {name}& other) const {{ return !(*this == other); }}")
        lines.append("void validate() const {")
        lines.extend(validation(message))
        lines.extend(["}", f'static constexpr const char* msg_name = "{message.name}";', "};", "}"])
        lines.append("#endif")

    lines.append("namespace eprosima::fastcdr {")
    for message in messages:
        guard = guards[message.name] + "_CODEC"
        lines.extend([f"#ifndef {guard}", f"#define {guard}"])
        name = qualified(message.name)
        lines.extend(
            [
                f"template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const {name}& value, size_t& alignment) {{",
                "size_t size = 0;",
            ]
        )
        for field in message.fields:
            lines.append(
                f"size += calculator.calculate_serialized_size(value.{identifier(field.name)}, alignment);"
            )
        if not message.fields:
            lines.append("size += calculator.calculate_serialized_size(uint8_t{0}, alignment);")
        lines.extend(
            [
                "return size;",
                "}",
                f"template<> inline void serialize(Cdr& cdr, const {name}& value) {{",
            ]
        )
        for field in message.fields:
            lines.append(f"cdr << value.{identifier(field.name)};")
        if not message.fields:
            lines.append("cdr << uint8_t{0};")
        lines.extend(["}", f"template<> inline void deserialize(Cdr& cdr, {name}& value) {{"])
        for field in message.fields:
            lines.append(f"cdr >> value.{identifier(field.name)};")
        if not message.fields:
            lines.append("uint8_t unused; cdr >> unused;")
        lines.extend(["value.validate();", "}"])
        lines.append("#endif")
    lines.append("}")
    return "\n".join(lines) + "\n"
