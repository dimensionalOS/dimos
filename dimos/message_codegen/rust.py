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

"""Generate native Rust types with Serde-backed CDR codecs."""

from __future__ import annotations

from collections import defaultdict
import json
import math
import re
from typing import Any

from dimos.message_codegen.definitions import Definitions, FieldType, Message

PRIMITIVES = {
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
    "string": "::std::string::String",
}
KEYWORDS = frozenset(
    [
        "as",
        "async",
        "await",
        "break",
        "const",
        "continue",
        "crate",
        "dyn",
        "else",
        "enum",
        "extern",
        "false",
        "fn",
        "for",
        "if",
        "impl",
        "in",
        "let",
        "loop",
        "match",
        "mod",
        "move",
        "mut",
        "pub",
        "ref",
        "return",
        "self",
        "Self",
        "static",
        "struct",
        "super",
        "trait",
        "true",
        "type",
        "unsafe",
        "use",
        "where",
        "while",
        "abstract",
        "become",
        "box",
        "do",
        "final",
        "gen",
        "macro",
        "override",
        "priv",
        "try",
        "typeof",
        "unsized",
        "virtual",
        "yield",
    ]
)


def identifier(name: str) -> str:
    return name + "_" if name in KEYWORDS else name


def type_name(type_: FieldType) -> str:
    if type_.nested:
        scalar = "crate::" + "::".join(identifier(part) for part in type_.name.split("/"))
    else:
        scalar = PRIMITIVES[type_.name]
    if type_.sequence:
        return f"::std::vec::Vec<{scalar}>"
    if type_.is_array:
        return f"[{scalar}; {type_.array_size}]"
    return scalar


def string_literal(value: str) -> str:
    return re.sub(r"\\u([0-9a-fA-F]{4})", r"\\u{\1}", json.dumps(value, ensure_ascii=False))


def literal(value: Any, type_: FieldType) -> str:
    if isinstance(value, tuple):
        values = ", ".join(literal(item, FieldType(type_.name)) for item in value)
        return ("vec!" if type_.sequence else "") + f"[{values}]"
    if isinstance(value, str):
        return string_literal(value) + ".to_owned()"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if math.isnan(value):
            return PRIMITIVES[type_.name] + "::NAN"
        if math.isinf(value):
            return PRIMITIVES[type_.name] + ("::NEG_INFINITY" if value < 0 else "::INFINITY")
        return repr(value)
    return str(value)


def generate(messages: tuple[Message, ...], definitions: Definitions) -> str:
    packages: dict[str, list[Message]] = defaultdict(list)
    for message in messages:
        packages[message.package].append(message)
    lines = ["// Generated from ROS2 .msg definitions. Do not edit.", "pub mod codec;"]
    for package, contents in sorted(packages.items()):
        lines.append(f"pub mod {identifier(package)} {{ pub mod msg {{")
        for message in contents:
            name = identifier(message.short_name)
            lines.extend(
                [
                    "#[derive(Debug, Clone, PartialEq, ::serde::Serialize, ::serde::Deserialize)]",
                    f"pub struct {name} {{",
                ]
            )
            for field in message.fields:
                if field.type.is_array and not field.type.sequence:
                    lines.append('#[serde(with = "serde_big_array::BigArray")]')
                if identifier(field.name) != field.name:
                    lines.append(f'#[serde(rename = "{field.name}")]')
                lines.append(f"pub {identifier(field.name)}: {type_name(field.type)},")
            if not message.fields:
                lines.extend(["#[serde(default)]", "_unused: u8,"])
            lines.extend(
                [
                    "}",
                    "// Explicit defaults also support .msg defaults and arrays longer than 32.",
                    "#[allow(clippy::derivable_impls)]",
                    f"impl ::std::default::Default for {name} {{ fn default() -> Self {{ Self {{",
                ]
            )
            for field in message.fields:
                if field.default is not None:
                    default = literal(field.default, field.type)
                elif field.type.is_array and not field.type.sequence:
                    default = "::std::array::from_fn(|_| ::std::default::Default::default())"
                else:
                    default = "::std::default::Default::default()"
                lines.append(f"{identifier(field.name)}: {default},")
            if not message.fields:
                lines.append("_unused: 0,")
            lines.extend(
                [
                    "} } }",
                    f"impl crate::codec::Message for {name} {{",
                    f'const NAME: &\'static str = "{message.name}";',
                    f"const SCHEMA: &'static str = {string_literal(definitions.schema(message.name))};",
                    "fn validate(&self) -> ::std::result::Result<(), ::std::string::String> {",
                ]
            )
            for field in message.fields:
                member = "self." + identifier(field.name)
                if field.type.array_bounded:
                    lines.append(
                        f'if {member}.len() > {field.type.array_size} {{ return Err("{field.name} exceeds sequence bound".into()); }}'
                    )
                checks = []
                value = "item" if field.type.is_array else member
                if field.type.string_bound is not None:
                    checks.append(
                        f'if {value}.len() > {field.type.string_bound} {{ return Err("{field.name} exceeds string bound".into()); }}'
                    )
                if field.type.nested:
                    checks.append(f"crate::codec::Message::validate(&{value})?;")
                if checks and field.type.is_array:
                    lines.append(
                        f"for item in &{member} {{ {' '.join(checks).replace('&item', 'item')} }}"
                    )
                else:
                    lines.extend(checks)
            lines.extend(["Ok(())", "}", "}"])
            if message.constants:
                lines.append(f"impl {name} {{")
                for constant in message.constants:
                    native = (
                        "&'static str" if constant.type == "string" else PRIMITIVES[constant.type]
                    )
                    value = (
                        string_literal(constant.value)
                        if constant.type == "string"
                        else literal(constant.value, FieldType(constant.type))
                    )
                    lines.append(f"pub const {identifier(constant.name)}: {native} = {value};")
                lines.append("}")
        lines.append("} }")
    return "\n".join(lines) + "\n"
