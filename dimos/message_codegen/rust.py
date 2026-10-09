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

"""Generate defaults/validation adapters; Cargo delegates Rust types to ros2msg."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
import shutil
from typing import Any

from .definitions import Definitions, FieldType, Message


def string_literal(value: str) -> str:
    return re.sub(
        r"\\u([0-9a-fA-F]{4})", r"\\u{\1}", json.dumps(value, ensure_ascii=False)
    ).replace("@", r"\u{40}")


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
            return ("f32" if type_.name == "float32" else "f64") + "::NAN"
        if math.isinf(value):
            return ("f32" if type_.name == "float32" else "f64") + (
                "::NEG_INFINITY" if value < 0 else "::INFINITY"
            )
        return repr(value)
    return str(value)


def write_crate(
    crate: Path,
    messages: tuple[Message, ...],
    definitions: Definitions,
    imports: dict[str, str] | None = None,
) -> None:
    imports = imports or {}
    if (crate / "interfaces").exists():
        shutil.rmtree(crate / "interfaces")
    adapters = []
    for message in messages:
        name = message.short_name
        lines = [
            f"#[allow(clippy::derivable_impls)] impl Default for {name} {{ fn default() -> Self {{ Self {{"
        ]
        for field in message.fields:
            if field.default is not None:
                default = literal(field.default, field.type)
            elif field.type.is_array and not field.type.sequence:
                default = "::std::array::from_fn(|_| ::std::default::Default::default())"
            else:
                default = "::std::default::Default::default()"
            lines.append(f"@{field.name}@: {default},")
        if not message.fields:
            lines.append("structure_needs_at_least_one_member: 0,")
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
            member = "self.@" + field.name + "@"
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
                field_owner = imports.get(field.type.name)
                trait = f"{field_owner}::codec::Message" if field_owner else "crate::codec::Message"
                checks.append(f"{trait}::validate(&{value})?;")
            if checks and field.type.is_array:
                lines.append(
                    f"for item in &{member} {{ {' '.join(checks).replace('&item', 'item')} }}"
                )
            else:
                lines.extend(checks)
        lines.extend(["Ok(())", "}", "}"])
        adapters.append(
            "("
            + ", ".join(
                [
                    string_literal(message.name),
                    string_literal("\n".join(lines)),
                    "&[" + ", ".join(string_literal(field.name) for field in message.fields) + "]",
                ]
            )
            + ")"
        )
        source = crate / "interfaces" / (message.name + ".msg")
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text(message.text)
    build = Path(__file__).with_name("templates").joinpath("message_build.rs").read_text()
    build += "\nconst ADAPTERS: &[(&str, &str, &[&str])] = &[" + ",\n".join(adapters) + "];\n"
    build += (
        "const IMPORTS: &[(&str, &str)] = &["
        + ",".join(
            f"({string_literal(name)}, {string_literal(owner)})"
            for name, owner in sorted(imports.items())
        )
        + "];\n"
    )
    (crate / "build.rs").write_text(build)
    (crate / "src" / "lib.rs").write_text(
        'pub mod codec;\ninclude!(concat!(env!("OUT_DIR"), "/messages.rs"));\n'
    )
