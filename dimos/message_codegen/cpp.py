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

from contextlib import redirect_stdout
from dataclasses import asdict
from hashlib import sha256
from io import StringIO
import json
from pathlib import Path
from tempfile import TemporaryDirectory

from dimos.message_codegen._vendor.rosidl.rosidl_adapter import convert_to_idl
from dimos.message_codegen._vendor.rosidl.rosidl_pycommon import generate_files
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
    upstream = Path(__file__).with_name("_vendor") / "rosidl"
    for name in [
        "rosidl_runtime_c/message_initialization.h",
        "rosidl_runtime_cpp/message_initialization.hpp",
        "rosidl_runtime_cpp/bounded_vector.hpp",
    ]:
        lines.append(
            "\n".join(
                line
                for line in (upstream / "include" / name).read_text().splitlines()
                if not line.startswith("#include <rosidl_runtime")
            )
        )
    with TemporaryDirectory(prefix="dimos-rosidl-") as temporary:
        output = Path(temporary)
        for message in messages:
            with redirect_stdout(StringIO()):
                idl = convert_to_idl(
                    message.source.parent.parent.resolve(),
                    message.package,
                    Path("msg") / message.source.name,
                    output / "idl" / message.package,
                )
                arguments = output / "arguments.json"
                arguments.write_text(
                    json.dumps(
                        {
                            "package_name": message.package,
                            "idl_tuples": [str(idl.parent.parent) + ":msg/" + idl.name],
                            "output_dir": str(output / "cpp" / message.package),
                            "template_dir": str(upstream / "rosidl_generator_cpp" / "resource"),
                            "target_dependencies": [str(message.source.resolve())],
                        }
                    )
                )
                generate_files(str(arguments), {"idl__struct.hpp.em": "detail/%s__struct.hpp"})
            headers = (output / "cpp" / message.package / "msg" / "detail").glob("*__struct.hpp")
            declaration = next(
                path.read_text()
                for path in headers
                if f"struct {message.short_name}_\n" in path.read_text()
            )
            declaration = "\n".join(
                line for line in declaration.splitlines() if not line.startswith('#include "')
            )
            # Identical dependency declarations share an include guard; conflicting
            # definitions must not be hidden by an upstream name-only guard.
            original_guard = next(
                line.split()[1] for line in declaration.splitlines() if line.startswith("#ifndef ")
            )
            identity = [
                message.name,
                [asdict(field) for field in message.fields],
                [asdict(constant) for constant in message.constants],
            ]
            guard = (
                "DIMOS_CDR_"
                + sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest().upper()
            )
            declaration = declaration.replace(original_guard, guard)
            marker = f"  using Type = {message.short_name}_<ContainerAllocator>;"
            adapter = "\nvoid validate() const {\n" + "\n".join(validation(message)) + "\n}\n"
            adapter += f'static constexpr const char* msg_name = "{message.name}";'
            lines.append(declaration.replace(marker, marker + adapter))

    lines.append("namespace eprosima::fastcdr {")
    bounded = set()
    for message in messages:
        for field in message.fields:
            if not field.type.array_bounded or field.type in bounded:
                continue
            bounded.add(field.type)
            alias = qualified(message.name) + "::_" + field.name + "_type"
            vector = type_name(field.type)
            guard = (
                "DIMOS_CDR_BOUNDED_"
                + sha256(json.dumps(asdict(field.type), sort_keys=True).encode())
                .hexdigest()
                .upper()
            )
            lines.extend([f"#ifndef {guard}", f"#define {guard}"])
            lines.extend(
                [
                    f"template<> inline size_t calculate_serialized_size(CdrSizeCalculator& c, const {alias}& v, size_t& a) {{ return c.calculate_serialized_size({vector}(v.begin(), v.end()), a); }}",
                    f"template<> inline void serialize(Cdr& c, const {alias}& v) {{ c << {vector}(v.begin(), v.end()); }}",
                    f"template<> inline void deserialize(Cdr& c, {alias}& v) {{ {vector} values; c >> values; v.assign(values.begin(), values.end()); }}",
                    "#endif",
                ]
            )
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
        lines.extend(["value.validate();", "}", "#endif"])
    lines.append("}")
    return "\n".join(lines) + "\n"
