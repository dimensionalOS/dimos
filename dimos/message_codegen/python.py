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

"""Generate Python bindings to the same native types/codecs used by C++."""

from __future__ import annotations

import json

from dimos.message_codegen import cpp
from dimos.message_codegen.definitions import Definitions, Message


def string_literal(value: str) -> str:
    delimiter = "msg"
    while f'){delimiter}"' in value:
        delimiter += "_"
    return f'R"{delimiter}({value}){delimiter}"'


def generate(
    messages: tuple[Message, ...], definitions: Definitions, module: str
) -> dict[str, str]:
    sources: dict[str, str] = {}
    containers = sorted(
        {
            cpp.type_name(field.type)
            for message in messages
            for field in message.fields
            if field.type.is_array
        }
    )
    container_names = {native: f"_Sequence{index}" for index, native in enumerate(containers)}
    functions = []
    for offset in range(0, len(messages), 10):
        function = f"bind_{offset // 10}"
        functions.append(function)
        lines = [
            "// Generated from ROS2 .msg definitions. Do not edit.",
            "#include <pybind11/pybind11.h>",
            "#include <pybind11/stl.h>",
            '#include "messages.hpp"',
            '#include "dimos_python.hpp"',
            "namespace py = pybind11;",
            f"void {function}(py::module_& root) {{",
        ]
        for message in messages[offset : offset + 10]:
            native = cpp.qualified(message.name)
            lines.extend(
                [
                    "{",
                    f'auto package = py::hasattr(root, "{message.package}") ? root.attr("{message.package}").cast<py::module_>() : root.def_submodule("{message.package}");',
                    'auto module = py::hasattr(package, "msg") ? package.attr("msg").cast<py::module_>() : package.def_submodule("msg");',
                    f'auto cls = py::class_<{native}>(module, "{message.short_name}", py::dynamic_attr(), py::module_local());',
                    f"cls.def(py::init([](py::kwargs kwargs) {{ {native} value{{}};",
                    "for (auto item : kwargs) { auto key = py::cast<std::string>(item.first);",
                ]
            )
            for index, field in enumerate(message.fields):
                condition = "if" if index == 0 else "else if"
                cast = "dimos::python::sequence_from_python" if field.type.is_array else "py::cast"
                lines.append(
                    f'{condition} (key == "{field.name}") value.{cpp.identifier(field.name)} = {cast}<{cpp.type_name(field.type)}>(item.second);'
                )
            fallback = "else " if message.fields else ""
            lines.append(f'{fallback}throw py::type_error("Unknown message field: " + key);')
            lines.extend(["}", "value.validate(); return value; }));"])
            for field in message.fields:
                field_native = cpp.type_name(field.type)
                member = f"self.cast<{native}&>().{cpp.identifier(field.name)}"
                if field.type.is_array:
                    lines.extend(
                        [
                            f'dimos::python::bind_sequence<{field_native}>(root, "{container_names[field_native]}");',
                            f'cls.def_property("{field.name}", [](py::object self) {{ return dimos::python::Sequence<{field_native}>{{ &{member}, dimos::python::owner_root(self) }}; }},',
                            f"[](py::object self, py::handle input) {{ dimos::python::require_unborrowed(self); {member} = dimos::python::sequence_from_python<{field_native}>(input); }});",
                        ]
                    )
                elif field.type.nested:
                    lines.extend(
                        [
                            f'cls.def_property("{field.name}", [](py::object self) {{ auto child = py::cast(&{member}, py::return_value_policy::reference_internal, self); child.attr("__dimos_owner") = dimos::python::owner_root(self); return child; }},',
                            f"[](py::object self, const {field_native}& input) {{ dimos::python::require_unborrowed(self); {member} = input; }});",
                        ]
                    )
                else:
                    lines.append(
                        f'cls.def_readwrite("{field.name}", &{native}::{cpp.identifier(field.name)});'
                    )
            for constant in message.constants:
                lines.append(
                    f'cls.attr("{constant.name}") = py::cast({native}::{cpp.identifier(constant.name)});'
                )
            lines.extend(
                [
                    f'cls.def("encode", [](const {native}& value, bool little_endian) {{ auto bytes = dimos::cdr::encode(value, little_endian); return py::bytes(reinterpret_cast<const char*>(bytes.data()), bytes.size()); }}, py::arg("little_endian") = true);',
                    f'cls.def_static("decode", [](py::bytes input) {{ std::string bytes = input; return dimos::cdr::decode<{native}>(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size()); }});',
                    f"cls.def(py::pickle([](const {native}& value) {{ auto bytes = dimos::cdr::encode(value); return py::bytes(reinterpret_cast<const char*>(bytes.data()), bytes.size()); }}, [](py::bytes input) {{ std::string bytes = input; return dimos::cdr::decode<{native}>(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size()); }}));",
                    f'cls.attr("msg_name") = {json.dumps(message.name)};',
                    f'cls.attr("schema") = {string_literal(definitions.schema(message.name))};',
                    "}",
                ]
            )
        lines.append("}")
        sources[function + ".cpp"] = "\n".join(lines) + "\n"
    sources["bindings.cpp"] = (
        "\n".join(
            [
                "// Generated from ROS2 .msg definitions. Do not edit.",
                "#include <pybind11/pybind11.h>",
                *(f"void {function}(pybind11::module_&);" for function in functions),
                f"PYBIND11_MODULE({module}, root) {{",
                *(f"{function}(root);" for function in functions),
                "}",
            ]
        )
        + "\n"
    )
    return sources
