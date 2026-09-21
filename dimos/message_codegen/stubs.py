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

"""Emit static Python interfaces from the same definitions as the native bindings."""

from collections import defaultdict

from dimos.message_codegen import cpp
from dimos.message_codegen.definitions import FieldType, Message

NUMPY_TYPES = {
    "byte": "uint8",
    "char": "uint8",
    "int8": "int8",
    "uint8": "uint8",
    "int16": "int16",
    "uint16": "uint16",
    "int32": "int32",
    "uint32": "uint32",
    "int64": "int64",
    "uint64": "uint64",
    "float32": "float32",
    "float64": "float64",
}


def scalar_type(name: str, module: str) -> str:
    if "/" in name:
        return f"{module}.{name.replace('/', '.')}"
    if name in ("string", "wstring"):
        return "str"
    if name == "bool":
        return "bool"
    return "float" if name.startswith("float") else "int"


def input_type(field: FieldType, module: str) -> str:
    scalar = scalar_type(field.name, module)
    if not field.is_array:
        return scalar
    result = f"Iterable[{scalar}]"
    if field.name in NUMPY_TYPES:
        result += " | NDArray[Any]"
    return result


def generate(messages: tuple[Message, ...], module: str) -> dict[str, str]:
    containers = {
        cpp.type_name(field.type): field.type
        for message in messages
        for field in message.fields
        if field.type.is_array
    }
    names = {native: f"_Sequence{index}" for index, native in enumerate(sorted(containers))}
    packages: dict[str, list[Message]] = defaultdict(list)
    for message in messages:
        packages[message.package].append(message)
    imports = [
        "from collections.abc import Iterable, Iterator",
        "from typing import Any, ClassVar",
        "import numpy as np",
        "from numpy.typing import NDArray",
        f"import {module}",
        *[f"import {module}.{package}.msg" for package in sorted(packages)],
    ]
    root = imports + [f"from . import {package} as {package}" for package in sorted(packages)]
    for native in sorted(containers):
        field = containers[native]
        scalar = scalar_type(field.name, module)
        root.extend(
            [
                f"class {names[native]}:",
                "    def __len__(self) -> int: ...",
                f"    def __getitem__(self, index: int) -> {scalar}: ...",
                f"    def __setitem__(self, index: int, value: {scalar}) -> None: ...",
                f"    def __iter__(self) -> Iterator[{scalar}]: ...",
            ]
        )
        if field.sequence:
            root.extend(
                [
                    f"    def append(self, value: {scalar}) -> None: ...",
                    f"    def extend(self, values: {input_type(field, module)}) -> None: ...",
                    "    def clear(self) -> None: ...",
                ]
            )
        if field.name in NUMPY_TYPES:
            array = f"NDArray[np.{NUMPY_TYPES[field.name]}]"
            root.extend(
                [
                    f"    def view(self) -> {array}: ...",
                    f"    def copy(self) -> {array}: ...",
                ]
            )
    output = {"__init__.pyi": "\n".join(root) + "\n", "py.typed": ""}
    for package, types in sorted(packages.items()):
        output[f"{package}/__init__.pyi"] = "from . import msg as msg\n"
        lines = imports.copy()
        for message in types:
            lines.extend(
                [
                    f"class {message.short_name}:",
                    "    msg_name: ClassVar[str]",
                    "    schema: ClassVar[str]",
                ]
            )
            for constant in message.constants:
                lines.append(f"    {constant.name}: ClassVar[{scalar_type(constant.type, module)}]")
            arguments = ", ".join(
                f"{field.name}: {input_type(field.type, module)} = ..." for field in message.fields
            )
            args = f", *, {arguments}" if arguments else ""
            lines.append(f"    def __init__(self{args}) -> None: ...")
            for field in message.fields:
                if field.type.is_array:
                    sequence = names[cpp.type_name(field.type)]
                    lines.extend(
                        [
                            "    @property",
                            f"    def {field.name}(self) -> {module}.{sequence}: ...",
                            f"    @{field.name}.setter",
                            f"    def {field.name}(self, value: {input_type(field.type, module)}) -> None: ...",
                        ]
                    )
                else:
                    lines.append(f"    {field.name}: {scalar_type(field.type.name, module)}")
            lines.extend(
                [
                    "    def encode(self, little_endian: bool = True) -> bytes: ...",
                    "    @staticmethod",
                    f"    def decode(data: bytes) -> {message.short_name}: ...",
                ]
            )
        output[f"{package}/msg/__init__.pyi"] = "\n".join(lines) + "\n"
    return output
