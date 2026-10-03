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

"""Generate ordinary Python source values using the shared pure-Python CDR runtime."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.message_codegen.definitions import Definitions, Message

ABI = "dimos-cdr-source-rosbags-0.11.0-v1"


def literal(value: Any) -> str:
    if isinstance(value, float) and not (-float("inf") < value < float("inf")):
        return f"float({str(value)!r})"
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(literal(item) for item in value) + "]"
    return repr(value)


def generate(
    messages: tuple[Message, ...],
    definitions: Definitions,
    module: str,
    *,
    imports: tuple[str, ...] = (),
    shared: bool = False,
    version: str = "0.1.0",
    imported: dict[str, str] | None = None,
    dependency_versions: dict[str, str] | None = None,
) -> dict[str, str]:
    """Emit importable packages; dependency messages retain their canonical class identity."""
    imported = imported or {}
    lines = [
        *Path(__file__).read_text().splitlines()[:13],
        "# Generated from ROS2 .msg definitions. Do not edit.",
        "from __future__ import annotations",
        "from ._runtime import Codec, Message, Sequence",
    ]
    lines.extend(f"import {name}" for name in imports)
    for dependency, expected in sorted((dependency_versions or {}).items()):
        lines.extend(
            [
                f"if {dependency}.__dimos_version__ != {expected!r} or {dependency}.__dimos_abi__ != {ABI!r}:",
                f"    raise ImportError('Incompatible message dependency: {dependency}')",
            ]
        )
    for name, owner in sorted(imported.items()):
        symbol = name.replace("/", "__")
        lines.append(f"{symbol} = {owner}.{name.replace('/', '.')}")
        lines.extend(
            [
                f"if {symbol}.schema != {definitions.schema(name)!r}:",
                f"    raise ImportError('Dependency schema mismatch: {name}')",
            ]
        )
    for message in messages:
        symbol = message.name.replace("/", "__")
        lines.extend(
            [
                f"class {symbol}(Message):",
                f"    msg_name = {message.name!r}",
                f"    schema = {definitions.schema(message.name)!r}",
                "    _fields = (",
            ]
        )
        for field in message.fields:
            kind = field.type
            spec = (
                field.name,
                kind.name,
                kind.is_array,
                kind.array_size,
                kind.array_bounded,
                kind.string_bound,
                field.default,
            )
            lines.append("        (" + ", ".join(literal(item) for item in spec) + "),")
        lines.append("    )")
        for field in message.fields:
            kind = field.type
            annotation = (
                "Sequence"
                if kind.is_array
                else kind.name.replace("/", "__")
                if kind.nested
                else "str"
                if kind.name == "string"
                else "bool"
                if kind.name == "bool"
                else "float"
                if kind.name.startswith("float")
                else "int"
            )
            lines.append(f"    {field.name}: {annotation}")
        for constant in message.constants:
            lines.append(f"    {constant.name} = {literal(constant.value)}")
        lines.append("")
    names = [message.name for message in messages] + list(imported)
    lines.append(
        "_types = {" + ", ".join(f"{name!r}: {name.replace('/', '__')}" for name in names) + "}"
    )
    lines.append(
        "_codec = Codec({"
        + ", ".join(f"{name!r}: {definitions.schema(name)!r}" for name in names)
        + "}, _types)"
    )
    for message in messages:
        symbol = message.name.replace("/", "__")
        lines.extend(
            [
                f"{symbol}._codec = _codec",
                f"{symbol}.__name__ = {message.short_name!r}",
                f"{symbol}.__qualname__ = {message.short_name!r}",
                f"{symbol}.__module__ = {module + '.' + message.package + '.msg'!r}",
            ]
        )
    packages = sorted({message.package for message in messages})
    result = {
        "_types.py": "\n".join(lines) + "\n",
        "_runtime.py": Path(__file__).with_name("templates").joinpath("runtime.py").read_text(),
        "__init__.py": f"__dimos_version__ = {version!r}\n__dimos_abi__ = {ABI!r}\n"
        + "".join(f"from . import {package} as {package}\n" for package in packages),
        "py.typed": "# Generated message packages contain inline type annotations.\n",
    }
    for package in packages:
        result[f"{package}/__init__.py"] = "from . import msg as msg\n"
        result[f"{package}/msg/__init__.py"] = (
            "".join(
                f"from ..._types import {message.name.replace('/', '__')} as {message.short_name}\n"
                for message in messages
                if message.package == package
            )
            + f"__all__ = {[message.short_name for message in messages if message.package == package]!r}\n"
        )
    return result
