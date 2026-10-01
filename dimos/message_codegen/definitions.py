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

"""Resolve ROS2 message definitions without a ROS environment or build system."""

from __future__ import annotations

from collections.abc import Iterable
from contextlib import redirect_stderr
from dataclasses import dataclass
from io import StringIO
from pathlib import Path
from typing import Any

from dimos.message_codegen._vendor.rosidl_parser import (
    InvalidSpecification,
    InvalidValue,
    parse_message_string,
)
from dimos.message_codegen.providers import schema_roots

BUNDLED_SCHEMAS = Path(__file__).with_name("schemas")


@dataclass(frozen=True)
class FieldType:
    """A scalar, fixed array, or sequence; bounds are separate from wire type."""

    name: str
    is_array: bool = False
    array_size: int | None = None
    array_bounded: bool = False
    string_bound: int | None = None

    @property
    def nested(self) -> bool:
        return "/" in self.name

    @property
    def sequence(self) -> bool:
        return self.is_array and (self.array_size is None or self.array_bounded)


@dataclass(frozen=True)
class Field:
    name: str
    type: FieldType
    default: Any = None


@dataclass(frozen=True)
class Constant:
    name: str
    type: str
    value: Any


@dataclass(frozen=True)
class Message:
    name: str
    fields: tuple[Field, ...]
    constants: tuple[Constant, ...]
    source: Path
    text: str

    @property
    def package(self) -> str:
        return self.name.split("/")[0]

    @property
    def short_name(self) -> str:
        return self.name.split("/")[-1]

    @property
    def dependencies(self) -> tuple[str, ...]:
        return tuple(sorted({field.type.name for field in self.fields if field.type.nested}))


def parse_message(path: Path) -> Message:
    """Read a package/msg/Type.msg, retaining its original text for viewers."""
    if path.parent.name != "msg" or path.suffix != ".msg":
        raise ValueError(f"{path}: expected package/msg/Type.msg")
    package = path.parent.parent.name
    text = path.read_text(encoding="utf-8")
    try:
        with redirect_stderr(StringIO()):
            parsed = parse_message_string(package, path.stem, text)
    except (ValueError, TypeError, NameError, InvalidSpecification, InvalidValue) as exc:
        # The upstream parser reports the bad declaration, but not its line.
        # Re-parse declarations only on failure to provide source diagnostics.
        for line_number, line in enumerate(text.splitlines(), 1):
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            try:
                with redirect_stderr(StringIO()):
                    parse_message_string(package, path.stem, line)
            except (ValueError, TypeError, NameError, InvalidSpecification, InvalidValue):
                raise ValueError(f"{path}:{line_number}: {exc}") from exc
        raise ValueError(f"{path}: {exc}") from exc

    fields = []
    for field in parsed.fields:
        type_ = field.type
        if type_.type in {"time", "duration"} and type_.pkg_name is None:
            raise ValueError(
                f"{path}: use builtin_interfaces/{type_.type.title()}, not ROS1 {type_.type}"
            )
        name = f"{type_.pkg_name}/msg/{type_.type}" if type_.pkg_name else type_.type
        default = field.default_value
        fields.append(
            Field(
                field.name,
                FieldType(
                    name,
                    type_.is_array,
                    type_.array_size,
                    type_.is_upper_bound,
                    type_.string_upper_bound,
                ),
                tuple(default) if isinstance(default, list) else default,
            )
        )
    return Message(
        f"{package}/msg/{path.stem}",
        tuple(fields),
        tuple(Constant(value.name, value.type, value.value) for value in parsed.constants),
        path,
        text,
    )


class Definitions:
    """A closed set of local definitions with dependency-first resolution."""

    def __init__(
        self, roots: Iterable[Path], *, bundled: bool = True, installed: bool = False
    ) -> None:
        self._messages: dict[str, Message] = {}
        for root in (
            ([BUNDLED_SCHEMAS] if bundled else [])
            + list(roots)
            + (list(schema_roots()) if installed else [])
        ):
            if not root.is_dir():
                raise ValueError(f"Schema root does not exist: {root}")
            for path in sorted(root.rglob("*.msg")):
                message = parse_message(path)
                previous = self._messages.get(message.name)
                if previous is not None:
                    if (previous.fields, previous.constants) != (message.fields, message.constants):
                        raise ValueError(
                            f"Conflicting definition {message.name}: {previous.source} and {path}"
                        )
                    continue
                self._messages[message.name] = message

    def resolve(self, names: Iterable[str] | None = None) -> tuple[Message, ...]:
        """Return the full dependency closure, with dependencies before users."""
        result: dict[str, Message] = {}
        visiting: list[str] = []

        def visit(name: str) -> None:
            if name in result:
                return
            if name in visiting:
                raise ValueError(f"Recursive message definition: {' -> '.join([*visiting, name])}")
            if name not in self._messages:
                origin = self._messages[visiting[-1]].source if visiting else "requested root"
                raise ValueError(f"{origin}: unresolved message dependency {name}")
            visiting.append(name)
            message = self._messages[name]
            for dependency in message.dependencies:
                visit(dependency)
            visiting.pop()
            result[name] = message

        for name in sorted(self._messages if names is None else names):
            visit(name)
        return tuple(result.values())

    def schema(self, name: str) -> str:
        """ROS2 MCAP schema text, including every transitive dependency once."""
        closure = self.resolve([name])
        root = self._messages[name]
        sections = [root.text.rstrip() + "\n"]
        for message in sorted(closure, key=lambda item: item.name):
            if message.name != name:
                # Concatenated .msg sections use the package/resource spelling
                # used by field references. The outer schema name stays pkg/msg/T.
                resource_name = message.name.replace("/msg/", "/")
                sections.append(f"{'=' * 80}\nMSG: {resource_name}\n{message.text.rstrip()}\n")
        return "".join(sections)
