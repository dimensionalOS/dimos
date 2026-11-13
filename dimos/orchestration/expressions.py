# Copyright 2025-2026 Dimensional Inc.
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

"""CEL expressions, checked against the shape of every blackboard key when compiled."""

from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Final, Literal, TypeAlias

from cel_expr_python import cel

Plain: TypeAlias = None | bool | int | float | str | list["Plain"] | dict[str, "Plain"]


class Marker(Enum):
    UNKNOWN = "unknown"
    NUMBER = "number"


UNKNOWN: Final = Marker.UNKNOWN
NUMBER: Final = Marker.NUMBER
Shape: TypeAlias = Plain | Literal[Marker.UNKNOWN]
Wanted: TypeAlias = Plain | Marker


class ExpressionError(Exception):
    """An expression that did not compile or did not evaluate."""

    def __init__(self, message: str, unset: bool = False) -> None:
        super().__init__(message)
        self.unset = unset


@dataclass(frozen=True)
class Expression:
    source: str
    program: cel.Expression
    reads: tuple[str, ...]


def _type(shape: Shape) -> cel.Type:
    """The type a shape has in the strict environment."""
    if isinstance(shape, bool):
        return cel.Type.BOOL
    if isinstance(shape, int):
        return cel.Type.INT
    if isinstance(shape, float):
        return cel.Type.DOUBLE
    if isinstance(shape, str):
        return cel.Type.STRING
    if isinstance(shape, list):
        return cel.Type.List(_element_type(shape))
    if isinstance(shape, dict):
        fields = ",".join(f"{name}:{_type(value).name()}" for name, value in sorted(shape.items()))
        return cel.Type.AbstractType("{" + fields + "}")
    return cel.Type.DYN


def _element_type(items: list[Plain]) -> cel.Type:
    if any(isinstance(item, dict) for item in items):
        return cel.Type.DYN
    types = {_type(item).name(): _type(item) for item in items}
    return next(iter(types.values())) if len(types) == 1 else cel.Type.DYN


def _declare(
    path: str, shape: Shape, strict: dict[str, cel.Type], loose: dict[str, cel.Type]
) -> None:
    """Declare one key and every path under it in both environments."""
    strict[path] = _type(shape)
    if not isinstance(shape, dict):
        loose[path] = strict[path]
        return
    loose[path] = cel.Type.DYN
    for name, value in shape.items():
        _declare(f"{path}.{name}", value, strict, loose)


def _fits(result: str, wanted: str) -> bool:
    """Whether a result type is acceptable. An unknown type on either side passes."""
    if "DYN" in (result, wanted) or result == wanted:
        return True
    if "LIST<DYN>" in (result, wanted):
        return result.startswith("LIST<") and wanted.startswith("LIST<")
    return result.startswith("MAP<") and wanted.startswith("{")


def _varint(value: int) -> bytes:
    encoded = bytearray()
    while value > 0x7F:
        encoded.append(value & 0x7F | 0x80)
        value >>= 7
    encoded.append(value)
    return bytes(encoded)


def _marker(path: str) -> bytes:
    """A name as a serialized checked expression holds it: field 1, its length, its text."""
    text = path.encode()
    return b"\n" + _varint(len(text)) + text


class Compiler:
    """Compiles expressions against the shape of every blackboard key."""

    def __init__(self, shapes: Mapping[str, Shape]) -> None:
        strict: dict[str, cel.Type] = {}
        loose: dict[str, cel.Type] = {}
        for key, shape in shapes.items():
            _declare(key, shape, strict, loose)
        self._markers = {path: _marker(path) for path in strict}
        self._strict = cel.NewEnv(variables=strict)
        self._loose = cel.NewEnv(variables=loose)

    def compile(self, source: str, gives: Wanted = UNKNOWN) -> Expression:
        """Compile one expression. `gives` is NUMBER, or an example of the value it must produce."""
        try:
            checked = self._strict.compile(source)
            program = self._loose.compile(source)
        except RuntimeError as error:
            reason = str(error).splitlines()[0].removeprefix("INVALID_ARGUMENT: ERROR: ")
            raise ExpressionError(f"`{source}`: {reason}") from None
        result: str = checked.return_type().name()
        wanted = ["INT", "DOUBLE"] if gives is NUMBER else [_type(gives).name()]
        if not any(_fits(result, name) for name in wanted):
            raise ExpressionError(f"`{source}` gives {result}, and {' or '.join(wanted)} is needed")
        serialized: bytes = checked.serialize()
        reads = tuple(path for path, marker in self._markers.items() if marker in serialized)
        return Expression(source, program, reads)


def _read(blackboard: Mapping[str, Plain], path: str) -> tuple[bool, Plain]:
    """Find the key a path is under and follow the rest of the path into its value."""
    parts = path.split(".")
    for cut in range(len(parts), 0, -1):
        key = ".".join(parts[:cut])
        if key not in blackboard:
            continue
        value = blackboard[key]
        for part in parts[cut:]:
            if not isinstance(value, dict) or part not in value:
                raise ExpressionError(f"`{path}`: the value of `{key}` has no field `{part}`")
            value = value[part]
        return True, value
    return False, None


def evaluate(expression: Expression, blackboard: Mapping[str, Plain]) -> Plain:
    """Evaluate an expression. A missing field is an error only if the result needs it."""
    data: dict[str, Plain] = {}
    missing: list[ExpressionError] = []
    for path in expression.reads:
        try:
            has_value, read = _read(blackboard, path)
        except ExpressionError as error:
            missing.append(error)
            continue
        if has_value:
            data[path] = read
    value = expression.program.eval(data=data)
    plain: Plain = value.plain_value()
    if value.type() != cel.Type.ERROR:
        return plain
    if missing:
        raise missing[0]
    unset = len(data) < len(expression.reads)
    raise ExpressionError(f"`{expression.source}`: {plain}", unset=unset)


def evaluate_bool(expression: Expression, blackboard: Mapping[str, Plain]) -> bool:
    """Evaluate a true-or-false expression. An error while a key it reads has no value is false."""
    try:
        value = evaluate(expression, blackboard)
    except ExpressionError as error:
        if error.unset:
            return False
        raise
    if not isinstance(value, bool):
        raise ExpressionError(f"`{expression.source}` gave {value!r}, and true or false is needed")
    return value
