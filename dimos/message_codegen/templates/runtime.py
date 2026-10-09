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

"""Shared pure-Python value/buffer contracts; CDR encoding is supplied by rosbags."""

from __future__ import annotations

import copy
from dataclasses import replace
import struct
from types import FunctionType
from typing import Any, ClassVar, TypeVar, cast
import weakref

import numpy as np
from rosbags.serde import cdr
from rosbags.serde.utils import compile_lines
from rosbags.typesys import get_types_from_msg
from rosbags.typesys.store import Typestore

_DTYPES = {
    "bool": "bool",
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
# name, scalar type, array, size/bound, bounded sequence, string bound, default
Field = tuple[str, str, bool, int | None, bool, int | None, Any]
_MessageT = TypeVar("_MessageT", bound="Message")


def _scalar(kind: str, value: Any, codec: Codec, bound: int | None = None) -> Any:
    if "/" in kind:
        expected = codec.types[kind]
        if not isinstance(value, expected):
            raise TypeError(f"Expected {kind}, got {type(value).__name__}")
        return copy.deepcopy(value)
    if kind == "string":
        if not isinstance(value, str):
            raise TypeError("Expected string")
        if bound is not None and len(value.encode()) > bound:
            raise ValueError("String exceeds string bound")
        return value
    if kind == "bool":
        if not isinstance(value, (bool, np.bool_)):
            raise TypeError("Expected bool")
        return bool(value)
    if kind.startswith("float"):
        if not isinstance(value, (int, float, np.number)):
            raise TypeError(f"Expected {kind}")
        return float(np.asarray(value, dtype=_DTYPES[kind]))
    if not isinstance(value, (int, np.integer)):
        raise TypeError(f"Expected {kind}")
    limits = np.iinfo(_DTYPES[kind])
    if not limits.min <= int(value) <= limits.max:
        raise ValueError(f"Value outside {kind} range")
    return int(value)


class Sequence:
    """Owned sequence with read-only borrowed NumPy views and explicit mutable copies."""

    def __init__(self, field: Field, values: Any, owner: Message) -> None:
        self.field, self.owner = field, owner
        kind = field[1]
        self.values: np.ndarray[Any, Any] | list[Any]
        if kind in _DTYPES:
            array = (
                np.frombuffer(values, dtype=np.uint8)
                if kind == "uint8" and isinstance(values, (bytes, bytearray))
                else np.asarray(values)
            )
            if array.ndim != 1:
                raise ValueError("Expected a one-dimensional sequence")
            if kind != "bool" and not kind.startswith("float") and array.size:
                limits = np.iinfo(_DTYPES[kind])
                if (
                    not np.issubdtype(array.dtype, np.integer)
                    or np.any(array < limits.min)
                    or np.any(array > limits.max)
                ):
                    raise ValueError(f"Sequence element outside {kind} range")
            self.values = np.array(array, dtype=_DTYPES[kind], copy=True)
        else:
            self.values = [_scalar(kind, item, owner._codec) for item in values]
        if field[3] is not None and not field[4] and len(self) != field[3]:
            raise ValueError("Incorrect fixed-array length")
        self._attach()

    def _attach(self) -> None:
        if self.field[1] not in _DTYPES:
            for value in self.values:
                if hasattr(value, "_fields"):
                    value._attach(self.owner)

    def validate(self) -> None:
        size = self.field[3]
        if size is not None and (len(self) > size if self.field[4] else len(self) != size):
            raise ValueError("Sequence exceeds sequence bound or fixed-array length")
        if self.field[1] not in _DTYPES:
            for value in self.values:
                if hasattr(value, "_fields"):
                    value.validate()
                elif self.field[5] is not None and len(value.encode()) > self.field[5]:
                    raise ValueError("String exceeds string bound")

    def __len__(self) -> int:
        return len(self.values)

    def __getitem__(self, index: int) -> Any:
        value = self.values[index]
        return (
            copy.deepcopy(value)
            if hasattr(value, "_fields")
            else value.item()
            if isinstance(value, np.generic)
            else value
        )

    def __setitem__(self, index: int, value: Any) -> None:
        converted = _scalar(self.field[1], value, self.owner._codec, self.field[5])
        if hasattr(converted, "_fields"):
            converted._attach(self.owner)
        self.values[index] = converted

    def __iter__(self) -> Any:
        return (self[index] for index in range(len(self)))

    def __repr__(self) -> str:
        return repr(list(self))

    def __eq__(self, other: Any) -> bool:
        try:
            return list(self) == list(other)
        except TypeError:
            return False

    def extend(self, values: Any) -> None:
        if self.field[3] is not None and not self.field[4]:
            raise AttributeError("Fixed arrays cannot resize")
        self.owner._check_borrow()
        extra = Sequence((*self.field[:3], None, *self.field[4:]), values, self.owner)
        combined = (
            np.concatenate((self.values, extra.values))
            if isinstance(self.values, np.ndarray)
            else self.values + extra.values
        )
        self.values = combined
        self._attach()

    def append(self, value: Any) -> None:
        self.extend([value])

    def clear(self) -> None:
        if self.field[3] is not None and not self.field[4]:
            raise AttributeError("Fixed arrays cannot resize")
        self.owner._check_borrow()
        self.values = (
            np.empty(0, dtype=self.values.dtype) if isinstance(self.values, np.ndarray) else []
        )

    def view(self, dtype: Any = None) -> np.ndarray[Any, Any]:
        if not isinstance(self.values, np.ndarray):
            raise TypeError("Only numeric sequences have NumPy views")
        result = np.frombuffer(memoryview(self.values).toreadonly(), dtype=self.values.dtype)
        if dtype is not None:
            result = result.view(dtype)
        owner = self.owner._root()
        owner._borrows += 1
        weakref.finalize(result, owner._release)
        return result

    def copy(self) -> np.ndarray[Any, Any]:
        if not isinstance(self.values, np.ndarray):
            raise TypeError("Only numeric sequences have NumPy copies")
        return self.values.copy()

    def byteswap(self) -> np.ndarray[Any, Any]:
        if not isinstance(self.values, np.ndarray):
            raise TypeError("Only numeric sequences support byte swapping")
        return self.values.byteswap()


class Message:
    msg_name: ClassVar[str]
    schema: ClassVar[str]
    _fields: ClassVar[tuple[Field, ...]] = ()
    _codec: ClassVar[Codec]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        if len(args) > len(self._fields):
            if self._fields or len(args) != 1:
                raise TypeError("Too many message arguments")
            args = ()
        self._owner: Message | None = None
        self._borrows = 0
        if not self._fields:
            self.structure_needs_at_least_one_member = 0
        positional = {field[0]: value for field, value in zip(self._fields, args, strict=False)}
        if positional.keys() & kwargs.keys():
            raise TypeError("Duplicate message arguments")
        values = positional | kwargs
        unknown = values.keys() - {field[0] for field in self._fields}
        if unknown:
            raise TypeError(f"Unknown message field: {sorted(unknown)[0]}")
        for field in self._fields:
            name, kind, array, size, bounded, _, default = field
            if name in values:
                value = values[name]
            elif default is not None:
                value = default
            elif array:
                value = (
                    [] if size is None or bounded else [self._default(kind) for _ in range(size)]
                )
            else:
                value = self._default(kind)
            setattr(self, name, value)
        self.validate()

    def _default(self, kind: str) -> Any:
        if "/" in kind:
            return self._codec.types[kind]()
        return (
            ""
            if kind == "string"
            else False
            if kind == "bool"
            else 0.0
            if kind.startswith("float")
            else 0
        )

    def __setattr__(self, name: str, value: Any) -> None:
        field = next((item for item in self._fields if item[0] == name), None)
        if field is not None:
            if field[2] or "/" in field[1]:
                self._check_borrow()
            value = (
                Sequence(field, value, self) if field[2] else _scalar(field[1], value, self._codec)
            )
            if hasattr(value, "_fields"):
                value._attach(self)
        object.__setattr__(self, name, value)

    def _root(self) -> Message:
        return self._owner._root() if self._owner is not None else self

    def _attach(self, owner: Message) -> None:
        object.__setattr__(self, "_owner", owner)

    def _check_borrow(self) -> None:
        if getattr(self._root(), "_borrows", 0):
            raise BufferError(
                "Cannot replace or resize message storage while a NumPy view is borrowed"
            )

    def _release(self) -> None:
        self._borrows -= 1

    def validate(self) -> None:
        for field in self._fields:
            value = getattr(self, field[0])
            if field[2] or hasattr(value, "_fields"):
                value.validate()
            else:
                _scalar(field[1], value, self._codec, field[5])

    def __deepcopy__(self, memo: dict[int, Any]) -> Message:
        return type(self)(
            **{
                field[0]: getattr(self, field[0]).values if field[2] else getattr(self, field[0])
                for field in self._fields
            }
        )

    def __reduce__(self) -> Any:
        return type(self).decode, (self.encode(),)

    def encode(self, little_endian: bool = True) -> bytes:
        self.validate()
        return bytes(
            self._codec.store.serialize_cdr(self, self.msg_name, little_endian=little_endian)
        )

    @classmethod
    def decode(cls: type[_MessageT], data: bytes) -> _MessageT:
        if len(data) < 4 or data[:1] != b"\0" or data[1] not in (0, 1) or data[2:4] != b"\0\0":
            raise ValueError("Expected plain CDR/XCDR1 encapsulation")
        body = memoryview(data)[4:]
        definition = cls._codec.store.get_msgdef(cls.msg_name)
        decoder = definition.deserialize_cdr_le if data[1] else definition.deserialize_cdr_be
        try:
            value, position = decoder(body, 0, cls, cls._codec.store)
            if position != len(body):
                raise ValueError("Trailing bytes after CDR message")
            return cast("_MessageT", value)
        except (struct.error, IndexError, OverflowError, AssertionError) as error:
            raise ValueError("Invalid or truncated CDR message") from error


class Codec:
    def __init__(self, schemas: dict[str, str], types: dict[str, type[Message]]) -> None:
        self.types = types
        self.store = _CheckedStore()
        definitions = {}
        for name, schema in schemas.items():
            definitions.update(get_types_from_msg(schema, name))
        self.store.register(definitions)
        # Rosbags accepts constructor-compatible generated classes, not only its dataclasses.
        self.store.types.update(cast("Any", types))
        self.store.cache.clear()


def _checked_compile(lines: list[str]) -> Any:
    """Add strict value checks to pinned rosbags code, without duplicating CDR layout."""
    checked = []
    for line in lines:
        checked.append(line)
        indent = line[: len(line) - len(line.lstrip())]
        condition, error = "", ""
        if "length = unpack_int32_" in line:
            condition = "length <= 0 or pos + 4 + length > len(rawdata) or rawdata[pos + 4 + length - 1] != 0"
            error = "Invalid or truncated CDR string"
        elif "size = unpack_int32_" in line:
            condition = "size < 0 or size > len(rawdata) - pos - 4"
            error = "Invalid CDR sequence length"
        elif "value = unpack_bool_" in line:
            condition = "rawdata[pos] > 1"
            error = "Invalid CDR bool"
        elif "numpy.frombuffer" in line and "dtype=numpy.bool" in line:
            condition = "numpy.any(val.view(numpy.uint8) > 1)"
            error = "Invalid CDR bool"
        if condition:
            checked.extend(
                [
                    indent + "if " + condition + ":",
                    indent + "  raise ValueError(" + repr(error) + ")",
                ]
            )
    return compile_lines(checked)


# Clone the generator with a local compiler callback; never patch rosbags globals.
_checked_decoder = FunctionType(
    cdr.generate_deserialize_cdr.__code__,
    {
        **cdr.generate_deserialize_cdr.__globals__,
        "compile_lines": _checked_compile,
    },
)


class _CheckedStore(Typestore):
    def get_msgdef(self, typename: str) -> Any:
        if typename not in self.cache:
            definition = super().get_msgdef(typename)
            self.cache[typename] = replace(
                definition,
                deserialize_cdr_le=_checked_decoder(definition.fields, self, "le"),
                deserialize_cdr_be=_checked_decoder(definition.fields, self, "be"),
            )
        return self.cache[typename]
