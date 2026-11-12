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

"""Plain data from messages, messages from plain data, and the shape of a type."""

from collections.abc import Mapping
from typing import ClassVar, Protocol, TypeGuard

from dimos.msgs.protocol import DimosMsg
from dimos.orchestration.expressions import UNKNOWN, Plain, Shape
from dimos.web.lcm_codec import schema_class_for

_EXAMPLES: tuple[tuple[object, Plain], ...] = (
    (bool, False),
    (int, 0),
    (float, 0.0),
    (str, ""),
    (None, None),
    (type(None), None),
)


class _Generated(DimosMsg, Protocol):
    """A generated dimos_lcm class, which lists its own fields."""

    __slots__: ClassVar[list[str]]
    __dimensions__: ClassVar[list[list[int | str] | None]]

    @classmethod
    def _get_field_type(cls, name: str) -> "type[_Generated] | None": ...


def _is_message_type(annotation: object) -> TypeGuard[type[DimosMsg]]:
    return isinstance(annotation, type) and callable(getattr(annotation, "lcm_encode", None))


def to_plain(value: object, path: str = "") -> Plain:
    """Plain data from a message, a field of one or an RPC return.

    A message becomes a map of its LCM fields by name. `path` names the value in an error.
    """
    if value is None or isinstance(value, bool | int | float | str):
        return value
    if isinstance(value, list | tuple):
        return [to_plain(item, f"{path}[{index}]") for index, item in enumerate(value)]
    if isinstance(value, dict) and all(isinstance(key, str) for key in value):
        return {key: to_plain(item, f"{path}.{key}".lstrip(".")) for key, item in value.items()}
    if isinstance(value, DimosMsg):
        try:
            generated: type[_Generated] = schema_class_for(type(value))
        except ValueError as error:
            raise TypeError(str(error)) from None
        # A wrapper is read through its LCM bytes, so its fields are the message's own.
        message = value if type(value) is generated else generated.lcm_decode(value.lcm_encode())
        return {
            slot: to_plain(getattr(message, slot), f"{path}.{slot}".lstrip("."))
            for slot in generated.__slots__
        }
    what = f"`{path}`" if path else "the value"
    raise TypeError(f"{what} is {type(value).__name__}, which is not plain data")


def from_plain(value: Plain, annotation: object) -> object:
    """An RPC argument of the annotated type. A map becomes a message, and the rest pass through."""
    if not (isinstance(value, dict) and _is_message_type(annotation)):
        return value
    generated: type[_Generated] = schema_class_for(annotation)
    message = _message(generated, value)
    return message if annotation is generated else annotation.lcm_decode(message.lcm_encode())


def _message(generated: type[_Generated], fields: Mapping[str, Plain]) -> _Generated:
    """A generated message from a map of its fields. A field left out is its type's default."""
    unknown = sorted(fields.keys() - set(generated.__slots__))
    if unknown:
        raise TypeError(f"`{unknown[0]}` is not a field of {generated.__name__}")
    message = generated()
    for name, dims in zip(generated.__slots__, generated.__dimensions__, strict=True):
        inner = generated._get_field_type(name)
        value = fields.get(name)
        # A nested message is always built here, since every instance shares the generated default.
        if inner is None:
            if name in fields:
                setattr(message, name, value)
        elif dims is None:
            setattr(message, name, _message(inner, _map(value, name)))
        elif isinstance(value, list):
            setattr(message, name, [_message(inner, _map(item, name)) for item in value])
        else:
            setattr(message, name, [])
    return message


def _map(value: Plain, name: str) -> Mapping[str, Plain]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise TypeError(f"`{name}` must be a map of its fields")
    return value


def shape_of(annotation: object) -> Shape:
    """The shape of a stream's message type or of an RPC annotation, for the static checks."""
    for kind, example in _EXAMPLES:
        if annotation is kind:
            return example
    if _is_message_type(annotation):
        try:
            return to_plain(schema_class_for(annotation)())
        except (TypeError, ValueError):
            return UNKNOWN
    return UNKNOWN
