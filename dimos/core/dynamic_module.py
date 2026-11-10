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

"""Module subclasses built at runtime that survive the CBOR worker pipe."""

from abc import ABCMeta
from collections.abc import Collection, Sequence
from dataclasses import dataclass
import hashlib
import keyword
import sys
import threading
import types
from typing import Any, Literal, TypeVar, cast, get_args, get_type_hints

from dimos.core.module import Module
from dimos.core.stream import In, Out

_Direction = Literal["in", "out"]
_PortFields = tuple[tuple[str, type[Any], str], ...]
_ModuleT = TypeVar("_ModuleT", bound=Module)


@dataclass(frozen=True)
class PortSpec:
    """One stream of a generated module class."""

    stream: str
    message_type: type[Any]
    direction: _Direction


class _DynamicModuleMeta(ABCMeta):
    """Metaclass whose CBOR hooks send the base by reference and rebuild through the factory."""

    _dynamic_base: type[Module]
    _dynamic_ports: tuple[PortSpec, ...]

    def cbor_encode(cls) -> tuple[type[Module], _PortFields]:
        # Plain field tuples so a class that survived a reload of this module still encodes.
        return cls._dynamic_base, _port_fields(cls._dynamic_ports)

    @classmethod
    def cbor_decode(cls, data: Sequence[Any]) -> type[Module]:
        base, fields = data
        ports = [
            PortSpec(stream, message_type, cast("_Direction", direction))
            for stream, message_type, direction in fields
        ]
        return make_module_class(base, ports)


_DIGEST_LEN = 10
_VALID_DIRECTIONS: tuple[str, ...] = get_args(_Direction)

# Process-global so repeated decodes return the identical class, since blueprints compare classes with is.
_memo: dict[tuple[type[Module], tuple[PortSpec, ...]], type[Module]] = {}
_memo_lock = threading.Lock()


def _resolves_by_reference(cls: type[Any]) -> bool:
    obj: Any = sys.modules.get(cls.__module__)
    for part in cls.__qualname__.split("."):
        obj = getattr(obj, part, None)
        if obj is None:
            return False
    return obj is cls


def _by_reference_error(cls: type[Any], what: str) -> str | None:
    if cls.__module__ == "__main__":
        return (
            f"{what} {cls.__qualname__} is only defined in __main__; move it into an "
            "importable module so worker processes can reconstruct it"
        )
    if "<locals>" in cls.__qualname__:
        return (
            f"{what} {cls.__qualname__} is defined inside a function; move it to module "
            "level so it can be sent by reference"
        )
    if not _resolves_by_reference(cls):
        return (
            f"{what} {cls.__module__}.{cls.__qualname__} cannot be sent by reference; it "
            "must be importable under its own module and qualified name"
        )
    return None


def _validate(base: type[Module], ports: tuple[PortSpec, ...], reserved: Collection[str]) -> None:
    if not isinstance(base, type) or not issubclass(base, Module):
        raise TypeError(f"base must be a Module subclass, got {base!r}")
    base_error = _by_reference_error(base, "base")
    if base_error is not None:
        raise ValueError(base_error)
    reserved_names = frozenset(dir(base)) | frozenset(get_type_hints(base)) | frozenset(reserved)
    for port in ports:
        if not isinstance(port, PortSpec):
            raise TypeError(f"expected PortSpec, got {type(port).__name__}")
        stream = port.stream
        if not isinstance(stream, str):
            raise TypeError(f"stream id must be str, got {type(stream).__name__}")
        if not stream.isidentifier() or keyword.iskeyword(stream):
            raise ValueError(f"stream id {stream!r} is not a valid Python identifier")
        if stream.startswith("_"):
            raise ValueError(
                f"stream id {stream!r} is invalid: leading underscores are reserved "
                "(underscored streams are invisible to Module.inputs/outputs)"
            )
        if stream in reserved_names:
            raise ValueError(
                f"stream id {stream!r} collides with an existing {base.__name__} attribute"
            )
        if port.direction not in _VALID_DIRECTIONS:
            raise ValueError(
                f"stream {stream!r}: direction must be one of {_VALID_DIRECTIONS}, "
                f"got {port.direction!r}"
            )
        message_type = port.message_type
        # TODO(PY311): drop the GenericAlias check — on 3.10,
        # isinstance(list[int], type) is True.
        if not isinstance(message_type, type) or isinstance(message_type, types.GenericAlias):
            raise TypeError(
                f"stream {stream!r}: message_type must be a class, got {message_type!r}"
            )
        message_error = _by_reference_error(message_type, "message type")
        if message_error is not None:
            raise ValueError(f"stream {stream!r}: {message_error}")
    seen: set[str] = set()
    for port in ports:
        if port.stream in seen:
            raise ValueError(f"duplicate stream id {port.stream!r}")
        seen.add(port.stream)


def _port_fields(ports: tuple[PortSpec, ...]) -> _PortFields:
    return tuple((p.stream, p.message_type, p.direction) for p in ports)


def _digest(base: type[Module], ports: tuple[PortSpec, ...]) -> str:
    lines = [f"{base.__module__}.{base.__qualname__}"] + [
        f"{p.stream}:{p.direction}:{p.message_type.__module__}.{p.message_type.__qualname__}"
        for p in ports
    ]
    return hashlib.sha256("\n".join(lines).encode()).hexdigest()[:_DIGEST_LEN]


def _annotation(port: PortSpec) -> Any:
    stream_class: Any = In if port.direction == "in" else Out
    return stream_class[port.message_type]


def make_module_class(
    base: type[_ModuleT], ports: Sequence[PortSpec], reserved: Collection[str] = ()
) -> type[_ModuleT]:
    """Build or return the memoized subclass of base with the given ports, in any order."""
    ordered = tuple(ports)
    _validate(base, ordered, reserved)
    canonical = tuple(sorted(ordered, key=lambda p: p.stream))
    key = (base, canonical)

    with _memo_lock:
        cached = _memo.get(key)
        if cached is not None:
            return cast("type[_ModuleT]", cached)

        name = f"Dynamic{base.__name__}_{_digest(base, canonical)}"
        existing = globals().get(name)
        if existing is not None:
            # A reload of this module resets the memo while published classes survive. Compare
            # field tuples because the reload also replaces PortSpec.
            if getattr(existing, "_dynamic_base", None) is base and _port_fields(
                existing._dynamic_ports
            ) == _port_fields(canonical):
                _memo[key] = existing
                return cast("type[_ModuleT]", existing)
            raise ValueError(f"digest collision: {name} already exists for different ports")

        namespace: dict[str, Any] = {
            "__annotations__": {port.stream: _annotation(port) for port in canonical},
            "__module__": __name__,
            "_dynamic_base": base,
            "_dynamic_ports": canonical,
        }
        klass = cast("type[_ModuleT]", _DynamicModuleMeta(name, (base,), namespace))
        _memo[key] = klass
        # Published so the coordinator's restart path can reload this module and find the class by name.
        globals()[name] = klass
        return klass
