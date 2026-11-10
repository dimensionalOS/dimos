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
from collections.abc import Callable, Collection, Sequence
from dataclasses import dataclass
import hashlib
import inspect
import keyword
import sys
import threading
import types
from typing import (
    Any,
    Literal,
    Protocol,
    TypeVar,
    _ProtocolMeta,
    cast,
    get_args,
    get_type_hints,
)

from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.spec.utils import Spec

_Direction = Literal["in", "out"]
_PortFields = tuple[tuple[str, type[Any], str], ...]
_MethodFields = tuple[tuple[str, tuple[str, ...]], ...]
_RefFields = tuple[tuple[str, type[Spec]], ...]
_ModuleT = TypeVar("_ModuleT", bound=Module)


@dataclass(frozen=True)
class PortSpec:
    """One stream of a generated module class."""

    stream: str
    message_type: type[Any]
    direction: _Direction


@dataclass(frozen=True)
class MethodSpec:
    """One method of a generated Spec protocol, by name and parameter names."""

    name: str
    params: tuple[str, ...]


@dataclass(frozen=True)
class RefSpec:
    """One module reference of a generated module class, typed by a generated Spec protocol."""

    name: str
    spec: type[Spec]


class _DynamicSpecMeta(_ProtocolMeta):
    """Metaclass whose CBOR hooks send a generated Spec protocol as its methods."""

    _dynamic_name: str
    _dynamic_methods: tuple[MethodSpec, ...]

    def cbor_encode(cls) -> tuple[str, _MethodFields]:  # noqa: N805
        return cls._dynamic_name, _method_fields(cls._dynamic_methods)

    @classmethod
    def cbor_decode(cls, data: Sequence[Any]) -> type[Spec]:
        name, fields = data
        return make_spec_class(
            name, [MethodSpec(method, tuple(params)) for method, params in fields]
        )


class _DynamicModuleMeta(ABCMeta):
    """Metaclass whose CBOR hooks send the base by reference and rebuild through the factory."""

    _dynamic_base: type[Module]
    _dynamic_ports: tuple[PortSpec, ...]
    _dynamic_refs: tuple[RefSpec, ...]

    def cbor_encode(cls) -> tuple[type[Module], _PortFields, _RefFields]:
        # Plain field tuples so a class that survived a reload of this module still encodes.
        return cls._dynamic_base, _port_fields(cls._dynamic_ports), _ref_fields(cls._dynamic_refs)

    @classmethod
    def cbor_decode(cls, data: Sequence[Any]) -> type[Module]:
        base, port_fields, ref_fields = data
        ports = [
            PortSpec(stream, message_type, cast("_Direction", direction))
            for stream, message_type, direction in port_fields
        ]
        refs = [RefSpec(name, spec) for name, spec in ref_fields]
        return make_module_class(base, ports, refs=refs)


_DIGEST_LEN = 10
_VALID_DIRECTIONS: tuple[str, ...] = get_args(_Direction)

# Process-global so repeated decodes return the identical class, since blueprints compare classes with is.
_memo: dict[tuple[type[Module], tuple[PortSpec, ...], tuple[RefSpec, ...]], type[Module]] = {}
_spec_memo: dict[tuple[str, tuple[MethodSpec, ...]], type[Spec]] = {}
_memo_lock = threading.Lock()


def _resolves_by_reference(cls: type[Any]) -> bool:
    obj: Any = sys.modules.get(cls.__module__)
    for part in cls.__qualname__.split("."):
        obj = getattr(obj, part, None)
        if obj is None:
            return False
    return obj is cls


def _check_identifier(kind: str, name: object) -> None:
    if not isinstance(name, str):
        raise TypeError(f"{kind} must be str, got {type(name).__name__}")
    if not name.isidentifier() or keyword.iskeyword(name):
        raise ValueError(f"{kind} {name!r} is not a valid Python identifier")


def _validate_methods(methods: tuple[MethodSpec, ...]) -> None:
    seen: set[str] = set()
    for method in methods:
        if not isinstance(method, MethodSpec):
            raise TypeError(f"expected MethodSpec, got {type(method).__name__}")
        _check_identifier("method name", method.name)
        if method.name.startswith("_"):
            raise ValueError(
                f"method name {method.name!r} is invalid: leading underscores are reserved"
            )
        if method.name in seen:
            raise ValueError(f"duplicate method {method.name!r}")
        seen.add(method.name)
        for param in method.params:
            _check_identifier(f"method {method.name!r} parameter", param)
        if "self" in method.params or len(set(method.params)) != len(method.params):
            raise ValueError(f"method {method.name!r} has a duplicate or reserved parameter name")


def _validate_refs(base: type[Module], refs: tuple[RefSpec, ...], taken: Collection[str]) -> None:
    seen: set[str] = set()
    for ref in refs:
        if not isinstance(ref, RefSpec):
            raise TypeError(f"expected RefSpec, got {type(ref).__name__}")
        _check_identifier("ref name", ref.name)
        if ref.name in taken:
            raise ValueError(
                f"ref name {ref.name!r} collides with an existing {base.__name__} attribute or port"
            )
        if ref.name in seen:
            raise ValueError(f"duplicate ref name {ref.name!r}")
        seen.add(ref.name)
        # Not isinstance, since a reload of this module replaces the metaclass.
        if not hasattr(ref.spec, "_dynamic_methods"):
            raise TypeError(
                f"ref {ref.name!r}: spec must come from make_spec_class, got {ref.spec!r}"
            )


def _validate(base: type[Module], ports: tuple[PortSpec, ...], reserved: Collection[str]) -> None:
    reserved_names = _reserved_names(base, reserved)
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
        if message_type.__module__ == "__main__":
            raise ValueError(
                f"stream {stream!r}: message type {message_type.__qualname__} is only "
                "defined in __main__; move it into an importable module so worker "
                "processes can reconstruct it"
            )
        if "<locals>" in message_type.__qualname__:
            raise ValueError(
                f"stream {stream!r}: message type {message_type.__qualname__} is defined "
                "inside a function; move it to module level so it can be sent by reference"
            )
        if not _resolves_by_reference(message_type):
            raise ValueError(
                f"stream {stream!r}: message type {message_type.__module__}."
                f"{message_type.__qualname__} cannot be sent by reference; it must be "
                "importable under its own module and qualified name"
            )
    seen: set[str] = set()
    for port in ports:
        if port.stream in seen:
            raise ValueError(f"duplicate stream id {port.stream!r}")
        seen.add(port.stream)


def _reserved_names(base: type[Module], reserved: Collection[str]) -> frozenset[str]:
    return frozenset(dir(base)) | frozenset(get_type_hints(base)) | frozenset(reserved)


def _port_fields(ports: tuple[PortSpec, ...]) -> _PortFields:
    return tuple((p.stream, p.message_type, p.direction) for p in ports)


def _method_fields(methods: tuple[MethodSpec, ...]) -> _MethodFields:
    return tuple((m.name, m.params) for m in methods)


def _ref_fields(refs: tuple[RefSpec, ...]) -> _RefFields:
    return tuple((r.name, r.spec) for r in refs)


def _hash(lines: Sequence[str]) -> str:
    return hashlib.sha256("\n".join(lines).encode()).hexdigest()[:_DIGEST_LEN]


def _digest(base: type[Module], ports: tuple[PortSpec, ...], refs: tuple[RefSpec, ...]) -> str:
    lines = [f"{base.__module__}.{base.__qualname__}"] + [
        f"{p.stream}:{p.direction}:{p.message_type.__module__}.{p.message_type.__qualname__}"
        for p in ports
    ]
    lines += [f"{r.name}:ref:{r.spec.__qualname__}" for r in refs]
    return _hash(lines)


def _annotation(port: PortSpec) -> Any:
    stream_class: Any = In if port.direction == "in" else Out
    return stream_class[port.message_type]


def _stub(method: MethodSpec) -> Callable[..., None]:
    def stub(self: object, *args: object, **kwargs: object) -> None: ...

    stub.__name__ = stub.__qualname__ = method.name
    parameters = [
        inspect.Parameter(name, inspect.Parameter.POSITIONAL_OR_KEYWORD)
        for name in ("self", *method.params)
    ]
    stub.__signature__ = inspect.Signature(parameters)  # type: ignore[attr-defined]
    return stub


def make_spec_class(name: str, methods: Sequence[MethodSpec]) -> type[Spec]:
    """Build or return the memoized Spec protocol with the given methods, in any order."""
    _check_identifier("spec name", name)
    ordered = tuple(methods)
    _validate_methods(ordered)
    canonical = tuple(sorted(ordered, key=lambda m: m.name))
    key = (name, canonical)

    with _memo_lock:
        cached = _spec_memo.get(key)
        if cached is not None:
            return cached

        signatures = [f"{m.name}({','.join(m.params)})" for m in canonical]
        class_name = f"Dynamic{name}_{_hash([name, *signatures])}"
        existing = globals().get(class_name)
        if existing is not None:
            if getattr(existing, "_dynamic_name", None) == name and _method_fields(
                existing._dynamic_methods
            ) == _method_fields(canonical):
                _spec_memo[key] = existing
                return cast("type[Spec]", existing)
            raise ValueError(f"digest collision: {class_name} already exists for different methods")

        namespace: dict[str, Any] = {"__module__": __name__}
        namespace.update({method.name: _stub(method) for method in canonical})
        spec = cast("type[Spec]", _DynamicSpecMeta(class_name, (Spec, Protocol), namespace))
        # Set after creation so they are not protocol members a provider would need.
        spec._dynamic_name = name  # type: ignore[attr-defined]
        spec._dynamic_methods = canonical  # type: ignore[attr-defined]
        _spec_memo[key] = spec
        globals()[class_name] = spec
        return spec


def make_module_class(
    base: type[_ModuleT],
    ports: Sequence[PortSpec],
    reserved: Collection[str] = (),
    refs: Sequence[RefSpec] = (),
) -> type[_ModuleT]:
    """Build or return the memoized subclass of base with the given ports and refs, in any order."""
    ordered = tuple(ports)
    _validate(base, ordered, reserved)
    ordered_refs = tuple(refs)
    _validate_refs(
        base, ordered_refs, _reserved_names(base, reserved) | {p.stream for p in ordered}
    )
    canonical = tuple(sorted(ordered, key=lambda p: p.stream))
    canonical_refs = tuple(sorted(ordered_refs, key=lambda r: r.name))
    key = (base, canonical, canonical_refs)

    with _memo_lock:
        cached = _memo.get(key)
        if cached is not None:
            return cast("type[_ModuleT]", cached)

        name = f"Dynamic{base.__name__}_{_digest(base, canonical, canonical_refs)}"
        existing = globals().get(name)
        if existing is not None:
            # A reload of this module resets the memo while published classes survive. Compare
            # field tuples because the reload also replaces PortSpec and RefSpec.
            if (
                getattr(existing, "_dynamic_base", None) is base
                and _port_fields(existing._dynamic_ports) == _port_fields(canonical)
                and _ref_fields(existing._dynamic_refs) == _ref_fields(canonical_refs)
            ):
                _memo[key] = existing
                return cast("type[_ModuleT]", existing)
            raise ValueError(f"digest collision: {name} already exists for different ports")

        annotations: dict[str, Any] = {port.stream: _annotation(port) for port in canonical}
        annotations.update({ref.name: ref.spec for ref in canonical_refs})
        namespace: dict[str, Any] = {
            "__annotations__": annotations,
            "__module__": __name__,
            "_dynamic_base": base,
            "_dynamic_ports": canonical,
            "_dynamic_refs": canonical_refs,
        }
        klass = cast("type[_ModuleT]", _DynamicModuleMeta(name, (base,), namespace))
        _memo[key] = klass
        # Published so the coordinator's restart path can reload this module and find the class by name.
        globals()[name] = klass
        return klass
