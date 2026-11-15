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

"""CBOR serialization for RPC payloads and the worker pipe.

JSON-like values (None, bool, int, float, str, bytes, list, dict with primitive keys)
map to native CBOR. Every other supported type travels inside a CBOR tag so it comes
back as the same Python type in the other process: tuple, NamedTuple, set, frozenset,
numpy arrays and scalars, Enum (by value, so Flag combinations survive), DimosMsg (as
its class plus LCM bytes), dataclasses, pydantic models, Path, classes, type forms
(``list[float]``, ``typing.List[float]``, ``X | Y``, ``Optional[X]``, ``Literal[...]``,
``Any``), module-level functions, functools.partial, and any class that implements
``cbor_encode``/``cbor_decode`` (see ``CborEncodable``) or is registered with
``register_codec``. Anything else raises ``TypeError`` at encode time, naming the
value's path in the payload. There is no pickle-style fallback.

Classes and functions are sent by reference as ``"module:qualname"`` and must be
importable under that name on the receiving side.

Dataclasses travel as their ``fields()`` and are rebuilt without ``__init__`` or
``__post_init__``: the sender already normalized the values, and anything
``__post_init__`` or a cached property derives stays behind. A dataclass that needs
such state on the receiving side recomputes it lazily or implements the hooks.

Pydantic models travel as their fields and extras and are rebuilt with
``model_construct``: validation does not run again, ``model_post_init`` does, and
private attributes start from their defaults, so private state is process-local.

Decoding never calls a callable the payload chose. It imports modules by name and
calls only ``cbor_decode``, ``lcm_decode``, ``model_construct``, ``_make``,
``__new__`` and ``__class_getitem__`` of the classes it resolves, plus the decoders
this process registered with ``register_codec``.
"""

from __future__ import annotations

from collections.abc import Callable
import dataclasses
import enum
import functools
import importlib
import operator
import pathlib
import sys
import types
import typing
from typing import Any, Protocol

import cbor2
from cbor2 import CBORTag
import numpy as np
from pydantic import BaseModel

TAG_TUPLE = 4040
TAG_NAMEDTUPLE = 4041
TAG_SET = 4042
TAG_FROZENSET = 4043
TAG_NDARRAY = 4044
TAG_NPSCALAR = 4045
TAG_ENUM = 4046
TAG_DIMOSMSG = 4047
TAG_DATACLASS = 4048
TAG_PYDANTIC = 4049
TAG_PATH = 4050
TAG_CLASS = 4051
TAG_FUNCTION = 4052
TAG_PARTIAL = 4053
TAG_CUSTOM = 4054
TAG_REGISTERED = 4055
TAG_DICT = 4056
TAG_GENERIC_ALIAS = 4057
TAG_UNION = 4058
TAG_ELLIPSIS = 4059
TAG_LITERAL = 4060

_PRIMITIVES = (type(None), bool, int, float, str, bytes)


class CborDecodeError(ValueError):
    """The payload is not valid CBOR for this codec."""


class CborEncodable(Protocol):
    """Opt-in serialization for classes the generic rules do not cover.

    ``cbor_encode`` returns any encodable value; ``cbor_decode`` rebuilds an instance
    from it. The returned object may be a stand-in of another class (a stream handle
    decodes to its remote twin), which is why ``cbor_decode`` is typed as ``Any``.
    The codec looks both hooks up on the class, never on the instance.
    """

    def cbor_encode(self) -> Any: ...

    @classmethod
    def cbor_decode(cls, data: Any) -> Any: ...


# type -> (encode, decode, name of the module that registered them)
_REGISTRY: dict[type, tuple[Callable[[Any], Any], Callable[[Any], Any], str]] = {}


def register_codec(type_: type, encode: Callable[[Any], Any], decode: Callable[[Any], Any]) -> None:
    """Make instances of ``type_`` (and subclasses) encodable without touching the class.

    ``decode`` must be a module-level function, and this call must run when its module
    is imported: a payload names the type and that module, the receiving process imports
    the module and then uses the decoder from its own registry. A decoder is never taken
    from the payload.
    """
    _ref(decode, repr(decode))
    _REGISTRY[type_] = (encode, decode, decode.__module__)


def encode(obj: Any) -> bytes:
    """Encode ``obj`` to CBOR bytes. Raises ``TypeError`` for any value that cannot cross."""
    try:
        return cbor2.dumps(_prepare(obj, "value", set()))
    except TypeError:
        raise
    except Exception as e:  # cbor2 itself, e.g. a str holding a lone surrogate
        raise TypeError(f"Cannot CBOR-encode value: {e}") from e


def decode(data: bytes | bytearray | memoryview) -> Any:
    """Decode CBOR bytes produced by ``encode``. Raises ``CborDecodeError`` on bad input."""
    try:
        return cbor2.loads(data, semantic_decoders=_DECODERS, tag_hook=_reject_unknown_tag)
    except CborDecodeError:
        raise
    except cbor2.CBORDecodeError as e:
        detail = f"{e}: {e.__cause__}" if e.__cause__ is not None else str(e)
        raise CborDecodeError(detail) from e


def _prepare(obj: Any, path: str, active: set[int]) -> Any:
    """Map an object graph to values cbor2 encodes natively (CBORTag for our tags).

    This walk is the cost of the codec (about 10x pickle on many small objects). A
    cbor2 ``encoders=`` hybrid was measured in 2026-10 and did not help: the C encoder
    resolves types along the MRO, so ``str``/``int``/``float`` need exact-type Python
    wrappers to keep ``Key(str)``, ``IntEnum`` and ``np.float64`` from being flattened,
    and those wrappers cost as much as this walk. Send arrays, not tuples of scalars.
    """
    t = type(obj)
    if t in _PRIMITIVES:
        return obj
    if id(obj) in active:
        raise TypeError(f"Cannot CBOR-encode a cyclic reference at {path}")
    active.add(id(obj))
    try:
        return _prepare_object(obj, t, path, active)
    finally:
        active.discard(id(obj))


def _prepare_object(obj: Any, t: type, path: str, active: set[int]) -> Any:
    if t is list:
        return _prepare_items(obj, path, active)
    if t is dict:
        return _prepare_dict(obj, path, active)
    if t is tuple:
        return CBORTag(TAG_TUPLE, _prepare_items(obj, path, active))

    # Always look hooks up on the class: proxies fabricate instance attributes in
    # __getattr__ (MethodCallProxy would send a worker request) and lazy paths resolve a
    # checkout in __getattribute__. For a class object, t is its metaclass, which lets a
    # metaclass serialize classes that are not importable by name.
    custom_encode = getattr(t, "cbor_encode", None)
    if callable(custom_encode):
        payload = _hook(functools.partial(custom_encode, obj), t, path)
        return CBORTag(
            TAG_CUSTOM, [_ref(t, path), _prepare(payload, f"{path}.cbor_encode()", active)]
        )
    # Type forms come before the class check: on Python 3.10 isinstance(list[int], type)
    # is True. typing's aliases and special forms all live in the typing module, but so
    # does the metaclass of Protocol subclasses, which are ordinary classes.
    if isinstance(obj, (types.GenericAlias, types.UnionType)) or (
        t.__module__ == "typing" and not isinstance(obj, type)
    ):
        return _prepare_type_form(obj, path, active)
    if isinstance(obj, type):
        return CBORTag(TAG_CLASS, _ref(obj, path))
    if obj is Ellipsis:  # the "..." in tuple[int, ...]
        return CBORTag(TAG_ELLIPSIS, None)
    for registered_type, (registered_encode, _, module) in _REGISTRY.items():
        if isinstance(obj, registered_type):
            payload = _hook(functools.partial(registered_encode, obj), t, path)
            prepared = _prepare(payload, f"{path}.encode()", active)
            return CBORTag(TAG_REGISTERED, [_ref(registered_type, path), module, prepared])
    if isinstance(obj, enum.Enum):
        # The value, not the name: a Flag combination has no member name.
        return CBORTag(TAG_ENUM, [_ref(t, path), _prepare(obj.value, f"{path}.value", active)])
    if isinstance(obj, np.ndarray):
        if obj.dtype.hasobject or obj.dtype.fields is not None:
            raise TypeError(f"Cannot CBOR-encode ndarray of dtype {obj.dtype} at {path}")
        return CBORTag(TAG_NDARRAY, [obj.dtype.str, list(obj.shape), obj.tobytes()])
    if isinstance(obj, np.generic):
        if obj.dtype.hasobject:
            raise TypeError(f"Cannot CBOR-encode numpy object scalar at {path}")
        return CBORTag(TAG_NPSCALAR, [obj.dtype.str, obj.tobytes()])
    if isinstance(getattr(t, "msg_name", None), str) and callable(getattr(t, "lcm_encode", None)):
        # The class, not msg_name: a subclass that inherits its parent's msg_name
        # must still come back as the subclass.
        return CBORTag(TAG_DIMOSMSG, [_ref(t, path), _hook(obj.lcm_encode, t, path)])
    if isinstance(obj, tuple) and hasattr(t, "_fields"):
        return CBORTag(TAG_NAMEDTUPLE, [_ref(t, path), _prepare_items(obj, path, active)])
    if dataclasses.is_dataclass(obj):
        fields = {
            f.name: _prepare(getattr(obj, f.name), f"{path}.{f.name}", active)
            for f in dataclasses.fields(obj)
        }
        return CBORTag(TAG_DATACLASS, [_ref(t, path), fields])
    if isinstance(obj, BaseModel):
        fields = {
            name: _prepare(getattr(obj, name), f"{path}.{name}", active)
            for name in type(obj).model_fields
        }
        extra = _prepare_dict(obj.__pydantic_extra__ or {}, path, active)
        return CBORTag(TAG_PYDANTIC, [_ref(t, path), fields, sorted(obj.model_fields_set), extra])
    if t is set:
        return CBORTag(TAG_SET, _prepare_items(obj, path, active))
    if t is frozenset:
        return CBORTag(TAG_FROZENSET, _prepare_items(obj, path, active))
    if isinstance(obj, pathlib.PurePath):
        return CBORTag(TAG_PATH, str(obj))
    if isinstance(obj, functools.partial):
        func = _prepare(obj.func, f"{path}.func", active)
        args = _prepare_items(obj.args, f"{path}.args", active)
        keywords = _prepare_dict(obj.keywords, f"{path}.keywords", active)
        return CBORTag(TAG_PARTIAL, [func, args, keywords])
    if isinstance(obj, (types.FunctionType, types.BuiltinFunctionType)):
        return CBORTag(TAG_FUNCTION, _ref(obj, path))
    raise TypeError(
        f"Cannot CBOR-encode object of type {t.__module__}.{t.__qualname__} at {path}: "
        "implement cbor_encode()/cbor_decode() on the class or call "
        "dimos.protocol.encode.cbor.register_codec()"
    )


def _hook(call: Callable[[], Any], t: type, path: str) -> Any:
    """Run an encode hook; whatever it raises becomes a TypeError that names the path."""
    try:
        return call()
    except Exception as e:
        raise TypeError(
            f"Cannot CBOR-encode object of type {t.__module__}.{t.__qualname__} at {path}: {e}"
        ) from e


def _prepare_type_form(obj: Any, path: str, active: set[int]) -> Any:
    """Generic aliases, unions and typing special forms.

    typing's spellings are normalized to the builtin ones: ``typing.List[int]`` arrives
    as ``list[int]`` and ``Optional[X]`` as ``X | None`` (they compare equal).
    """
    if obj is typing.Any:  # not a class on Python 3.10
        return CBORTag(TAG_CLASS, "typing:Any")
    origin, args = typing.get_origin(obj), typing.get_args(obj)
    if origin is typing.Union or origin is types.UnionType:
        return CBORTag(TAG_UNION, _prepare_items(args, path, active))
    if origin is typing.Literal:
        return CBORTag(TAG_LITERAL, _prepare_items(args, path, active))
    if isinstance(origin, type):
        prepared_origin = _prepare(origin, f"{path}.__origin__", active)
        return CBORTag(TAG_GENERIC_ALIAS, [prepared_origin, _prepare_items(args, path, active)])
    raise TypeError(f"Cannot CBOR-encode {obj!r} at {path}: unsupported typing form")


def _prepare_items(items: Any, path: str, active: set[int]) -> list[Any]:
    return [_prepare(item, f"{path}[{i}]", active) for i, item in enumerate(items)]


def _prepare_dict(obj: dict[Any, Any], path: str, active: set[int]) -> Any:
    if all(type(key) in _PRIMITIVES for key in obj):
        return {key: _prepare(value, f"{path}[{key!r}]", active) for key, value in obj.items()}
    # Keys of other types (tuples, enums, str subclasses) travel as a list of pairs so
    # their type survives; cbor2 would coerce or refuse them as map keys.
    pairs = [
        [_prepare(key, f"{path}.keys()", active), _prepare(value, f"{path}[{key!r}]", active)]
        for key, value in obj.items()
    ]
    return CBORTag(TAG_DICT, pairs)


def _ref(obj: Any, path: str) -> str:
    """``"module:qualname"`` of a class or function that the receiver can import again."""
    if obj is type(None):  # claims builtins as its module, which does not export it
        return "types:NoneType"
    module = getattr(obj, "__module__", None)
    qualname = getattr(obj, "__qualname__", None)
    if not isinstance(module, str) or not isinstance(qualname, str):
        raise TypeError(f"Cannot CBOR-encode {obj!r} at {path}: it has no module path")
    found = _lookup(sys.modules.get(module), qualname)
    # Identity for functions, as pickle does. A class only has to be found under its
    # name: after importlib.reload the module holds a replacement class object, and
    # the receiver resolves by name anyway.
    if found is not obj and not (isinstance(obj, type) and isinstance(found, type)):
        raise TypeError(
            f"Cannot CBOR-encode {obj!r} at {path}: it is not importable as "
            f"{module}.{qualname} (a lambda, a local definition or an unassigned runtime "
            "object); define it at module level"
        )
    return f"{module}:{qualname}"


def _lookup(target: Any, qualname: str) -> Any:
    for part in qualname.split("."):
        target = getattr(target, part, None)
        if target is None:
            return None
    return target


def _import(module_name: str, what: str) -> types.ModuleType:
    try:
        return importlib.import_module(module_name)
    except ImportError as e:
        raise CborDecodeError(f"cannot import {module_name!r} for {what}: {e}") from e


def _resolve(ref: str) -> Any:
    module_name, _, qualname = ref.partition(":")
    target = _lookup(_import(module_name, repr(ref)), qualname)
    if target is None:
        raise CborDecodeError(f"{ref!r} does not resolve to an object")
    return target


def _resolve_class(ref: str, base: type) -> Any:
    cls = _resolve(ref)
    if not (isinstance(cls, type) and issubclass(cls, base)):
        raise CborDecodeError(f"{ref!r} is not a {base.__name__} subclass")
    return cls


def _is_type_form(value: Any) -> bool:
    """A class, ``Any`` or a parameterized form: what ``|`` and ``[]`` accept."""
    return isinstance(value, type) or value is typing.Any or typing.get_origin(value) is not None


def _decode_tuple(value: Any, _: bool) -> Any:
    return tuple(value)


def _decode_namedtuple(value: Any, _: bool) -> Any:
    ref, items = value
    cls = _resolve_class(ref, tuple)
    if not hasattr(cls, "_fields"):
        raise CborDecodeError(f"{ref!r} is not a NamedTuple")
    return cls._make(items)


def _decode_set(value: Any, _: bool) -> Any:
    return set(value)


def _decode_frozenset(value: Any, _: bool) -> Any:
    return frozenset(value)


def _decode_ndarray(value: Any, _: bool) -> Any:
    dtype, shape, data = value
    # frombuffer on the decoded bytes is read-only; the copy makes it writable.
    return np.frombuffer(data, dtype=np.dtype(dtype)).reshape(shape).copy()


def _decode_npscalar(value: Any, _: bool) -> Any:
    dtype, data = value
    return np.frombuffer(data, dtype=np.dtype(dtype))[0]


def _decode_enum(value: Any, _: bool) -> Any:
    ref, member_value = value
    return _resolve_class(ref, enum.Enum)(member_value)


def _decode_dimosmsg(value: Any, _: bool) -> Any:
    ref, data = value
    lcm_decode = getattr(_resolve(ref), "lcm_decode", None)
    if not callable(lcm_decode):
        raise CborDecodeError(f"{ref!r} has no lcm_decode()")
    return lcm_decode(data)


def _decode_dataclass(value: Any, _: bool) -> Any:
    ref, fields = value
    cls = _resolve(ref)
    if not (isinstance(cls, type) and dataclasses.is_dataclass(cls)):
        raise CborDecodeError(f"{ref!r} is not a dataclass")
    # No __init__ or __post_init__: the sender already normalized the values, and this
    # also handles frozen classes and init=False fields. Only fields come back; see the
    # module docstring for derived state.
    obj = cls.__new__(cls)
    for f in dataclasses.fields(cls):
        if f.name not in fields:
            raise CborDecodeError(f"{ref!r} payload has no field {f.name!r}")
        object.__setattr__(obj, f.name, fields[f.name])
    return obj


def _decode_pydantic(value: Any, _: bool) -> Any:
    ref, fields, fields_set, extra = value
    cls = _resolve_class(ref, BaseModel)
    # model_construct skips validation (the sender validated) and keeps model_fields_set.
    # It also keeps BaseSettings from re-reading the environment on the receiving side.
    return cls.model_construct(_fields_set=set(fields_set), **fields, **extra)


def _decode_path(value: Any, _: bool) -> Any:
    return pathlib.Path(value)


def _decode_class(value: Any, _: bool) -> Any:
    cls = _resolve(value)
    if not (isinstance(cls, type) or cls is typing.Any):  # Any is not a class on 3.10
        raise CborDecodeError(f"{value!r} is not a class")
    return cls


def _decode_function(value: Any, _: bool) -> Any:
    func = _resolve(value)
    if not callable(func):
        raise CborDecodeError(f"{value!r} is not callable")
    return func


def _decode_partial(value: Any, _: bool) -> Any:
    func, args, keywords = value
    return functools.partial(func, *args, **keywords)


def _decode_custom(value: Any, _: bool) -> Any:
    ref, payload = value
    custom_decode = getattr(_resolve(ref), "cbor_decode", None)
    if not callable(custom_decode):
        raise CborDecodeError(f"{ref!r} has no cbor_decode()")
    return custom_decode(payload)


def _decode_registered(value: Any, _: bool) -> Any:
    type_ref, module_name, payload = value
    # Importing the registering module runs its register_codec() call; the decoder
    # itself comes from this process's registry, never from the payload.
    _import(module_name, f"the codec of {type_ref!r}")
    entry = _REGISTRY.get(_decode_class(type_ref, False))
    if entry is None:
        raise CborDecodeError(f"{type_ref!r} has no codec registered by {module_name!r}")
    return entry[1](payload)


def _decode_dict(value: Any, _: bool) -> Any:
    return dict(value)


def _decode_generic_alias(value: Any, _: bool) -> Any:
    origin, args = value
    if not isinstance(origin, type):
        raise CborDecodeError(f"{origin!r} is not a class and cannot be parameterized")
    # Subscripting rather than types.GenericAlias(): a Generic subclass then gives back
    # its own typing alias, and list[(int,)] is list[int].
    generic: Any = origin  # a plain `type` is not indexable for mypy
    return generic[tuple(args)]


def _decode_union(value: Any, _: bool) -> Any:
    if not value or not all(_is_type_form(member) for member in value):
        raise CborDecodeError(f"{value!r} is not a union of types")
    return functools.reduce(operator.or_, value)


def _decode_literal(value: Any, _: bool) -> Any:
    return typing.Literal[tuple(value)]


def _decode_ellipsis(_value: Any, _: bool) -> Any:
    return Ellipsis


def _reject_unknown_tag(tag: CBORTag, _: bool) -> Any:
    raise CborDecodeError(f"unknown CBOR tag {tag.tag}")


_DECODERS: dict[int, Callable[[Any, bool], Any]] = {
    TAG_TUPLE: _decode_tuple,
    TAG_NAMEDTUPLE: _decode_namedtuple,
    TAG_SET: _decode_set,
    TAG_FROZENSET: _decode_frozenset,
    TAG_NDARRAY: _decode_ndarray,
    TAG_NPSCALAR: _decode_npscalar,
    TAG_ENUM: _decode_enum,
    TAG_DIMOSMSG: _decode_dimosmsg,
    TAG_DATACLASS: _decode_dataclass,
    TAG_PYDANTIC: _decode_pydantic,
    TAG_PATH: _decode_path,
    TAG_CLASS: _decode_class,
    TAG_FUNCTION: _decode_function,
    TAG_PARTIAL: _decode_partial,
    TAG_CUSTOM: _decode_custom,
    TAG_REGISTERED: _decode_registered,
    TAG_DICT: _decode_dict,
    TAG_GENERIC_ALIAS: _decode_generic_alias,
    TAG_UNION: _decode_union,
    TAG_LITERAL: _decode_literal,
    TAG_ELLIPSIS: _decode_ellipsis,
}
