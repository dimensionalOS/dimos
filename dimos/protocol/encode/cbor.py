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
numpy arrays and scalars, Enum, DimosMsg (as its class plus LCM bytes), dataclasses, pydantic models,
Path, classes (including ``list[float]``-style generics and ``X | Y`` unions), module-level
functions, functools.partial, and any class that implements
``cbor_encode``/``cbor_decode`` (see ``CborEncodable``). Anything else raises
``TypeError`` at encode time. There is no pickle-style fallback.

Classes and functions are sent by reference as ``"module:qualname"`` and must be
importable under that name on the receiving side.
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

_PRIMITIVES = (type(None), bool, int, float, str, bytes)


class CborDecodeError(ValueError):
    """The payload is not valid CBOR for this codec."""


class CborEncodable(Protocol):
    """Opt-in serialization for classes the generic rules do not cover.

    ``cbor_encode`` returns any encodable value; ``cbor_decode`` rebuilds an instance
    from it. The returned object may be a stand-in of another class (a stream handle
    decodes to its remote twin), which is why ``cbor_decode`` is typed as ``Any``.
    """

    def cbor_encode(self) -> Any: ...

    @classmethod
    def cbor_decode(cls, data: Any) -> Any: ...


# type -> (encode, "module:qualname" of the decode function)
_REGISTRY: dict[type, tuple[Callable[[Any], Any], str]] = {}


def register_codec(type_: type, encode: Callable[[Any], Any], decode: Callable[[Any], Any]) -> None:
    """Make instances of ``type_`` (and subclasses) encodable without touching the class.

    ``decode`` must be a module-level function: the payload names it by reference so the
    decoding process imports the registering module on demand.
    """
    _REGISTRY[type_] = (encode, _ref(decode, repr(decode)))


def encode(obj: Any) -> bytes:
    """Encode ``obj`` to CBOR bytes. Raises ``TypeError`` for unsupported values."""
    return cbor2.dumps(_prepare(obj, "value", set()))


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
    """Map an object graph to values cbor2 encodes natively (CBORTag for our tags)."""
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
        payload = _prepare(custom_encode(obj), f"{path}.cbor_encode()", active)
        return CBORTag(TAG_CUSTOM, [_ref(t, path), payload])
    # Generic aliases and unions come before the class check: on Python 3.10
    # isinstance(list[int], type) is True.
    if isinstance(obj, types.GenericAlias):
        origin = _prepare(obj.__origin__, f"{path}.__origin__", active)
        return CBORTag(TAG_GENERIC_ALIAS, [origin, _prepare_items(obj.__args__, path, active)])
    if isinstance(obj, types.UnionType):
        return CBORTag(TAG_UNION, _prepare_items(obj.__args__, path, active))
    if isinstance(obj, type):
        return CBORTag(TAG_CLASS, _ref(obj, path))
    if obj is Ellipsis:  # the "..." in tuple[int, ...]
        return CBORTag(TAG_ELLIPSIS, None)
    for registered_type, (registered_encode, decode_ref) in _REGISTRY.items():
        if isinstance(obj, registered_type):
            payload = _prepare(registered_encode(obj), f"{path}.encode()", active)
            return CBORTag(TAG_REGISTERED, [decode_ref, payload])
    if isinstance(obj, enum.Enum):
        return CBORTag(TAG_ENUM, [_ref(t, path), obj.name])
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
        return CBORTag(TAG_DIMOSMSG, [_ref(t, path), obj.lcm_encode()])
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


def _resolve(ref: str) -> Any:
    module_name, _, qualname = ref.partition(":")
    try:
        module = importlib.import_module(module_name)
    except ImportError as e:
        raise CborDecodeError(f"cannot import {module_name!r} for {ref!r}: {e}") from e
    target = _lookup(module, qualname)
    if target is None:
        raise CborDecodeError(f"{ref!r} does not resolve to an object")
    return target


def _resolve_class(ref: str, base: type) -> Any:
    cls = _resolve(ref)
    if not (isinstance(cls, type) and issubclass(cls, base)):
        raise CborDecodeError(f"{ref!r} is not a {base.__name__} subclass")
    return cls


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
    ref, name = value
    return _resolve_class(ref, enum.Enum)[name]


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
    # also handles frozen classes and init=False fields.
    obj = cls.__new__(cls)
    for f in dataclasses.fields(cls):
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
    if not isinstance(cls, type):
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
    ref, payload = value
    return _resolve(ref)(payload)


def _decode_dict(value: Any, _: bool) -> Any:
    return dict(value)


def _decode_generic_alias(value: Any, _: bool) -> Any:
    origin, args = value
    return types.GenericAlias(origin, tuple(args))


def _decode_union(value: Any, _: bool) -> Any:
    return functools.reduce(operator.or_, value)


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
    TAG_ELLIPSIS: _decode_ellipsis,
}
