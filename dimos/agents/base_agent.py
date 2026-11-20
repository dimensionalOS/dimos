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

import dataclasses
from enum import Enum
from pathlib import Path
from typing import (
    Any,
    ClassVar,
    Generic,
    Literal,
    Protocol,
    TypeVar,
    get_args,
    get_origin,
    get_type_hints,
    is_typeddict,
)

from langchain_core.messages.base import BaseMessage
import numpy as np
from pydantic import BaseModel, ConfigDict, TypeAdapter, ValidationError
from pydantic.errors import PydanticUserError

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.protocol.encode import cbor

I = TypeVar("I")
O = TypeVar("O")
I_contra = TypeVar("I_contra", contravariant=True)
O_co = TypeVar("O_co", covariant=True)


class Infers(Protocol[I_contra, O_co]):
    """Anything with one typed inference call: an agent module or a plain model."""

    def infer(self, x: I_contra) -> O_co: ...


def _resolve(cls: type, target: type) -> tuple[Any, ...] | None:
    """target's type arguments as cls binds them, following TypeVars through generic bases."""
    for base in getattr(cls, "__orig_bases__", ()):
        origin = get_origin(base) or base
        if origin is target:
            return get_args(base)
        if isinstance(origin, type) and issubclass(origin, target):
            inner = _resolve(origin, target)
            if inner is None:
                continue
            binding = dict(zip(getattr(origin, "__parameters__", ()), get_args(base), strict=False))
            return tuple(binding.get(arg, arg) for arg in inner)
    return None


def _matches(tp: Any, value: Any) -> bool:
    if isinstance(tp, type):
        try:
            return isinstance(value, tp)
        except TypeError:  # TypedDict and other classes that refuse isinstance
            pass
    try:
        adapter = TypeAdapter(tp, config=ConfigDict(arbitrary_types_allowed=True))
    except PydanticUserError:  # the type carries its own config
        adapter = TypeAdapter(tp)
    try:
        adapter.validate_python(value, strict=True)
    except ValidationError:
        return False
    return True


_PLAIN = (type(None), bool, int, float, str, bytes, Path)


def _unsendable(tp: Any, seen: frozenset[int] = frozenset()) -> str | None:
    """Why values of tp cannot cross RPC or a transport (see protocol/encode/cbor.py), or None.

    ``Any`` and ``object`` pass: what they hold is only known when a value is sent.
    """
    if tp is Any or tp is object or tp in _PLAIN or id(tp) in seen:
        return None
    origin = get_origin(tp)
    if origin is Literal:
        return None
    if origin is not None:
        if isinstance(origin, type) and issubclass(origin, np.ndarray):
            return None
        for arg in get_args(tp):
            if arg is not Ellipsis and (reason := _unsendable(arg, seen)) is not None:
                return reason
        return None
    if not isinstance(tp, type):
        return f"{tp!r} is not a class or a typing form of classes"
    if "<locals>" in tp.__qualname__:
        return f"{tp.__qualname__} is defined inside a function, so the receiver cannot import it"
    if issubclass(tp, (Enum, BaseModel, np.ndarray, np.generic, tuple, list, dict, set, frozenset)):
        fields: dict[str, Any] = get_type_hints(tp) if is_typeddict(tp) else {}
    elif dataclasses.is_dataclass(tp):
        fields = get_type_hints(tp)
    elif (
        callable(getattr(tp, "lcm_encode", None))
        or callable(getattr(tp, "cbor_encode", None))
        or issubclass(tp, tuple(cbor._REGISTRY))
    ):
        return None
    else:
        return (
            f"{tp.__module__}.{tp.__qualname__} is a plain class; make it a dataclass, a "
            "pydantic model or a DimOS message, or give it cbor_encode/cbor_decode"
        )
    for name, hint in fields.items():
        if (reason := _unsendable(hint, seen | {id(tp)})) is not None:
            return f"{tp.__qualname__}.{name}: {reason}"
    return None


class BaseAgent(Module, Generic[I, O]):
    """Base for agents: one typed inference call.

    A subclass names its types, ``class Agent(BaseAgent[HumanMessage, AIMessage])``, and they
    are kept on the class as ``input_type`` and ``output_type``. An input with several parts
    is one dataclass. An agent declares the streams it reads and writes itself and passes
    what arrives to ``infer``.
    """

    input_type: ClassVar[Any]
    output_type: ClassVar[Any]

    agent: Out[BaseMessage]
    agent_idle: Out[bool]

    def __init_subclass__(cls, **kwargs: Any) -> None:
        types = _resolve(cls, BaseAgent)
        if types is None:
            raise TypeError(
                f"{cls.__name__} must name its input and output types, "
                f"e.g. class {cls.__name__}(BaseAgent[str, str])"
            )
        unbound = [tp for tp in types if isinstance(tp, TypeVar)]
        # Generic.__init_subclass__ has not run yet, so __parameters__ is still the parent's.
        generic = any(
            isinstance(arg, TypeVar)
            for base in cls.__dict__.get("__orig_bases__", ())
            for arg in get_args(base)
        )
        if unbound and not generic:
            raise TypeError(f"{cls.__name__} leaves {unbound} of BaseAgent unbound")
        for tp in types if not unbound else ():
            if (reason := _unsendable(tp)) is not None:
                raise TypeError(
                    f"{cls.__name__} names a type that cannot be sent over RPC or a transport: "
                    f"{reason}"
                )
        cls.input_type, cls.output_type = types
        super().__init_subclass__(**kwargs)

    @classmethod
    def accepts(cls, x: object) -> bool:
        """Whether x is a valid input for this agent."""
        return _matches(cls.input_type, x)

    @rpc
    def agent_transports(self) -> dict[str, Any]:
        """The transport of each chat stream this agent has, for a client outside the blueprint."""
        streams = ("agent_input", "agent_output", "agent", "agent_idle")
        transports = {
            name: getattr(getattr(self, name, None), "transport", None) for name in streams
        }
        return {name: transport for name, transport in transports.items() if transport is not None}

    @rpc
    def infer(self, x: I) -> O:
        """One inference: the agent's input in, its output back."""
        raise NotImplementedError
