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
    TypeVar,
    get_args,
    get_origin,
    is_typeddict,
)

from langchain_core.messages.base import BaseMessage
import numpy as np
from pydantic import BaseModel, ConfigDict, TypeAdapter, ValidationError

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out

I = TypeVar("I")
O = TypeVar("O")
_SENDABLE: tuple[type, ...] = (bool, int, float, str, bytes, tuple, list, dict, set, frozenset)
_SENDABLE += (type(None), Enum, Path, BaseModel, np.ndarray, np.generic)


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
    try:
        return isinstance(value, tp)
    except TypeError:  # dict[str, float], a TypedDict: isinstance refuses them
        config = None if is_typeddict(tp) else ConfigDict(arbitrary_types_allowed=True)
        try:
            TypeAdapter(tp, config=config).validate_python(value, strict=True)
        except ValidationError:
            return False
        return True


def _sendable(tp: Any) -> bool:
    """Whether values of tp can cross RPC or a transport (see protocol/encode/cbor.py)."""
    if args := get_args(tp):
        return all(_sendable(arg) for arg in args if arg is not Ellipsis)
    if not isinstance(tp, type) or tp in (Any, object) or dataclasses.is_dataclass(tp):
        return True
    return issubclass(tp, _SENDABLE) or hasattr(tp, "lcm_encode") or hasattr(tp, "cbor_encode")


class BaseAgent(Module, Generic[I, O]):
    """Base for agents: one inference call and the chat streams.

    A subclass names its types, ``class Agent(BaseAgent[HumanMessage, AIMessage])``, and they are
    kept on the class as ``input_type`` and ``output_type``. An input with several
    parts is one dataclass.
    """

    input_type: ClassVar[Any]
    output_type: ClassVar[Any]

    agent: Out[BaseMessage]
    agent_idle: Out[bool]

    def __init_subclass__(cls, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        types = _resolve(cls, BaseAgent)
        if types is None:
            raise TypeError(
                f"{cls.__name__} must name its input and output types, "
                f"e.g. class {cls.__name__}(BaseAgent[str, str])"
            )
        unbound = [tp for tp in types if isinstance(tp, TypeVar)]
        if unbound and not getattr(cls, "__parameters__", ()):
            raise TypeError(f"{cls.__name__} leaves {unbound} of BaseAgent unbound")
        for tp in types:
            if not _sendable(tp):
                raise TypeError(
                    f"{cls.__name__}: {tp!r} cannot be sent over RPC or a transport; use a "
                    "dataclass, a pydantic model or a DimOS message"
                )
        cls.input_type, cls.output_type = types

    @classmethod
    def accepts(cls, x: object) -> bool:
        """Whether x is a valid input for this agent."""
        return _matches(cls.input_type, x)

    @rpc
    def agent_types(self) -> tuple[Any, Any]:
        """(input_type, output_type), for a caller that only holds a proxy to this agent."""
        return self.input_type, self.output_type

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
