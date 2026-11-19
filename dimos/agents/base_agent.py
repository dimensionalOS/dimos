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

from typing import Any, ClassVar, Generic, Protocol, TypeVar, get_args, get_origin

from langchain_core.messages.base import BaseMessage
from pydantic import ConfigDict, TypeAdapter, ValidationError
from pydantic.errors import PydanticUserError

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out

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


def _declared(cls: type, stream: str) -> bool:
    """Whether cls or an agent it extends wrote this stream's annotation by hand."""
    for klass in cls.__mro__:
        if klass.__name__ == "BaseAgent" and klass.__module__ == __name__:
            return False
        if stream in klass.__dict__.get("__annotations__", {}):
            return stream not in klass.__dict__.get("_typed_streams", ())
    return False


class BaseAgent(Module, Generic[I, O]):
    """Base for agents: one inference call and the streams that feed it.

    A subclass names its types, ``class Agent(BaseAgent[HumanMessage, AIMessage])``. They are
    kept on the class as ``input_type`` and ``output_type``, and the class gets the streams
    ``agent_input: In[Input]`` and ``agent_output: Out[Output]`` unless it declares a stream
    of that name itself. An input with several parts is one dataclass.
    """

    input_type: ClassVar[Any]
    output_type: ClassVar[Any]
    _typed_streams: ClassVar[frozenset[str]] = frozenset()

    agent_input: In[Any]
    agent_output: Out[Any]
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
        cls.input_type, cls.output_type = types
        if not unbound:
            in_stream: Any = In
            out_stream: Any = Out
            streams = {"agent_input": in_stream[types[0]], "agent_output": out_stream[types[1]]}
            typed = {name: hint for name, hint in streams.items() if not _declared(cls, name)}
            cls.__annotations__ = {**cls.__dict__.get("__annotations__", {}), **typed}
            cls._typed_streams = frozenset(typed)
        super().__init_subclass__(**kwargs)

    @classmethod
    def accepts(cls, x: object) -> bool:
        """Whether x is a valid input for this agent."""
        return _matches(cls.input_type, x)

    @rpc
    def agent_transports(self) -> dict[str, Any]:
        """The transport each agent stream is on, for a client outside the blueprint."""
        streams = ("agent_input", "agent_output", "agent", "agent_idle")
        transports = {name: getattr(self, name).transport for name in streams}
        return {name: transport for name, transport in transports.items() if transport is not None}

    @rpc
    def infer(self, x: I) -> O:
        """One inference: the agent's input in, its output back."""
        raise NotImplementedError
