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


class BaseAgent(Module, Generic[I, O]):
    """Base for agents: one inference call and the chat streams.

    A subclass names its types, ``class Agent(BaseAgent[str, str])``, and they are
    kept on the class as ``input_type`` and ``output_type``. An input with several
    parts is one dataclass.
    """

    input_type: ClassVar[Any]
    output_type: ClassVar[Any]

    human_input: In[str]
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
        cls.input_type, cls.output_type = types

    @classmethod
    def accepts(cls, x: object) -> bool:
        """Whether x is a valid input for this agent."""
        return _matches(cls.input_type, x)

    @rpc
    def infer(self, x: I) -> O:
        """One inference: the agent's input in, its output back."""
        raise NotImplementedError
