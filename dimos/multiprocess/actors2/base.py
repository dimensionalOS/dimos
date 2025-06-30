# Copyright 2025 Dimensional Inc.
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

from __future__ import annotations

import enum
import inspect
from typing import Any, Callable, Generic, TypeVar, get_args, get_origin, get_type_hints

import dimos.multiprocess.actors2.colors as colors

T = TypeVar("T")


def rpc(fn):
    fn.__rpc__ = True
    return fn


class State(enum.Enum):
    DORMANT = "dormant"
    READY = "ready"


class Stream(Generic[T]):
    def __init__(self, type: type[T], name: str = "In", owner: Any = None):
        self.type = type
        self.name = name
        self.owner = owner

    def __set_name__(self, owner, n):
        self.name = n

    @property
    def type_name(self) -> str:
        return getattr(self.type, "__name__", repr(self.type))

    def __str__(self):
        return self.color()(f"{self.name}[{self.type_name}]")

    def color(self) -> Callable[[str], str]:
        if self.state == State.DORMANT:
            return colors.orange
        if self.state == State.READY:
            return colors.green

    @property
    def state(self) -> State:
        if not self.owner:
            return State.DORMANT
        return State.READY


class In(Stream[T]):
    def __str__(self):
        return f"IN {super().__str__()}"


class Out(Stream[T]):
    def __str__(self):
        return f"OUT {super().__str__()}"


class Module:
    @classmethod
    def io(c):
        def boundary_iter(iterable, first, middle, last):
            l = list(iterable)
            for idx, sd in enumerate(l):
                if idx == len(l) - 1:
                    yield last + sd
                elif idx == 0:
                    yield first + sd
                else:
                    yield middle + sd

        def box(name):
            top = "┌┴" + "─" * (len(name) + 1) + "┐"
            middle = f"│ {name} │"
            bottom = "└┬" + "─" * (len(name) + 1) + "┘"
            return f"{top}\n{middle}\n{bottom}"

        inputs = list(boundary_iter(map(str, c.inputs.values()), " ┌─ ", " ├─ ", " ├─ "))

        rpcs = []
        for n, fn in c.rpcs.items():
            sig = inspect.signature(fn)
            hints = get_type_hints(fn, include_extras=True)
            param_strs: list[str] = []
            for pname, _ in sig.parameters.items():
                if pname in ("self", "cls"):
                    continue
                ann = hints.get(pname, Any)
                ann_name = getattr(ann, "__name__", repr(ann))
                param_strs.append(f"{pname}: {ann_name}")
            ret_ann = hints.get("return", Any)
            ret_name = getattr(ret_ann, "__name__", repr(ret_ann))
            rpcs.append(f"{n}({', '.join(param_strs)}) → {ret_name}")

        rpcs = list(boundary_iter(rpcs, " ├─ ", " ├─ ", " └─ "))

        outputs = list(
            boundary_iter(map(str, c.outputs.values()), " ├─ ", " ├─ ", " ├─ " if rpcs else " └─ ")
        )

        if rpcs:
            rpcs = [" │"] + rpcs

        return "\n".join(inputs + [box(c.__name__)] + outputs + rpcs)


def module(cls: type) -> type:
    cls.inputs = dict(getattr(cls, "inputs", {}))
    cls.outputs = dict(getattr(cls, "outputs", {}))
    cls.rpcs = dict(getattr(cls, "rpcs", {}))

    cls_type_hints = get_type_hints(cls, include_extras=True)

    for n, ann in cls_type_hints.items():
        origin = get_origin(ann)
        # print(n, ann, origin)

        if origin is Out:
            inner_type, *_ = get_args(ann) or (Any,)
            md = Out(inner_type, n)
            cls.outputs[n] = md
            setattr(cls, n, md)

    for n, a in cls.__dict__.items():
        if callable(a) and getattr(a, "__rpc__", False):
            cls.rpcs[n] = a

    sig = inspect.signature(cls.__init__)
    type_hints = get_type_hints(cls.__init__, include_extras=True)

    for name, _ in sig.parameters.items():
        if name == "self":
            continue

        md = None
        ann = type_hints.get(name)
        origin = get_origin(ann)

        if origin is In:
            inner_type, *_ = get_args(ann) or (Any,)
            md = In(inner_type, name)

        if md is not None:
            cls.inputs[name] = md

    original_init = cls.__init__

    def init_override(self, *args, **kwargs):
        for name, out in self.outputs.items():
            out.owner = self
        for name, inp in self.inputs.items():
            inp.owner = self

        return original_init(self, *args, **kwargs)

    cls.__init__ = init_override

    return cls
