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

import inspect
from typing import (
    Any,
    Generic,
    TypeVar,
    get_args,
    get_origin,
    get_type_hints,
)

T = TypeVar("T")


class StreamDef(Generic[T]):
    def __init__(self, type: type[T], direction: str = "in"):
        self.type = type
        self.direction = direction  # 'in' or 'out'
        self.name: str | None = None

    def __set_name__(self, owner, n):
        self.name = n

    def __get__(self, *_):
        raise AttributeError("metadata only")

    @property
    def type_name(self) -> str:
        getattr(self.type, "__name__", repr(self.type))


def rpc(fn):
    fn.__rpc__ = True
    return fn


class In(Generic[T]):
    def __init__(self, type: type[T], name: str = "In"):
        self.type = type
        self.name = name

    def __set_name__(self, owner, n):
        self.name = n

    def __get__(self, *_):
        raise AttributeError("metadata only")

    @property
    def type_name(self) -> str:
        return getattr(self.type, "__name__", repr(self.type))

    def __str__(self):
        return f"{self.name}[{self.type_name}]"


class Out(Generic[T]):
    def __init__(self, type: type[T], name: str = "Out"):
        self.type = type
        self.name = name

    def __set_name__(self, owner, n):
        self.name = n

    def __get__(self, *_):
        raise AttributeError("metadata only")

    @property
    def type_name(self) -> str:
        return getattr(self.type, "__name__", repr(self.type))

    def __str__(self):
        return f"{self.name}[{self.type_name}]"


# ── decorator with *type-based* input / output detection ────────────────────
def module(cls: type) -> type:
    cls.inputs = dict(getattr(cls, "inputs", {}))
    cls.outputs = dict(getattr(cls, "outputs", {}))
    cls.rpcs = dict(getattr(cls, "rpcs", {}))

    cls_type_hints = get_type_hints(cls, include_extras=True)

    for n, ann in cls_type_hints.items():
        origin = get_origin(ann)
        print(n, ann, origin)
        if origin is Out:
            inner_type, *_ = get_args(ann) or (Any,)
            md = Out(inner_type, n)
            cls.outputs[n] = md
            # make attribute accessible via instance / class
            setattr(cls, n, md)

    # RPCs
    for n, a in cls.__dict__.items():
        if callable(a) and getattr(a, "__rpc__", False):
            cls.rpcs[n] = a

    sig = inspect.signature(cls.__init__)
    type_hints = get_type_hints(cls.__init__, include_extras=True)

    for pname, param in sig.parameters.items():
        if pname == "self":
            continue

        md = None
        ann = type_hints.get(pname)
        origin = get_origin(ann)

        if origin is In:
            inner_type, *_ = get_args(ann) or (Any,)
            md = In(inner_type, pname)

        if md is not None:
            cls.inputs[pname] = md

    def _io_inner(c):
        def boundary_iter(iterable, first, middle, last):
            l = list(iterable)
            for idx, sd in enumerate(l):  # idx = 0,1,2…
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
            for pname, param in sig.parameters.items():
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

    setattr(cls, "io", classmethod(_io_inner))

    # instance method simply forwards to classmethod
    def _io_instance(self):
        return self.__class__.io()

    setattr(cls, "io_instance", _io_instance)

    return cls
