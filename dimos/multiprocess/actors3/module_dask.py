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

import inspect
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    List,
    Protocol,
    TypeVar,
    get_args,
    get_origin,
    get_type_hints,
)

from dask.distributed import Actor

from dimos.multiprocess.actors3.base import In, Out, RemoteIn


class Module:
    ref: Actor

    def __init__(self):
        self.ref = None

        for name, ann in get_type_hints(self, include_extras=True).items():
            origin = get_origin(ann)
            if origin is Out:
                inner, *_ = get_args(ann) or (Any,)
                stream = Out(inner, name, self)
                setattr(self, name, stream)
            elif origin is In:
                inner, *_ = get_args(ann) or (Any,)
                stream = In(inner, name, self)
                setattr(self, name, stream)

    def set_ref(self, ref):
        self.ref = ref

    def __str__(self):
        return f"{self.__class__.__name__}"

    @property
    def outputs(self) -> dict[str, Out]:
        return {
            name: s
            for name, s in self.__dict__.items()
            if isinstance(s, Out) and not name.startswith("_")
        }

    @property
    def inputs(self) -> dict[str, In]:
        return {
            name: s
            for name, s in self.__dict__.items()
            if isinstance(s, In) and not name.startswith("_")
        }

    @property
    def rpcs(self) -> List[Callable]:
        return [
            getattr(self, name)
            for name in dir(self)
            if callable(getattr(self, name)) and hasattr(getattr(self, name), "__rpc__")
        ]

    def io(self) -> str:
        def _box(name: str) -> str:
            return [
                "┌┴" + "─" * (len(name) + 1) + "┐",
                f"│ {name} │",
                "└┬" + "─" * (len(name) + 1) + "┘",
            ]

        ret = [
            *(f" ├─ {name:<16} {stream}" for name, stream in self.inputs.items()),
            *_box(self.__class__.__name__),
            *(f" ├─ {name:<16} {stream}" for name, stream in self.outputs.items()),
        ]

        return "\n".join(ret)
