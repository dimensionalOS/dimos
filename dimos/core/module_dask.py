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

from typing import (
    Any,
    Callable,
    List,
    get_args,
    get_origin,
    get_type_hints,
)

from dask.distributed import Actor

from dimos.core.core import In, Out, RemoteIn, RemoteOut, T, Transport


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

    # called from remote
    def set_transport(self, stream_name: str, transport: Transport):
        stream = getattr(self, stream_name, None)
        if not stream:
            raise ValueError(f"{stream_name} not found in {self.__class__.__name__}")

        if not isinstance(stream, Out) and not isinstance(stream, In):
            raise TypeError(f"Output {stream_name} is not a valid stream")

        stream._transport = transport
        return True

    # called from remote
    def connect_stream(self, input_name: str, remote_stream: RemoteOut[T]):
        input_stream = getattr(self, input_name, None)
        if not input_stream:
            raise ValueError(f"{input_name} not found in {self.__class__.__name__}")
        if not isinstance(input_stream, In):
            raise TypeError(f"Input {input_name} is not a valid stream")
        input_stream.connection = remote_stream

    def dask_receive_msg(self, input_name: str, msg: Any):
        getattr(self, input_name).transport.dask_receive_msg(msg)

    def dask_register_subscriber(self, output_name: str, subscriber: RemoteIn[T]):
        getattr(self, output_name).transport.dask_register_subscriber(subscriber)

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
        return [name for name in dir(self) if hasattr(getattr(self, name), "__rpc__")]

    def io(self) -> str:
        def _box(name: str) -> str:
            return [
                "┌┴" + "─" * (len(name) + 1) + "┐",
                f"│ {name} │",
                "└┬" + "─" * (len(name) + 1) + "┘",
            ]

        ret = [
            *(f" ├─ {stream}" for stream in self.inputs.values()),
            *_box(self.__class__.__name__),
            *(f" ├─ {stream}" for stream in self.outputs.values()),
        ]

        return "\n".join(ret)
