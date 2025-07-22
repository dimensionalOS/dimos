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
    get_args,
    get_origin,
    get_type_hints,
)

from dask.distributed import Actor, get_worker

from dimos.core import colors
from dimos.core.core import In, Out, RemoteIn, RemoteOut, T, Transport, rpc
from dimos.protocol.rpc.lcmrpc import LCMRPC


class RPCWrapper:
    def __init__(self, actor_instance, actor_class):
        self.rpc = LCMRPC()
        self.actor_class = actor_class
        self.remote_name = actor_class.__name__
        self.actor_instance = actor_instance
        self.rpcs = actor_class.rpcs.keys()
        self.rpc.start()

    def __reduce__(self):
        # Return the class and the arguments needed to reconstruct the object
        return (
            self.__class__,
            (self.actor_instance, self.actor_class),
        )

    # passthrough
    def __getattr__(self, name: str):
        # Check if accessing a known safe attribute to avoid recursion
        if name in {
            "__class__",
            "__init__",
            "__dict__",
            "__getattr__",
            "rpcs",
            "remote_name",
            "remote_instance",
            "actor_instance",
        }:
            raise AttributeError(f"{name} is not found.")

        if name in self.rpcs:
            return lambda *args: self.rpc.call_sync(f"{self.remote_name}/{name}", args)

        val = self.actor_instance.__getattr__(name)
        # check if val is function
        if callable(val):
            raise AttributeError(
                f"Cannot access method {self.remote_name}.{name}() directly. on Use @rpc decorator on this method if attending to call remotely."
            )
        return val


class ModuleBase:
    _module_init_called: bool = False

    def __init__(self, *args, **kwargs):
        self._module_init_called = True
        try:
            get_worker()
            self.rpc = LCMRPC()
            self.rpc.serve_module_rpc(self)
            self.rpc.start()
        except ValueError:
            return

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

    @classmethod
    @property
    def rpcs(cls) -> dict[str, Callable]:
        return {
            name: getattr(cls, name)
            for name in dir(cls)
            if not name.startswith("_")
            and name != "rpcs"  # Exclude the rpcs property itself to prevent recursion
            and callable(getattr(cls, name, None))
            and hasattr(getattr(cls, name), "__rpc__")
        }

    @rpc
    def io(self) -> str:
        def _box(name: str) -> str:
            return [
                f"┌┴" + "─" * (len(name) + 1) + "┐",
                f"│ {name} │",
                f"└┬" + "─" * (len(name) + 1) + "┘",
            ]

        # can't modify __str__ on a function like we are doing for I/O
        # so we have a separate repr function here
        def repr_rpc(fn: Callable) -> str:
            sig = inspect.signature(fn)
            # Remove 'self' parameter
            params = [p for name, p in sig.parameters.items() if name != "self"]

            # Format parameters with colored types
            param_strs = []
            for param in params:
                param_str = param.name
                if param.annotation != inspect.Parameter.empty:
                    type_name = getattr(param.annotation, "__name__", str(param.annotation))
                    param_str += ": " + colors.green(type_name)
                if param.default != inspect.Parameter.empty:
                    param_str += f" = {param.default}"
                param_strs.append(param_str)

            # Format return type
            return_annotation = ""
            if sig.return_annotation != inspect.Signature.empty:
                return_type = getattr(sig.return_annotation, "__name__", str(sig.return_annotation))
                return_annotation = " -> " + colors.green(return_type)

            return (
                "RPC " + colors.blue(fn.__name__) + f"({', '.join(param_strs)})" + return_annotation
            )

        ret = [
            *(f" ├─ {stream}" for stream in self.inputs.values()),
            *_box(self.__class__.__name__),
            *(f" ├─ {stream}" for stream in self.outputs.values()),
            " │",
            *(f" ├─ {repr_rpc(rpc)}" for rpc in self.rpcs.values()),
        ]

        return "\n".join(ret)


class DaskModule(ModuleBase):
    ref: Actor
    worker: int

    def __init__(self, *args, **kwargs):
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
        super().__init__(*args, **kwargs)

    def set_ref(self, ref) -> int:
        if not self._module_init_called:
            raise RuntimeError(
                f"Module must be initialized\nmake sure to call super().__init__() in your {self.__class__.__name__} constructor"
            )
        worker = get_worker()
        self.ref = ref
        self.worker = worker.name
        return worker.name

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


# global setting
Module = DaskModule
