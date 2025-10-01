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
import asyncio
import inspect
import threading
from dataclasses import dataclass
from typing import (
    Any,
    Callable,
    Optional,
    get_args,
    get_origin,
    get_type_hints,
)
from reactivex.disposable import CompositeDisposable

from dask.distributed import Actor, get_worker

from dimos.core import colors
from dimos.core.core import T, rpc
from dimos.core.stream import In, Out, RemoteIn, RemoteOut, Transport
from dimos.protocol.rpc import LCMRPC, RPCSpec
from dimos.protocol.service import Configurable
from dimos.protocol.skill.skill import SkillContainer
from dimos.protocol.tf import LCMTF, TFSpec


def get_loop() -> tuple[asyncio.AbstractEventLoop, Optional[threading.Thread]]:
    # we are actually instantiating a new loop here
    # to not interfere with an existing dask loop

    # try:
    #     # here we attempt to figure out if we are running on a dask worker
    #     # if so we use the dask worker _loop as ours,
    #     # and we register our RPC server
    #     worker = get_worker()
    #     if worker.loop:
    #         print("using dask worker loop")
    #         return worker.loop.asyncio_loop

    # except ValueError:
    #     ...

    try:
        running_loop = asyncio.get_running_loop()
        return running_loop, None
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        thr = threading.Thread(target=loop.run_forever, daemon=True)
        thr.start()
        return loop, thr


@dataclass
class ModuleConfig:
    rpc_transport: type[RPCSpec] = LCMRPC
    tf_transport: type[TFSpec] = LCMTF


class ModuleBase(Configurable[ModuleConfig], SkillContainer):
    _rpc: Optional[RPCSpec] = None
    _tf: Optional[TFSpec] = None
    _loop: Optional[asyncio.AbstractEventLoop] = None
    _loop_thread: Optional[threading.Thread]
    _disposables: CompositeDisposable

    default_config = ModuleConfig

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._loop, self._loop_thread = get_loop()
        self._disposables = CompositeDisposable()
        # we can completely override comms protocols if we want
        try:
            # here we attempt to figure out if we are running on a dask worker
            # if so we use the dask worker _loop as ours,
            # and we register our RPC server
            self.rpc = self.config.rpc_transport()
            self.rpc.serve_module_rpc(self)
            self.rpc.start()
        except ValueError:
            ...

    def _close_module(self):
        self._close_rpc()
        if hasattr(self, "_loop") and self._loop_thread:
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._loop = None
            self._loop_thread.join(timeout=2)
            self._loop_thread = None
        if hasattr(self, "_disposables"):
            self._disposables.dispose()

    def _close_rpc(self):
        # Using hasattr is needed because SkillCoordinator skips ModuleBase.__init__ and self.rpc is never set.
        if hasattr(self, "rpc") and self.rpc:
            self.rpc.stop()
            self.rpc = None

    @property
    def tf(self):
        if self._tf is None:
            self._tf = self.config.tf_transport()
        return self._tf

    @tf.setter
    def tf(self, value):
        import warnings

        warnings.warn(
            "tf is available on all modules. Call self.tf.start() to activate tf functionality. No need to assign it",
            UserWarning,
            stacklevel=2,
        )

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
