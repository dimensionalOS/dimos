# Copyright 2025-2026 Dimensional Inc.
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
from collections.abc import Callable
from dataclasses import dataclass
from functools import partial
import sys
import threading
from typing import (
    TYPE_CHECKING,
    Any,
    get_args,
    get_origin,
    get_type_hints,
    overload,
)

from typing_extensions import TypeVar as TypeVarExtension

if TYPE_CHECKING:
    from dimos.core.introspection.module import ModuleInfo

from typing import TypeVar

from dask.distributed import Actor, get_worker
from reactivex.disposable import CompositeDisposable

from dimos.core import colors
from dimos.core.core import T, rpc
from dimos.core.introspection.module import extract_module_info, render_module_io
from dimos.core.resource import Resource
from dimos.core.rpc_client import RpcCall
from dimos.core.stream import In, Out, RemoteIn, RemoteOut, Transport
from dimos.protocol.rpc import LCMRPC, RPCSpec
from dimos.protocol.service import Configurable  # type: ignore[attr-defined]
from dimos.protocol.skill.skill import SkillContainer
from dimos.protocol.tf import LCMTF, TFSpec
from dimos.utils.generic import classproperty


def get_loop() -> tuple[asyncio.AbstractEventLoop, threading.Thread | None]:
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
    frame_id_prefix: str | None = None
    frame_id: str | None = None


ModuleConfigT = TypeVarExtension("ModuleConfigT", bound=ModuleConfig, default=ModuleConfig)


class ModuleBase(Configurable[ModuleConfigT], SkillContainer, Resource):
    _rpc: RPCSpec | None = None
    _tf: TFSpec | None = None
    _loop: asyncio.AbstractEventLoop | None = None
    _loop_thread: threading.Thread | None
    _disposables: CompositeDisposable
    _bound_rpc_calls: dict[str, RpcCall] = {}

    rpc_calls: list[str] = []

    default_config: type[ModuleConfigT] = ModuleConfig  # type: ignore[assignment]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
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
            self.rpc.start()  # type: ignore[attr-defined]
        except ValueError:
            ...

    @property
    def frame_id(self) -> str:
        base = self.config.frame_id or self.__class__.__name__
        if self.config.frame_id_prefix:
            return f"{self.config.frame_id_prefix}/{base}"
        return base

    @rpc
    def start(self) -> None:
        pass

    @rpc
    def stop(self) -> None:
        self._close_module()
        super().stop()

    def _close_module(self) -> None:
        self._close_rpc()
        if hasattr(self, "_loop") and self._loop_thread:
            if self._loop_thread.is_alive():
                self._loop.call_soon_threadsafe(self._loop.stop)  # type: ignore[union-attr]
                self._loop_thread.join(timeout=2)
            self._loop = None
            self._loop_thread = None
        if hasattr(self, "_tf") and self._tf is not None:
            self._tf.stop()
            self._tf = None
        if hasattr(self, "_disposables"):
            self._disposables.dispose()

    def _close_rpc(self) -> None:
        # Using hasattr is needed because SkillCoordinator skips ModuleBase.__init__ and self.rpc is never set.
        if hasattr(self, "rpc") and self.rpc:
            self.rpc.stop()  # type: ignore[attr-defined]
            self.rpc = None  # type: ignore[assignment]

    def __getstate__(self):  # type: ignore[no-untyped-def]
        """Exclude unpicklable runtime attributes when serializing."""
        state = self.__dict__.copy()
        # Remove unpicklable attributes
        state.pop("_disposables", None)
        state.pop("_loop", None)
        state.pop("_loop_thread", None)
        state.pop("_rpc", None)
        state.pop("_tf", None)
        return state

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        """Restore object from pickled state."""
        self.__dict__.update(state)
        # Reinitialize runtime attributes
        self._disposables = CompositeDisposable()
        self._loop = None
        self._loop_thread = None
        self._rpc = None
        self._tf = None

    @property
    def tf(self):  # type: ignore[no-untyped-def]
        if self._tf is None:
            # self._tf = self.config.tf_transport()
            self._tf = LCMTF()
        return self._tf

    @tf.setter
    def tf(self, value) -> None:  # type: ignore[no-untyped-def]
        import warnings

        warnings.warn(
            "tf is available on all modules. Call self.tf.start() to activate tf functionality. No need to assign it",
            UserWarning,
            stacklevel=2,
        )

    @property
    def outputs(self) -> dict[str, Out]:  # type: ignore[type-arg]
        return {
            name: s
            for name, s in self.__dict__.items()
            if isinstance(s, Out) and not name.startswith("_")
        }

    @property
    def inputs(self) -> dict[str, In]:  # type: ignore[type-arg]
        return {
            name: s
            for name, s in self.__dict__.items()
            if isinstance(s, In) and not name.startswith("_")
        }

    @classproperty
    def rpcs(self) -> dict[str, Callable[..., Any]]:
        return {
            name: getattr(self, name)
            for name in dir(self)
            if not name.startswith("_")
            and name != "rpcs"  # Exclude the rpcs property itself to prevent recursion
            and callable(getattr(self, name, None))
            and hasattr(getattr(self, name), "__rpc__")
        }

    @rpc
    def _io_instance(self, color: bool = True) -> str:
        """Instance-level io() - shows actual running streams."""
        return render_module_io(
            name=self.__class__.__name__,
            inputs=self.inputs,
            outputs=self.outputs,
            rpcs=self.rpcs,
            color=color,
        )

    @classmethod
    def _io_class(cls, color: bool = True) -> str:
        """Class-level io() - shows declared stream types from annotations."""
        hints = get_type_hints(cls)

        _yellow = colors.yellow if color else (lambda x: x)
        _green = colors.green if color else (lambda x: x)

        def is_stream(hint: type, stream_type: type) -> bool:
            origin = get_origin(hint)
            if origin is stream_type:
                return True
            if isinstance(hint, type) and issubclass(hint, stream_type):
                return True
            return False

        def format_stream(name: str, hint: type) -> str:
            args = get_args(hint)
            type_name = args[0].__name__ if args else "?"
            return f"{_yellow(name)}: {_green(type_name)}"

        inputs = {
            name: format_stream(name, hint) for name, hint in hints.items() if is_stream(hint, In)
        }
        outputs = {
            name: format_stream(name, hint) for name, hint in hints.items() if is_stream(hint, Out)
        }

        return render_module_io(
            name=cls.__name__,
            inputs=inputs,
            outputs=outputs,
            rpcs=cls.rpcs,
            color=color,
        )

    class _io_descriptor:
        """Descriptor that makes io() work on both class and instance."""

        def __get__(
            self, obj: "ModuleBase | None", objtype: type["ModuleBase"]
        ) -> Callable[[bool], str]:
            if obj is None:
                return objtype._io_class
            return obj._io_instance

    io = _io_descriptor()

    @classmethod
    def _module_info_class(cls) -> "ModuleInfo":
        """Class-level module_info() - returns ModuleInfo from annotations."""

        hints = get_type_hints(cls)

        def is_stream(hint: type, stream_type: type) -> bool:
            origin = get_origin(hint)
            if origin is stream_type:
                return True
            if isinstance(hint, type) and issubclass(hint, stream_type):
                return True
            return False

        def format_stream(name: str, hint: type) -> str:
            args = get_args(hint)
            type_name = args[0].__name__ if args else "?"
            return f"{name}: {type_name}"

        inputs = {
            name: format_stream(name, hint) for name, hint in hints.items() if is_stream(hint, In)
        }
        outputs = {
            name: format_stream(name, hint) for name, hint in hints.items() if is_stream(hint, Out)
        }

        return extract_module_info(
            name=cls.__name__,
            inputs=inputs,
            outputs=outputs,
            rpcs=cls.rpcs,
        )

    class _module_info_descriptor:
        """Descriptor that makes module_info() work on both class and instance."""

        def __get__(
            self, obj: "ModuleBase | None", objtype: type["ModuleBase"]
        ) -> Callable[[], "ModuleInfo"]:
            if obj is None:
                return objtype._module_info_class
            # For instances, extract from actual streams
            return lambda: extract_module_info(
                name=obj.__class__.__name__,
                inputs=obj.inputs,
                outputs=obj.outputs,
                rpcs=obj.rpcs,
            )

    module_info = _module_info_descriptor()

    @classproperty
    def blueprint(self):  # type: ignore[no-untyped-def]
        # Here to prevent circular imports.
        from dimos.core.blueprints import create_module_blueprint

        return partial(create_module_blueprint, self)  # type: ignore[arg-type]

    @rpc
    def get_rpc_method_names(self) -> list[str]:
        return self.rpc_calls

    @rpc
    def set_rpc_method(self, method: str, callable: RpcCall) -> None:
        callable.set_rpc(self.rpc)  # type: ignore[arg-type]
        self._bound_rpc_calls[method] = callable

    @overload
    def get_rpc_calls(self, method: str) -> RpcCall: ...

    @overload
    def get_rpc_calls(self, method1: str, method2: str, *methods: str) -> tuple[RpcCall, ...]: ...

    def get_rpc_calls(self, *methods: str) -> RpcCall | tuple[RpcCall, ...]:  # type: ignore[misc]
        missing = [m for m in methods if m not in self._bound_rpc_calls]
        if missing:
            raise ValueError(
                f"RPC methods not found. Class: {self.__class__.__name__}, RPC methods: {', '.join(missing)}"
            )
        result = tuple(self._bound_rpc_calls[m] for m in methods)
        return result[0] if len(result) == 1 else result


class Module(ModuleBase[ModuleConfigT]):
    ref: Actor
    worker: int

    def __init_subclass__(cls, **kwargs: Any) -> None:
        """Set class-level None attributes for In/Out type annotations.

        This is needed because Dask's Actor proxy looks up attributes on the class
        (not instance) when proxying attribute access. Without class-level attributes,
        the proxy would fail with AttributeError even though the instance has the attrs.
        """
        super().__init_subclass__(**kwargs)

        # Get type hints for this class only (not inherited ones).
        globalns = {}
        for c in cls.__mro__:
            if c.__module__ in sys.modules:
                globalns.update(sys.modules[c.__module__].__dict__)

        try:
            hints = get_type_hints(cls, globalns=globalns, include_extras=True)
        except (NameError, AttributeError, TypeError):
            hints = {}

        for name, ann in hints.items():
            origin = get_origin(ann)
            if origin in (In, Out):
                # Set class-level attribute if not already set.
                if not hasattr(cls, name) or getattr(cls, name) is None:
                    setattr(cls, name, None)

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        self.ref = None  # type: ignore[assignment]

        # Get type hints with proper namespace resolution for subclasses
        # Collect namespaces from all classes in the MRO chain
        import sys

        globalns = {}
        for cls in self.__class__.__mro__:
            if cls.__module__ in sys.modules:
                globalns.update(sys.modules[cls.__module__].__dict__)

        try:
            hints = get_type_hints(self.__class__, globalns=globalns, include_extras=True)
        except (NameError, AttributeError, TypeError):
            # If we still can't resolve hints, skip type hint processing
            # This can happen with complex forward references
            hints = {}

        for name, ann in hints.items():
            origin = get_origin(ann)
            if origin is Out:
                inner, *_ = get_args(ann) or (Any,)
                stream = Out(inner, name, self)  # type: ignore[var-annotated]
                setattr(self, name, stream)
            elif origin is In:
                inner, *_ = get_args(ann) or (Any,)
                stream = In(inner, name, self)  # type: ignore[assignment]
                setattr(self, name, stream)
        super().__init__(*args, **kwargs)

    def set_ref(self, ref) -> int:  # type: ignore[no-untyped-def]
        worker = get_worker()
        self.ref = ref
        self.worker = worker.name
        return worker.name  # type: ignore[no-any-return]

    def __str__(self) -> str:
        return f"{self.__class__.__name__}"

    @rpc
    def set_transport(self, stream_name: str, transport: Transport) -> bool:  # type: ignore[type-arg]
        stream = getattr(self, stream_name, None)
        if not stream:
            raise ValueError(f"{stream_name} not found in {self.__class__.__name__}")

        if not isinstance(stream, Out) and not isinstance(stream, In):
            raise TypeError(f"Output {stream_name} is not a valid stream")

        stream._transport = transport
        return True

    # called from remote
    def connect_stream(self, input_name: str, remote_stream: RemoteOut[T]):  # type: ignore[no-untyped-def]
        input_stream = getattr(self, input_name, None)
        if not input_stream:
            raise ValueError(f"{input_name} not found in {self.__class__.__name__}")
        if not isinstance(input_stream, In):
            raise TypeError(f"Input {input_name} is not a valid stream")
        input_stream.connection = remote_stream

    def dask_receive_msg(self, input_name: str, msg: Any) -> None:
        getattr(self, input_name).transport.dask_receive_msg(msg)

    def dask_register_subscriber(self, output_name: str, subscriber: RemoteIn[T]) -> None:
        getattr(self, output_name).transport.dask_register_subscriber(subscriber)


ModuleT = TypeVar("ModuleT", bound="Module")
