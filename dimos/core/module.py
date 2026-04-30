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
from collections.abc import AsyncGenerator, Callable
from dataclasses import dataclass
from functools import partial
import inspect
import json
import sys
import threading
from typing import (
    TYPE_CHECKING,
    Any,
    ClassVar,
    Literal,
    Protocol,
    get_args,
    get_origin,
    get_type_hints,
)

from pydantic import Field
from reactivex.disposable import CompositeDisposable, Disposable

from dimos.core.core import T, rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.introspection.module.info import extract_module_info
from dimos.core.introspection.module.render import render_module_io
from dimos.core.resource import CompositeResource
from dimos.core.rpc_client import RpcCall
from dimos.core.stream import In, Out, RemoteOut, Transport
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUT, DEFAULT_RPC_TIMEOUTS, RPCSpec
from dimos.protocol.service.spec import BaseConfig, Configurable
from dimos.protocol.tf.tf import LCMTF, TFSpec
from dimos.utils import colors
from dimos.utils.generic import classproperty
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

if TYPE_CHECKING:
    from reactivex import Observable
    from reactivex.abc import DisposableBase

    from dimos.core.coordination.blueprints import Blueprint
    from dimos.core.introspection.module.info import ModuleInfo
    from dimos.core.rpc_client import RPCClient

if sys.version_info >= (3, 13):
    from typing import TypeVar
else:
    from typing_extensions import TypeVar


@dataclass(frozen=True)
class SkillInfo:
    class_name: str
    func_name: str
    args_schema: str


def get_loop() -> tuple[asyncio.AbstractEventLoop, threading.Thread | None]:
    try:
        running_loop = asyncio.get_running_loop()
        return running_loop, None
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.set_task_factory(_logging_task_factory)

        thr = threading.Thread(target=loop.run_forever, daemon=True)
        thr.start()
        return loop, thr


Deployment = Literal["python", "docker"]


class ModuleConfig(BaseConfig):
    rpc_transport: type[RPCSpec] = LCMRPC
    default_rpc_timeout: float = DEFAULT_RPC_TIMEOUT
    rpc_timeouts: dict[str, float] = Field(default_factory=lambda: dict(DEFAULT_RPC_TIMEOUTS))
    tf_transport: type[TFSpec] = LCMTF  # type: ignore[type-arg]
    frame_id_prefix: str | None = None
    frame_id: str | None = None
    g: GlobalConfig = global_config


ModuleConfigT = TypeVar("ModuleConfigT", bound=ModuleConfig, default=ModuleConfig)


class _BlueprintPartial(Protocol):
    def __call__(self, **kwargs: Any) -> "Blueprint": ...


class ModuleBase(Configurable, CompositeResource):
    config: ModuleConfig

    # Deployment target. Worker managers declare which deployment type they
    # handle; the coordinator routes modules accordingly.
    deployment: ClassVar[Deployment] = "python"

    _rpc: RPCSpec | None = None
    _tf: TFSpec | None = None
    _loop: asyncio.AbstractEventLoop | None = None
    _loop_thread: threading.Thread | None
    _bound_rpc_calls: dict[str, RpcCall] = {}
    _module_closed: bool = False
    _module_closed_lock: threading.Lock
    _loop_thread_timeout: float = 2.0
    _main_gen: AsyncGenerator[None, None] | None = None

    def __init__(self, config_args: dict[str, Any]) -> None:
        super().__init__(**config_args)
        self._module_closed_lock = threading.Lock()
        self._loop, self._loop_thread = get_loop()
        try:
            self.rpc = self.config.rpc_transport(  # type: ignore[call-arg]
                rpc_timeouts=self.config.rpc_timeouts,
                default_rpc_timeout=self.config.default_rpc_timeout,
            )
            self.rpc.serve_module_rpc(self)
            self.rpc.start()  # type: ignore[attr-defined]
        except ValueError:
            ...

    @classproperty
    def name(self) -> str:
        """Name for this module to be used for blueprint configs."""
        return self.__name__.lower()  # type: ignore[attr-defined,no-any-return]

    @property
    def frame_id(self) -> str:
        base = self.config.frame_id or self.__class__.__name__
        if self.config.frame_id_prefix:
            return f"{self.config.frame_id_prefix}/{base}"
        return base

    @rpc
    def build(self) -> None:
        """Optional build step for heavy one-time work (docker builds, LFS downloads, etc.).

        Called after deploy and stream wiring but before start().
        Has a very long timeout (24h) so long-running builds don't fail.
        Default is a no-op — override in subclasses that need a build step.
        """

    @rpc
    def start(self) -> None:
        self._start_main()
        self._auto_bind_handlers()

    @rpc
    def stop(self) -> None:
        self._stop_main()
        super().stop()
        self._close_module()

    def _close_module(self) -> None:
        with self._module_closed_lock:
            if self._module_closed:
                return
            self._module_closed = True

        self._close_rpc()

        # Save into local variables to avoid race when stopping concurrently
        # (from RPC and worker shutdown)
        loop_thread = getattr(self, "_loop_thread", None)
        loop = getattr(self, "_loop", None)

        if loop_thread:
            if loop_thread.is_alive():
                if loop:
                    loop.call_soon_threadsafe(loop.stop)
                loop_thread.join(timeout=self._loop_thread_timeout)
            self._loop = None
            self._loop_thread = None

        if hasattr(self, "_tf") and self._tf is not None:
            self._tf.stop()
            self._tf = None

        # Stop transports and break the In/Out -> owner -> self reference
        # cycle so the instance can be freed by refcount instead of waiting for GC.
        for attr in [*self.inputs.values(), *self.outputs.values()]:
            attr.stop()
            attr.owner = None

    def _close_rpc(self) -> None:
        if self.rpc:
            self.rpc.stop()  # type: ignore[attr-defined]
            self.rpc = None  # type: ignore[assignment]

    def __getstate__(self):  # type: ignore[no-untyped-def]
        """Exclude unpicklable runtime attributes when serializing."""
        state = self.__dict__.copy()
        # Remove unpicklable attributes
        state.pop("_disposables", None)
        state.pop("_module_closed_lock", None)
        state.pop("_loop", None)
        state.pop("_loop_thread", None)
        state.pop("_rpc", None)
        state.pop("_tf", None)
        state.pop("_main_gen", None)
        return state

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        """Restore object from pickled state."""
        self.__dict__.update(state)
        # Reinitialize runtime attributes
        self._module_closed_lock = threading.Lock()
        self._loop = None
        self._loop_thread = None
        self._rpc = None
        self._tf = None
        self._main_gen = None

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
            self, obj: "ModuleBase | None", objtype: "type[ModuleBase]"
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
            self, obj: "ModuleBase | None", objtype: "type[ModuleBase]"
        ) -> "Callable[[], ModuleInfo]":
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
    def blueprint(self) -> _BlueprintPartial:
        # Here to prevent circular imports.
        from dimos.core.coordination.blueprints import Blueprint

        return partial(Blueprint.create, self)  # type: ignore[arg-type]

    @rpc
    def set_module_ref(self, name: str, module_ref: "RPCClient") -> None:
        setattr(self, name, module_ref)

    @rpc
    def get_skills(self) -> list[SkillInfo]:
        from langchain_core.tools import tool  # ~170ms: deferred to avoid CLI startup cost

        skills: list[SkillInfo] = []
        for name in dir(self):
            attr = getattr(self, name)
            if callable(attr) and hasattr(attr, "__skill__"):
                schema = json.dumps(tool(attr).args_schema.model_json_schema())
                skills.append(
                    SkillInfo(
                        class_name=self.__class__.__name__, func_name=name, args_schema=schema
                    )
                )
        return skills

    def spawn(self, coro: Any) -> Any:
        """
        Schedule a coroutine on self._loop from any thread.

        Use this instead of bare `asyncio.run_coroutine_threadsafe(coro,
        self._loop)` when scheduling a long-running async task sync context like
        start().

        Unhandled exceptions are routed to the module logger instead of being
        silently stored in the returned Future, which is the common pitfall when
        nothing ever reads `.result()`.
        """

        loop = self._loop
        if loop is None or not loop.is_running():
            raise RuntimeError(f"{type(self).__name__}._loop is not running")
        future = asyncio.run_coroutine_threadsafe(coro, loop)
        future.add_done_callback(self._log_async_handler_error)
        return future

    def process_observable(
        self,
        observable: "Observable[Any]",
        async_cb: Callable[[Any], Any],
    ) -> "DisposableBase":
        """Subscribe `async_cb` (an async function) to `observable`, dispatching
        each emitted value onto self._loop. Invocations are serialized through a
        per-subscription dispatcher task with LATEST coalescing. The subscription
        is registered for cleanup on stop()."""
        if not inspect.iscoroutinefunction(async_cb):
            raise TypeError("process_observable requires an `async def` callback")
        on_msg, dispatcher_disp = self._make_async_dispatch(async_cb)
        sub = observable.subscribe(on_msg)
        return self.register_disposable(CompositeDisposable(sub, dispatcher_disp))

    def _start_main(self) -> None:
        """
        If the subclass defines `async def main(self)` as an async generator
        with exactly one `yield`, run everything before the `yield` as part of
        start().
        """
        main_fn = getattr(type(self), "main", None)
        if main_fn is None:
            return
        if not inspect.isasyncgenfunction(main_fn):
            raise TypeError(
                f"{type(self).__name__}.main must be an `async def` with exactly "
                "one `yield` (an async generator function)"
            )
        loop = self._loop
        if loop is None or not loop.is_running():
            raise RuntimeError(f"{type(self).__name__}._loop is not running")
        gen = main_fn(self)
        try:
            asyncio.run_coroutine_threadsafe(gen.__anext__(), loop).result()
        except StopAsyncIteration:
            raise RuntimeError(
                f"{type(self).__name__}.main must contain exactly one `yield` (found none)"
            ) from None
        except BaseException:
            try:
                asyncio.run_coroutine_threadsafe(gen.aclose(), loop).result()
            except BaseException:
                pass
            raise
        self._main_gen = gen

    def _stop_main(self) -> None:
        """Resume `main` past its yield so the teardown section runs."""
        gen = self._main_gen
        if gen is None:
            return
        self._main_gen = None
        loop = self._loop
        if loop is None or not loop.is_running():
            return
        try:
            asyncio.run_coroutine_threadsafe(gen.__anext__(), loop).result()
        except StopAsyncIteration:
            return
        except BaseException as e:
            # Do not fail teardown if main raises. Log and continue with best
            # effort to close the module.
            logger.exception(
                f"Error during {type(self).__name__}.main teardown: {type(e).__name__}: {e}"
            )
            return
        # No StopAsyncIteration means main yielded a second time.
        try:
            asyncio.run_coroutine_threadsafe(gen.aclose(), loop).result()
        except BaseException:
            pass
        logger.error(
            f"{type(self).__name__}.main yielded more than once; "
            "expected exactly one yield (setup, then teardown)"
        )

    def _auto_bind_handlers(self) -> None:
        """
        For each declared `x: In[T]`, if `async def handle_x` exists, subscribe it
        via process_observable so it runs on self._loop.
        """
        # Validate every handler before subscribing any of them.
        bindings: list[tuple[Any, Callable[[Any], Any]]] = []
        for input_name, in_stream in self.inputs.items():
            handler = getattr(self, f"handle_{input_name}", None)
            if handler is None:
                continue
            # Async @rpc wraps the coroutine fn in a sync dispatcher. Unwrap it
            # so we subscribe the raw coroutine fn instead of the wrapper (which
            # would block on run_coroutine_threadsafe from the rx thread).
            if hasattr(handler, "aio"):
                handler = handler.aio.__get__(self, type(self))
            if not inspect.iscoroutinefunction(handler):
                raise TypeError(
                    f"{type(self).__name__}.handle_{input_name} must be `async def` "
                    "(use a manual self.<input>.subscribe(...) for sync handlers)"
                )
            bindings.append((in_stream, handler))

        for in_stream, handler in bindings:
            # process_observable runs each handler through a per-subscription
            # dispatcher task on self._loop that serializes invocations and
            # keeps only the latest unprocessed message. We subscribe to
            # pure_observable() because the dispatcher already provides
            # backpressure.
            self.process_observable(in_stream.pure_observable(), handler)

    def _make_async_dispatch(
        self, async_handler: Callable[[Any], Any]
    ) -> tuple[Callable[[Any], None], "DisposableBase"]:
        """Build a sync callback that delivers `msg` into a single-slot LATEST
        mailbox drained by a dedicated dispatcher task on `self._loop`.

        Guarantees:
          - The handler is invoked at most one-at-a-time (no interleaving across
            awaits).
          - If messages arrive faster than the handler can process them,
            intermediate messages are dropped and only the most recent unprocessed
            message is kept (LATEST policy).
          - The returned Disposable cancels the dispatcher task.
        """
        loop = self._loop
        if loop is None or not loop.is_running():
            raise RuntimeError(f"{type(self).__name__}._loop is not running")

        async def _bootstrap() -> tuple[asyncio.Event, dict[str, Any], asyncio.Task[None]]:
            event = asyncio.Event()
            slot: dict[str, Any] = {"value": None, "has_value": False}

            async def dispatcher() -> None:
                try:
                    while True:
                        await event.wait()
                        event.clear()
                        if not slot["has_value"]:
                            continue
                        msg = slot["value"]
                        slot["value"] = None
                        slot["has_value"] = False
                        try:
                            await async_handler(msg)
                        except asyncio.CancelledError:
                            raise
                        except BaseException as e:
                            self._log_async_handler_exception(e)
                except asyncio.CancelledError:
                    return

            return event, slot, asyncio.create_task(dispatcher())

        event, slot, task = asyncio.run_coroutine_threadsafe(_bootstrap(), loop).result(timeout=5.0)

        def on_msg(msg: Any) -> None:
            loop_now = self._loop
            if loop_now is None or not loop_now.is_running():
                return

            def _set() -> None:
                slot["value"] = msg
                slot["has_value"] = True
                event.set()

            loop_now.call_soon_threadsafe(_set)

        disposed = False

        def _dispose() -> None:
            nonlocal disposed
            if disposed:
                return
            disposed = True
            loop_now = self._loop
            if loop_now is not None and loop_now.is_running():
                loop_now.call_soon_threadsafe(task.cancel)

        return on_msg, Disposable(_dispose)

    def _log_async_handler_exception(self, e: BaseException) -> None:
        if isinstance(e, asyncio.CancelledError):
            return  # task cancelled during shutdown
        # A coroutine interacting with a stopped loop surfaces as
        # RuntimeError ("Event loop is closed", "no running event loop",
        # etc.). Only swallow that when the loop is actually gone.  Anything
        # else (including RuntimeError raised by user code while the loop is
        # healthy) is a real bug worth logging.
        loop = self._loop
        if isinstance(e, RuntimeError) and (loop is None or not loop.is_running()):
            return
        # Include exception type+message in the event string so it is
        # visible on consoles whose formatters strip exc_info/traceback.
        logger.exception(
            f"Unhandled error in async task on {type(self).__name__}._loop: {type(e).__name__}: {e}"
        )

    def _log_async_handler_error(self, fut: Any) -> None:
        try:
            fut.result()
        except BaseException as e:
            self._log_async_handler_exception(e)


class Module(ModuleBase):
    def __init_subclass__(cls, **kwargs: Any) -> None:
        """Set class-level None attributes for In/Out type annotations.

        This is needed because Dask's Actor proxy looks up attributes on the class
        (not instance) when proxying attribute access. Without class-level attributes,
        the proxy would fail with AttributeError even though the instance has the attrs.
        """
        super().__init_subclass__(**kwargs)

        try:
            hints = get_type_hints(cls, include_extras=True)
        except (NameError, AttributeError, TypeError):
            hints = {}

        for name, ann in hints.items():
            origin = get_origin(ann)
            if origin in (In, Out):
                # Set class-level attribute if not already set.
                if not hasattr(cls, name) or getattr(cls, name) is None:
                    setattr(cls, name, None)

    def __init__(self, **kwargs: Any) -> None:
        self.ref = None

        try:
            hints = get_type_hints(self.__class__, include_extras=True)
        except (NameError, AttributeError, TypeError):
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
        super().__init__(config_args=kwargs)

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


ModuleSpec = tuple[type[ModuleBase], GlobalConfig, dict[str, Any]]


def is_module_type(value: Any) -> bool:
    try:
        return inspect.isclass(value) and issubclass(value, Module)
    except Exception:
        return False


def _logging_task_factory(
    loop: asyncio.AbstractEventLoop, coro: Any, **kwargs: Any
) -> asyncio.Task[Any]:
    """
    Adds a done callback to log unhandled exceptions from any task created on
    the loop.
    """
    task = asyncio.Task(coro, loop=loop, **kwargs)
    task.add_done_callback(_log_task_exception)
    return task


def _log_task_exception(task: asyncio.Task[Any]) -> None:
    if task.cancelled():
        return
    try:
        exc = task.exception()
    except asyncio.InvalidStateError:
        return
    if exc is None or isinstance(exc, (asyncio.CancelledError, StopAsyncIteration)):
        return
    # Calling task.exception() above marks the exception as retrieved, so
    # asyncio's GC-time logger won't fire. We must log here.
    name = task.get_name()
    logger.error(
        f"Unhandled exception in async task {name!r}: {type(exc).__name__}: {exc}",
        exc_info=exc,
    )
