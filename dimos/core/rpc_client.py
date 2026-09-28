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

from __future__ import annotations

import asyncio
from collections.abc import Callable, Iterable
import functools
import inspect
import threading
from typing import TYPE_CHECKING, Any, Protocol

from dimos.core.coordination.python_worker import Actor, MethodCallProxy
from dimos.core.stream import RemoteStream
from dimos.core.transport_factory import rpc_backend
from dimos.protocol.rpc.jsonrpc import JsonRPC
from dimos.protocol.rpc.spec import RPCSpec
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.module import ModuleBase, SkillInfo

logger = setup_logger()


def _rpc_signature(method: Callable[..., Any], *, for_pickle: bool = False) -> inspect.Signature:
    signature = inspect.signature(method)
    parameters = list(signature.parameters.values())
    if parameters and parameters[0].name in {"self", "cls"}:
        parameters = parameters[1:]
    if for_pickle:
        parameters = [p.replace(annotation=inspect.Parameter.empty) for p in parameters]
        return signature.replace(
            parameters=parameters,
            return_annotation=inspect.Signature.empty,
        )
    return signature.replace(parameters=parameters)


class RpcCall:
    _rpc: RPCSpec | None
    _rpc_factory: Callable[[], RPCSpec] | None
    _name: str
    _remote_name: str
    _unsub_fns: list  # type: ignore[type-arg]
    _stop_rpc_client: Callable[[], None] | None = None
    __signature__: inspect.Signature | None = None

    def __init__(
        self,
        original_method: Callable[..., Any] | None,
        rpc: RPCSpec | None,
        name: str,
        remote_name: str,
        unsub_fns: list,  # type: ignore[type-arg]
        stop_client: Callable[[], None] | None = None,
        *,
        rpc_factory: Callable[[], RPCSpec] | None = None,
        signature: inspect.Signature | None = None,
    ) -> None:
        self._rpc = rpc
        self._rpc_factory = rpc_factory
        self._name = name
        self._remote_name = remote_name
        self._unsub_fns = unsub_fns
        self._stop_rpc_client = stop_client

        self.__name__ = name
        self.__qualname__ = f"{self.__class__.__name__}.{name}"
        if original_method is not None:
            functools.update_wrapper(self, original_method)
        if signature is not None:
            self.__signature__ = signature
        elif original_method is not None:
            self.__signature__ = _rpc_signature(original_method)

    @property
    def rpc_name(self) -> str:
        """The method name advertised by the remote module."""
        return self._name

    @property
    def remote_name(self) -> str:
        """The exact deployed module-instance name."""
        return self._remote_name

    def set_rpc(self, rpc: RPCSpec) -> None:
        self._rpc = rpc
        self._rpc_factory = None

    def __call__(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        rpc = self._rpc
        if rpc is None and self._rpc_factory is not None:
            rpc = self._rpc_factory()
            self._rpc = rpc
        if rpc is None:
            logger.warning("RPC client not initialized")
            return None

        arguments = (list(args), kwargs)
        if rpc.named_params is True and self.__signature__ is not None:
            # Send every argument by name, as the signature binds it.
            bound = self.__signature__.bind(*args, **kwargs)
            arguments = ([], dict(bound.arguments))

        # For stop, use call_nowait to avoid deadlock
        # (the remote side stops its RPC service before responding)
        if self._name == "stop":
            rpc.call_nowait(f"{self._remote_name}/{self._name}", arguments)
            if self._stop_rpc_client:
                self._stop_rpc_client()
            return None

        result, unsub_fn = rpc.call_sync(
            f"{self._remote_name}/{self._name}",
            arguments,
        )
        self._unsub_fns.append(unsub_fn)
        return result

    def __getstate__(self):  # type: ignore[no-untyped-def]
        return (self._name, self._remote_name)

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        self._name, self._remote_name = state
        self._unsub_fns = []
        self._rpc = None
        self._rpc_factory = None
        self._stop_rpc_client = None


class ModuleProxyProtocol(Protocol):
    """Protocol for host-side handles to remote modules (worker or Docker)."""

    remote_name: str

    def build(self) -> None: ...
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def get_skills(self) -> list[SkillInfo]: ...
    def set_transport(self, stream_name: str, transport: Any) -> bool: ...


class RPCClient:
    def __init__(
        self,
        actor_instance: Actor | None,
        actor_class: type[ModuleBase] | None,
        remote_name: str | None = None,
        rpcs: Iterable[str] | None = None,
        native_rpc_signatures: dict[str, inspect.Signature] | None = None,
        *,
        rpc: RPCSpec | None = None,
    ) -> None:
        if rpc is None:
            self.rpc = rpc_backend()()
            self._owns_rpc = True
            self.rpc.start()
        else:
            self.rpc = rpc
            self._owns_rpc = False
        if actor_class is None:
            # Unpickled in another process; the class stays out of the pickle (see __reduce__).
            assert remote_name is not None and rpcs is not None
        else:
            remote_name = remote_name or actor_class.__name__
            rpcs = actor_class.rpcs if rpcs is None else rpcs
        self.actor_class = actor_class
        self.remote_name = remote_name
        self.actor_instance = actor_instance
        self.rpcs = frozenset(rpcs)
        if native_rpc_signatures is None and actor_class is not None:
            native_rpc_signatures = {
                name: _rpc_signature(method, for_pickle=True)
                for name in self.rpcs
                if (method := getattr(actor_class, name, None)) is not None
                and getattr(method, "__native_rpc__", False)
            }
        self._native_rpc_signatures = native_rpc_signatures or {}
        self._unsub_fns: list = []  # type: ignore[type-arg]
        self._native_rpc: RPCSpec | None = None
        self._native_rpc_lock = threading.Lock()

    @classmethod
    def remote(
        cls,
        actor_class: type[ModuleBase],
        remote_name: str | None = None,
        *,
        rpc: RPCSpec | None = None,
    ) -> RPCClient:
        """Build an RPCClient with no parent-side Actor (cross-process clients)."""
        return cls(None, actor_class, remote_name, rpc=rpc)

    def stop_rpc_client(self) -> None:
        for unsub in self._unsub_fns:
            try:
                unsub()
            except Exception:
                pass

        self._unsub_fns = []

        with self._native_rpc_lock:
            native, self._native_rpc = self._native_rpc, None
        if native is not None:
            native.stop()

        if self.rpc and self._owns_rpc:
            self.rpc.stop()
            self.rpc = None  # type: ignore[assignment]

    def _native(self) -> RPCSpec:
        """JSON-RPC client for @native_rpc methods; every other call stays on self.rpc."""
        with self._native_rpc_lock:
            if self._native_rpc is None:
                native = JsonRPC(
                    rpc_timeouts=self.rpc.rpc_timeouts,
                    default_rpc_timeout=self.rpc.default_rpc_timeout,
                )
                native.start()  # kept only once started, so the next call retries a failed start
                self._native_rpc = native
            return self._native_rpc

    def __reduce__(self):  # type: ignore[no-untyped-def]
        # The module class stays out of the pickle: unpickling it in another
        # worker would import the module and everything under it (a
        # SpatialMemory proxy alone pulls in transformers and torch). A proxy
        # only needs the rpc names and stripped signatures for native methods.
        # remote_name must be included or proxies pickled into workers would
        # fall back to class-name RPC topics.
        return (
            self.__class__,
            (
                self.actor_instance,
                None,
                self.remote_name,
                self.rpcs,
                self._native_rpc_signatures,
            ),
        )

    def __dir__(self) -> list[str]:
        return sorted(set(super().__dir__()) | set(self.rpcs))

    # passthrough
    def __getattr__(self, name: str):  # type: ignore[no-untyped-def]
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
            original_method = getattr(self.actor_class, name, None)
            native = name in self._native_rpc_signatures
            return RpcCall(
                original_method,
                None if native else self.rpc,
                name,
                self.remote_name,
                self._unsub_fns,
                self.stop_rpc_client,
                rpc_factory=self._native if native else None,
                signature=self._native_rpc_signatures.get(name),
            )

        if self.actor_instance is None:
            raise AttributeError(
                f"{self.remote_name!r} has no @rpc method named {name!r}; "
                f"this client was constructed without a parent-side Actor "
                f"(remote-mode), so non-@rpc attribute access is unavailable."
            )

        # return super().__getattr__(name)
        # Try to avoid recursion by directly accessing attributes that are known
        result = self.actor_instance.__getattr__(name)

        # When streams are returned from the worker, their owner is a pickled
        # Actor with no connection. Replace it with a MethodCallProxy that can
        # talk to the worker through the parent-side Actor's pipe.
        if isinstance(result, RemoteStream):
            result.owner = MethodCallProxy(self.actor_instance)

        return result


class AsyncSpecProxy:
    """Wraps an RPCClient (or compatible proxy) so methods declared `async def`
    on the consumer's Spec are exposed as awaitables on the proxy.

    A consumer that types `ref: SomeSpec` where `SomeSpec` declares `async def
    foo` will see `self.ref.foo(x)` return an awaitable. The underlying RPC call
    is still synchronous over the wire.  The caller's event loop stays unblocked
    while the response round-trips.

    It's picklable so `set_module_ref`` can ship it across to the worker process.
    """

    def __init__(self, inner: Any, async_methods: frozenset[str]) -> None:
        # Use object.__setattr__ for clarity; we don't override __setattr__
        # but this mirrors how DisabledModuleProxy guards its internals.
        object.__setattr__(self, "_inner", inner)
        object.__setattr__(self, "_async_methods", async_methods)

    def __getattr__(self, name: str) -> Any:
        inner = object.__getattribute__(self, "_inner")
        attr = getattr(inner, name)
        async_methods = object.__getattribute__(self, "_async_methods")
        if name not in async_methods or not callable(attr):
            return attr

        def async_call(*args: Any, **kwargs: Any) -> Any:
            async def _run() -> Any:
                running = asyncio.get_running_loop()
                return await running.run_in_executor(None, lambda: attr(*args, **kwargs))

            return _run()

        return async_call

    def __reduce__(self) -> Any:
        return (
            AsyncSpecProxy,
            (
                object.__getattribute__(self, "_inner"),
                object.__getattribute__(self, "_async_methods"),
            ),
        )


if TYPE_CHECKING:
    from dimos.core.module import Module

    # the class below is only ever used for type hinting
    # why? because the RPCClient instance is going to have all the methods of a Module
    # but those methods/attributes are super dynamic, so the type hints can't figure that out
    class ModuleProxy(RPCClient, Module):  # type: ignore[misc]
        def build(self) -> None: ...
        def start(self) -> None: ...
        def stop(self) -> None: ...
