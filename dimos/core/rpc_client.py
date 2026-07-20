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
from collections.abc import Callable
from typing import TYPE_CHECKING, Any, Protocol

from dimos.core.coordination.python_worker import Actor, MethodCallProxy
from dimos.core.stream import RemoteStream
from dimos.core.transport_factory import rpc_backend
from dimos.protocol.rpc.spec import RPCSpec
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.module import ModuleBase

logger = setup_logger()


class RpcCall:
    _rpc: RPCSpec | None
    _name: str
    _remote_name: str
    _unsub_fns: list  # type: ignore[type-arg]
    _stop_rpc_client: Callable[[], None] | None = None

    def __init__(
        self,
        original_method: Callable[..., Any] | None,
        rpc: RPCSpec,
        name: str,
        remote_name: str,
        unsub_fns: list,  # type: ignore[type-arg]
        stop_client: Callable[[], None] | None = None,
    ) -> None:
        self._rpc = rpc
        self._name = name
        self._remote_name = remote_name
        self._unsub_fns = unsub_fns
        self._stop_rpc_client = stop_client

        if original_method:
            self.__doc__ = original_method.__doc__
            self.__name__ = original_method.__name__
            self.__qualname__ = f"{self.__class__.__name__}.{original_method.__name__}"

    def set_rpc(self, rpc: RPCSpec) -> None:
        self._rpc = rpc

    def __call__(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        if not self._rpc:
            logger.warning("RPC client not initialized")
            return None

        # For stop, use call_nowait to avoid deadlock
        # (the remote side stops its RPC service before responding)
        if self._name == "stop":
            self._rpc.call_nowait(f"{self._remote_name}/{self._name}", (args, kwargs))  # type: ignore[arg-type]
            if self._stop_rpc_client:
                self._stop_rpc_client()
            return None

        result, unsub_fn = self._rpc.call_sync(
            f"{self._remote_name}/{self._name}",
            (args, kwargs),  # type: ignore[arg-type]
        )
        self._unsub_fns.append(unsub_fn)
        return result

    def __getstate__(self):  # type: ignore[no-untyped-def]
        return (self._name, self._remote_name)

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        self._name, self._remote_name = state
        self._unsub_fns = []
        self._rpc = None
        self._stop_rpc_client = None


class ModuleProxyProtocol(Protocol):
    """Protocol for host-side handles to remote modules (worker or Docker)."""

    def build(self) -> None: ...
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def set_transport(self, stream_name: str, transport: Any) -> bool: ...


class RPCClient:
    def __init__(
        self,
        actor_instance: Actor | None,
        actor_class: type[ModuleBase],
        remote_name: str | None = None,
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
        self.actor_class = actor_class
        self.remote_name = remote_name or actor_class.__name__
        self.actor_instance = actor_instance
        self.rpcs = actor_class.rpcs.keys()
        self._unsub_fns: list = []  # type: ignore[type-arg]

    @classmethod
    def remote(cls, actor_class: type[ModuleBase], *, rpc: RPCSpec | None = None) -> RPCClient:
        """Build an RPCClient with no parent-side Actor (cross-process clients)."""
        return cls(None, actor_class, rpc=rpc)

    def stop_rpc_client(self) -> None:
        for unsub in self._unsub_fns:
            try:
                unsub()
            except Exception:
                pass

        self._unsub_fns = []

        if self.rpc and self._owns_rpc:
            self.rpc.stop()
            self.rpc = None  # type: ignore[assignment]

    def __reduce__(self):  # type: ignore[no-untyped-def]
        # Return the class and the arguments needed to reconstruct the object.
        # remote_name must be included or proxies pickled into workers would
        # fall back to class-name RPC topics.
        return (
            self.__class__,
            (self.actor_instance, self.actor_class, self.remote_name),
        )

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
            return RpcCall(
                original_method,
                self.rpc,
                name,
                self.remote_name,
                self._unsub_fns,
                self.stop_rpc_client,
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
