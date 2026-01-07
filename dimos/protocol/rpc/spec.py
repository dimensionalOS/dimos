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
from collections.abc import Callable
import threading
from typing import Any, Protocol, overload


class Empty: ...


Args = tuple[list, dict[str, Any]]  # type: ignore[type-arg]


# module that we can inspect for RPCs
class RPCInspectable(Protocol):
    @property
    def rpcs(self) -> dict[str, Callable]: ...  # type: ignore[type-arg]


class RPCClient(Protocol):
    # if we don't provide callback, we don't get a return unsub f
    @overload
    def call(self, name: str, arguments: Args, cb: None) -> None: ...

    # if we provide callback, we do get return unsub f
    @overload
    def call(self, name: str, arguments: Args, cb: Callable[[Any], None]) -> Callable[[], Any]: ...

    def call(self, name: str, arguments: Args, cb: Callable | None) -> Callable[[], Any] | None: ...  # type: ignore[type-arg]

    # we expect to crash if we don't get a return value after 10 seconds
    # but callers can override this timeout for extra long functions
    def call_sync(
        self, name: str, arguments: Args, rpc_timeout: float | None = 120.0
    ) -> tuple[Any, Callable[[], None]]:
        if name == "start":
            rpc_timeout = 1200.0  # starting modules can take longer
        event = threading.Event()

        def receive_value(val) -> None:  # type: ignore[no-untyped-def]
            event.result = val  # type: ignore[attr-defined]  # attach to event
            event.set()

        unsub_fn = self.call(name, arguments, receive_value)
        if not event.wait(rpc_timeout):
            raise TimeoutError(f"RPC call to '{name}' timed out after {rpc_timeout} seconds")

        # Check if the result is an exception and raise it
        result = event.result  # type: ignore[attr-defined]
        if isinstance(result, BaseException):
            raise result

        return result, unsub_fn

    async def call_async(self, name: str, arguments: Args) -> Any:
        loop = asyncio.get_event_loop()
        future = loop.create_future()

        def receive_value(val) -> None:  # type: ignore[no-untyped-def]
            try:
                # Check if the value is an exception
                if isinstance(val, BaseException):
                    loop.call_soon_threadsafe(future.set_exception, val)
                else:
                    loop.call_soon_threadsafe(future.set_result, val)
            except Exception as e:
                loop.call_soon_threadsafe(future.set_exception, e)

        self.call(name, arguments, receive_value)

        return await future


class RPCServer(Protocol):
    def serve_rpc(self, f: Callable, name: str) -> Callable[[], None]: ...  # type: ignore[type-arg]

    def serve_module_rpc(self, module: RPCInspectable, name: str | None = None) -> None:
        for fname in module.rpcs.keys():
            if not name:
                name = module.__class__.__name__

            def override_f(*args, fname=fname, **kwargs):  # type: ignore[no-untyped-def]
                return getattr(module, fname)(*args, **kwargs)

            topic = name + "/" + fname
            self.serve_rpc(override_f, topic)


class RPCSpec(RPCServer, RPCClient): ...
