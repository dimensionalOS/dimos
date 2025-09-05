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
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Protocol, Tuple, overload


class Empty: ...


Args = Tuple[List, Dict[str, Any]]


# module that we can inspect for RPCs
class RPCInspectable(Protocol):
    @property
    def rpcs(self) -> dict[str, Callable]: ...


class RPCClient(Protocol):
    # if we don't provide callback, we don't get a return unsub f
    @overload
    def call(self, name: str, arguments: Args, cb: None) -> None: ...

    # if we provide callback, we do get return unsub f
    @overload
    def call(self, name: str, arguments: Args, cb: Callable[[Any], None]) -> Callable[[], Any]: ...

    def call(
        self, name: str, arguments: Args, cb: Optional[Callable]
    ) -> Optional[Callable[[], Any]]: ...

    # we expect to crash if we don't get a return value after 10 seconds
    # but callers can override this timeout for extra long functions
    def call_sync(
        self, name: str, arguments: Args, rpc_timeout: Optional[float] = 2.0, max_retries: int = 3
    ) -> Any:
        last_error = None
        for attempt in range(max_retries):
            event = threading.Event()

            def receive_value(val):
                event.result = val  # attach to event
                event.set()

            try:
                self.call(name, arguments, receive_value)
                if event.wait(rpc_timeout):
                    # Got a response, return it (whether success or failure)
                    return event.result

                # Timeout occurred, retry if we have attempts left
                last_error = TimeoutError(
                    f"RPC call to '{name}' timed out after {rpc_timeout} seconds (attempt {attempt + 1}/{max_retries})"
                )
                if attempt < max_retries - 1:
                    time.sleep(0.5)  # Brief delay before retry
            except Exception as e:
                # Non-timeout exception, don't retry
                raise e

        raise last_error

    async def call_async(
        self, name: str, arguments: Args, rpc_timeout: Optional[float] = 3.0, max_retries: int = 3
    ) -> Any:
        last_error = None
        for attempt in range(max_retries):
            loop = asyncio.get_event_loop()
            future = loop.create_future()

            def receive_value(val):
                try:
                    loop.call_soon_threadsafe(future.set_result, val)
                except Exception as e:
                    loop.call_soon_threadsafe(future.set_exception, e)

            self.call(name, arguments, receive_value)

            try:
                # Got a response, return it (whether success or failure)
                return await asyncio.wait_for(future, timeout=rpc_timeout)
            except asyncio.TimeoutError:
                # Timeout occurred, retry if we have attempts left
                last_error = TimeoutError(
                    f"RPC call to '{name}' timed out after {rpc_timeout} seconds (attempt {attempt + 1}/{max_retries})"
                )
                if attempt < max_retries - 1:
                    await asyncio.sleep(0.1)  # Brief delay before retry
            except Exception as e:
                # Non-timeout exception, don't retry
                raise e

        raise last_error


class RPCServer(Protocol):
    def serve_rpc(self, f: Callable, name: str) -> None: ...

    def serve_module_rpc(self, module: RPCInspectable, name: Optional[str] = None):
        for fname in module.rpcs.keys():
            if not name:
                name = module.__class__.__name__

            def override_f(*args, fname=fname, **kwargs):
                return getattr(module, fname)(*args, **kwargs)

            topic = name + "/" + fname
            self.serve_rpc(override_f, topic)


class RPCSpec(RPCServer, RPCClient): ...
