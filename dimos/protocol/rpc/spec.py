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

    # we bootstrap these from the call() implementation above
    def call_sync(self, name: str, arguments: Args, timeout: float = 1.0) -> Any:
        res = Empty
        start_time = time.time()

        def receive_value(val):
            nonlocal res
            res = val

        self.call(name, arguments, receive_value)

        total_time = 0.0
        while res is Empty:
            if time.time() - start_time > timeout:
                print(f"RPC {name} timed out")
                return None
            time.sleep(0.05)
            total_time += 0.1
        return res

    async def call_async(self, name: str, arguments: Args) -> Any:
        loop = asyncio.get_event_loop()
        future = loop.create_future()

        def receive_value(val):
            try:
                loop.call_soon_threadsafe(future.set_result, val)
            except Exception as e:
                loop.call_soon_threadsafe(future.set_exception, e)

        self.call(name, arguments, receive_value)

        return await future


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


class RPC(RPCServer, RPCClient): ...
