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
from typing import Any, Callable, Optional, Protocol, overload


class Empty: ...


class RPCClient(Protocol):
    # if we don't provide callback, we don't get a return unsub f
    @overload
    def call(self, name: str, arguments: list, cb: None) -> None: ...

    # if we provide callback, we do get return unsub f
    @overload
    def call(self, name: str, arguments: list, cb: Callable[[Any], None]) -> Callable[[], Any]: ...

    def call(
        self, name: str, arguments: list, cb: Optional[Callable]
    ) -> Optional[Callable[[], Any]]: ...

    # we bootstrap these from the call() implementation above
    def call_sync(self, name: str, arguments: list) -> Any:
        res = Empty

        def receive_value(val):
            nonlocal res
            res = val

        self.call(name, arguments, receive_value)
        while res is Empty:
            time.sleep(0.05)
        return res

    async def call_async(self, name: str, arguments: list) -> Any:
        loop = asyncio.get_event_loop()
        future = loop.create_future()

        def receive_value(val):
            try:
                # Use call_soon_threadsafe to safely set the result from another thread
                loop.call_soon_threadsafe(future.set_result, val)
            except Exception as e:
                loop.call_soon_threadsafe(future.set_exception, e)

        self.call(name, arguments, receive_value)

        return await future


class RPCServer(Protocol):
    def serve(self, f: Callable, name: str) -> None: ...


class RPC(RPCServer, RPCClient): ...
