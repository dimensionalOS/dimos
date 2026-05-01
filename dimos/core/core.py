#!/usr/bin/env python3
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
import functools
import inspect
from typing import (
    TYPE_CHECKING,
    Any,
    ParamSpec,
    TypeVar,
    cast,
)

if TYPE_CHECKING:
    from collections.abc import Callable

T = TypeVar("T")
P = ParamSpec("P")
R = TypeVar("R")


def rpc(fn: Callable[P, R]) -> Callable[P, R]:
    """Mark a method as an RPC body callable across modules.

    Sync methods are tagged in place. Async methods get a sync dispatcher that
    runs the coroutine on `self._loop`:

      * Caller is on self._loop (another async @rpc, a handle_*, or a
        process_observable callback): returns the coroutine so the caller can
        `await` it normally.
      * Caller is on any other thread (RPC dispatcher, sync test, sync @rpc on
        the same module): schedules the coroutine onto self._loop and blocks
        until done.
    """
    if not inspect.iscoroutinefunction(fn):
        fn.__rpc__ = True  # type: ignore[attr-defined]
        return fn

    @functools.wraps(fn)
    def wrapper(self: Any, *args: Any, **kwargs: Any) -> Any:
        loop = self._loop
        if loop is None:
            raise RuntimeError("async @rpc method called outside a running module loop")
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if running is loop:
            return fn(self, *args, **kwargs)  # type: ignore[call-arg]
        future = asyncio.run_coroutine_threadsafe(fn(self, *args, **kwargs), loop)  # type: ignore[call-arg, arg-type]
        return future.result()

    wrapper.__rpc__ = True  # type: ignore[attr-defined]
    wrapper.aio = fn  # type: ignore[attr-defined]
    return cast("Callable[P, R]", wrapper)
