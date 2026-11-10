# Copyright 2026 Dimensional Inc.
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

"""Runs a StateTree on its owning Module's event loop."""

import asyncio
from collections.abc import Callable
import math
import threading
from types import TracebackType
from typing import Generic, TypeVar

from dimos.core.statetree.runtime import Runtime, StateTree, Status
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

C = TypeVar("C")


class StateTreeComponent(Generic[C]):
    """One running StateTree owned by a Module.

    Start it from the owner's loop, normally with ``async with`` inside ``main()``.
    Selection, task callbacks and ticks then all run on that loop, so tasks may
    touch the owner without locks. Only ``send_event`` is safe from other threads.
    """

    def __init__(
        self,
        tree: StateTree[C],
        context: C,
        tick_rate: float = 10.0,
        on_status_changed: Callable[[Status | None], None] | None = None,
    ) -> None:
        """Bind a tree to its context without starting it.

        Args:
            tree: Definition to run. It may be shared between components.
            context: Passed to every task and predicate. Usually the owning Module.
            tick_rate: Ticks per second while the tree is running.
            on_status_changed: Called on the loop with each new status, and with
                None once the tree is stopped.
        """
        if not (math.isfinite(tick_rate) and tick_rate > 0):
            raise ValueError("tick_rate must be finite and positive")
        self._runtime = Runtime(tree, context)
        self._tick_period = 1.0 / tick_rate
        self._on_status_changed = on_status_changed
        self._reported_status: Status | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_lock = threading.Lock()
        self._ticker: asyncio.Task[None] | None = None

    @property
    def status(self) -> Status | None:
        """None before start and after stop. Terminal results persist until restart."""
        return self._runtime.status

    @property
    def active_path(self) -> tuple[str, ...]:
        """Names of the active states from root to leaf."""
        return self._runtime.active_path

    async def start(self) -> None:
        """Select the initial states and begin ticking. Restarts a finished tree."""
        loop = asyncio.get_running_loop()
        try:
            self._runtime.start()
        finally:
            self._report_status()
        with self._loop_lock:
            self._loop = loop
        if self._runtime.status is Status.RUNNING:
            self._ticker = asyncio.create_task(self._tick_until_finished(loop.time()))

    async def stop(self) -> None:
        """Stop ticking and exit every active task."""
        ticker, self._ticker = self._ticker, None
        if ticker is not None:
            ticker.cancel()
        try:
            self._runtime.stop()
        finally:
            self._report_status()
            if ticker is not None:
                await asyncio.gather(ticker, return_exceptions=True)

    def send_event(self, name: str) -> None:
        """Queue an event for the next tick, from any thread.

        The event is dropped when the tree is not running.
        """
        if not name:
            raise ValueError("Event name must be nonempty")
        with self._loop_lock:
            loop = self._loop
        if loop is not None:
            loop.call_soon_threadsafe(self._deliver_event, name)

    async def __aenter__(self) -> "StateTreeComponent[C]":
        await self.start()
        return self

    async def __aexit__(
        self,
        exctype: type[BaseException] | None,
        excinst: BaseException | None,
        exctb: TracebackType | None,
    ) -> None:
        await self.stop()

    def _deliver_event(self, name: str) -> None:
        if self._runtime.status is Status.RUNNING:
            self._runtime.send_event(name)

    async def _tick_until_finished(self, started_at: float) -> None:
        loop = asyncio.get_running_loop()
        last_tick = deadline = started_at
        while self._runtime.status is Status.RUNNING:
            deadline = max(deadline + self._tick_period, loop.time())
            await asyncio.sleep(deadline - loop.time())
            now = loop.time()
            path = self._runtime.active_path
            try:
                self._runtime.tick(now - last_tick)
            except Exception:
                logger.exception("StateTree failed", active_path=path)
            last_tick = now
            self._report_status()

    def _report_status(self) -> None:
        status = self._runtime.status
        if status is self._reported_status:
            return
        self._reported_status = status
        if self._on_status_changed is None:
            return
        try:
            self._on_status_changed(status)
        except Exception:
            logger.exception("StateTree on_status_changed failed", status=status)
