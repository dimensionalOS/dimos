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

import asyncio
from collections.abc import AsyncIterator, Iterator
import logging
from queue import Queue
from typing import Any

import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import pLCMTransport


@pytest.fixture
def module_log_records() -> Iterator[list[logging.LogRecord]]:
    records: list[logging.LogRecord] = []

    class _ListHandler(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            records.append(record)

    handler = _ListHandler(level=logging.DEBUG)
    target = logging.getLogger("dimos/core/module.py")
    target.addHandler(handler)
    try:
        yield records
    finally:
        target.removeHandler(handler)


class _Resource:
    """Tiny stand-in for an external resource a module might own."""

    def __init__(self) -> None:
        self.started = False
        self.stop_count = 0

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stop_count += 1


class HappyMain(Module):
    """Records setup/teardown order and verifies main runs on _loop."""

    events: list[str]
    resource: _Resource

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.events = []
        self.resource = _Resource()

    async def main(self) -> AsyncIterator[None]:
        assert asyncio.get_running_loop() is self._loop
        self.resource.start()
        self.events.append("setup")
        yield
        self.events.append("teardown")
        self.resource.stop()


def test_main_setup_runs_before_start_returns():
    m = HappyMain()
    assert m.events == []
    m.start()
    try:
        assert m.events == ["setup"]
        assert m.resource.started is True
    finally:
        m.stop()


def test_main_teardown_runs_during_stop():
    m = HappyMain()
    m.start()
    m.stop()
    assert m.events == ["setup", "teardown"]
    assert m.resource.stop_count == 1


def test_main_teardown_runs_only_once():
    m = HappyMain()
    m.start()
    m.stop()
    # Calling stop() again should be a no-op for main (already torn down).
    m.stop()
    assert m.resource.stop_count == 1


class NoYieldMain(Module):
    async def main(self) -> AsyncIterator[None]:
        # Lexically contains yield (so isasyncgenfunction is True), but runtime
        # never reaches it -> StopAsyncIteration on first __anext__.
        if False:
            yield


def test_main_with_zero_runtime_yields_raises():
    m = NoYieldMain()
    with pytest.raises(RuntimeError, match="exactly one `yield`.*found none"):
        m.start()
    # Even though start failed, stop should still be safe to call.
    m.stop()


class NotAGeneratorMain(Module):
    async def main(self) -> None:
        return None


def test_main_that_is_not_an_async_generator_raises():
    m = NotAGeneratorMain()
    with pytest.raises(TypeError, match="must be an `async def` with exactly one"):
        m.start()
    m.stop()


class TwoYieldsMain(Module):
    teardown_count: int

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.teardown_count = 0

    async def main(self) -> AsyncIterator[None]:
        yield
        yield
        self.teardown_count += 1


def test_main_with_two_yields_logs_and_continues_stop(module_log_records):
    m = TwoYieldsMain()
    m.start()
    m.stop()  # Must not raise.
    assert any("yielded more than once" in rec.getMessage() for rec in module_log_records)
    # The second-section code after the second yield should NOT have run
    # because we close the generator instead of running it through.
    assert m.teardown_count == 0


class TeardownErrorMain(Module):
    teardown_attempted: bool

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.teardown_attempted = False

    async def main(self) -> AsyncIterator[None]:
        yield
        self.teardown_attempted = True
        raise RuntimeError("teardown failure")


def test_main_teardown_error_is_logged_not_raised(module_log_records):
    m = TeardownErrorMain()
    m.start()
    m.stop()  # Must not raise.
    assert m.teardown_attempted is True
    assert any(
        "teardown" in rec.getMessage() and rec.levelno == logging.ERROR
        for rec in module_log_records
    )


class SetupErrorMain(Module):
    teardown_ran: bool

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.teardown_ran = False

    async def main(self) -> AsyncIterator[None]:
        raise RuntimeError("setup failure")
        yield  # pragma: no cover  (unreachable, but makes this an async gen)
        self.teardown_ran = True  # pragma: no cover


def test_main_setup_error_propagates_from_start():
    m = SetupErrorMain()
    with pytest.raises(RuntimeError, match="setup failure"):
        m.start()
    # Generator was never stored, so stop() should not try to drive teardown.
    m.stop()
    assert m.teardown_ran is False


class MainAndHandlerModule(Module):
    a: In[int]
    out: Out[int]

    multiplier: int = 0
    setup_ran: bool = False
    teardown_ran: bool = False

    async def main(self) -> AsyncIterator[None]:
        self.multiplier = 7
        self.setup_ran = True
        yield
        self.teardown_ran = True
        self.multiplier = 0

    async def handle_a(self, value: int) -> None:
        self.out.publish(value * self.multiplier)


@pytest.fixture
def start_main_handler_module():
    blueprint = MainAndHandlerModule.blueprint()
    coordinator = ModuleCoordinator.build(blueprint)
    yield
    coordinator.stop()


@pytest.fixture
def a_transport():
    a_tr = pLCMTransport("/a")
    a_tr.start()
    yield a_tr
    a_tr.stop()


@pytest.fixture
def out_transport():
    out_tr = pLCMTransport("/out")
    out_tr.start()
    yield out_tr
    out_tr.stop()


@pytest.mark.slow
def test_main_and_handle_together(start_main_handler_module, a_transport, out_transport):
    queue: Queue[int] = Queue()
    out_transport.subscribe(queue.put)
    a_transport.publish(6)
    result = queue.get(timeout=4)
    assert result == 42  # 6 * 7
