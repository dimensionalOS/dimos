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

from __future__ import annotations

from collections.abc import Callable
from threading import Event

import pytest

from dimos.robot.drone.px4.gstreamer import pygi_gstreamer
from dimos.robot.drone.px4.gstreamer.gstreamer_api import (
    GstPipelineError,
    GstSourceConfig,
    GstTeePipelineSpec,
)


class FakeBus:
    def __init__(self) -> None:
        self.add_watch_calls = 0
        self.remove_watch_calls = 0

    def add_signal_watch(self) -> None:
        self.add_watch_calls += 1

    def remove_signal_watch(self) -> None:
        self.remove_watch_calls += 1

    def connect(self, _signal: str, _callback: Callable[..., None]) -> int:
        return 0


class FakeSink:
    def connect(self, _signal: str, _callback: Callable[..., int]) -> int:
        return 0


class FakePipeline:
    def __init__(self, bus: FakeBus, *, state_result: int = 0) -> None:
        self._bus = bus
        self._state_result = state_result

    def get_by_name(self, _name: str) -> FakeSink:
        return FakeSink()

    def get_bus(self) -> FakeBus:
        return self._bus

    def set_state(self, _state: int) -> int:
        return self._state_result


class FakeState:
    PLAYING = 1
    NULL = 0


class FakeStateChangeReturn:
    FAILURE = -1


class FakeMainContext:
    last_created: FakeMainContext | None = None

    def __init__(self) -> None:
        self.iterations: list[bool] = []
        self.wakeup_calls = 0
        self._wakeup = Event()

    @classmethod
    def new(cls) -> FakeMainContext:
        context = cls()
        cls.last_created = context
        return context

    def push_thread_default(self) -> None:
        return None

    def pop_thread_default(self) -> None:
        return None

    def iteration(self, may_block: bool) -> bool:
        self.iterations.append(may_block)
        if may_block:
            _ = self._wakeup.wait(1.0)
        return False

    def wakeup(self) -> None:
        self.wakeup_calls += 1
        self._wakeup.set()


class FakeGLib:
    MainContext = FakeMainContext


class FakeGst:
    State = FakeState
    StateChangeReturn = FakeStateChangeReturn

    def __init__(self, pipeline: FakePipeline) -> None:
        self._pipeline = pipeline

    def init(self, _value: None) -> None:
        return None

    def parse_launch(self, _description: str) -> FakePipeline:
        return self._pipeline


class FakeGi:
    def require_version(self, _name: str, _version: str) -> None:
        return None


def test_missing_gi_is_reported_as_a_typed_pipeline_error(monkeypatch: pytest.MonkeyPatch) -> None:
    # Given: a runtime without PyGObject installed.
    def missing_gi(_name: str) -> None:
        raise ModuleNotFoundError("gi")

    monkeypatch.setattr(pygi_gstreamer, "import_module", missing_gi)

    # When: the optional pipeline adapter is constructed.
    with pytest.raises(GstPipelineError, match="PyGObject gi"):
        pygi_gstreamer.PyGstPipeline(GstTeePipelineSpec(source=GstSourceConfig()))

    # Then: callers receive the typed camera-pipeline error.


def test_missing_gst_namespace_is_reported_as_a_typed_pipeline_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Given: PyGObject is available but its Gst namespace is absent.
    def import_without_gst(name: str) -> FakeGi:
        if name == "gi":
            return FakeGi()
        raise ModuleNotFoundError(name)

    monkeypatch.setattr(pygi_gstreamer, "import_module", import_without_gst)

    # When: the optional pipeline adapter is constructed.
    with pytest.raises(GstPipelineError, match="Gst namespace"):
        pygi_gstreamer.PyGstPipeline(GstTeePipelineSpec(source=GstSourceConfig()))

    # Then: missing Gst does not leak an import error.


def test_stop_removes_the_bus_watch_once(monkeypatch: pytest.MonkeyPatch) -> None:
    # Given: a fully fake PyGObject pipeline and bus.
    bus = FakeBus()
    pipeline = FakePipeline(bus)
    gi = FakeGi()
    gst = FakeGst(pipeline)

    def import_fake_runtime(name: str) -> FakeGi | FakeGst | FakeGLib:
        if name == "gi":
            return gi
        if name == "gi.repository.GLib":
            return FakeGLib()
        return gst

    monkeypatch.setattr(pygi_gstreamer, "import_module", import_fake_runtime)
    adapter = pygi_gstreamer.PyGstPipeline(GstTeePipelineSpec(source=GstSourceConfig()))

    # When: callback setup adds a bus watch and stop runs twice.
    adapter.set_callbacks(lambda _sample: None, lambda _sample: None, lambda _message: None)
    adapter.stop()
    adapter.stop()

    # Then: the watch is removed once and repeated teardown is safe.
    assert bus.add_watch_calls == 1
    assert bus.remove_watch_calls == 1


def test_start_failure_wakes_and_joins_the_context_thread(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bus = FakeBus()
    pipeline = FakePipeline(bus, state_result=FakeStateChangeReturn.FAILURE)
    gi = FakeGi()
    gst = FakeGst(pipeline)

    def import_fake_runtime(name: str) -> FakeGi | FakeGst | FakeGLib:
        if name == "gi":
            return gi
        if name == "gi.repository.GLib":
            return FakeGLib()
        return gst

    monkeypatch.setattr(pygi_gstreamer, "import_module", import_fake_runtime)
    adapter = pygi_gstreamer.PyGstPipeline(GstTeePipelineSpec(source=GstSourceConfig()))
    adapter.set_callbacks(lambda _sample: None, lambda _sample: None, lambda _message: None)

    with pytest.raises(GstPipelineError, match="failed to enter PLAYING"):
        adapter.start()

    context = FakeMainContext.last_created
    assert context is not None
    assert context.iterations == [False, True]
    assert context.wakeup_calls == 1
    assert adapter._main_context_thread is None
