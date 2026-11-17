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

import os
from pathlib import Path
import threading
from typing import Any

from pytest_mock import MockerFixture

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module


class RecordedTeardown(Module):
    """Record every stop attempt, including concurrent calls that return immediately."""

    def __init__(self, record: str, **kwargs: Any) -> None:
        self._record = record
        self._stop_started = False
        self._stop_lock = threading.Lock()
        super().__init__(**kwargs)

    def _note(self, event: str) -> None:
        with open(self._record, "a") as record:
            record.write(f"{event} {os.getpid()}:{id(self)}\n")

    @rpc
    def build(self) -> None:
        self._note("build")

    @rpc
    def stop(self) -> None:
        self._note("stop")
        with self._stop_lock:
            if self._stop_started:
                return
            self._stop_started = True
        super().stop()
        self._note("stopped")


class Companion(Module):
    """Keep the old worker alive after its recorded module is unloaded."""


def test_restart_stops_old_instance_once_before_replacement_build(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    record = tmp_path / "events"
    coordinator = ModuleCoordinator(g=GlobalConfig(n_workers=1, viewer="none"))
    coordinator.start()
    try:
        coordinator.load_blueprint(
            autoconnect(RecordedTeardown.blueprint(record=str(record)), Companion.blueprint())
        )
        old_proxy = coordinator.get_instance(RecordedTeardown)
        stop_rpc = mocker.spy(old_proxy, "stop")

        coordinator.restart_module(RecordedTeardown, reload_source=False)
        stop_rpc.assert_not_called()
    finally:
        coordinator.stop()

    events = [line.split() for line in record.read_text().splitlines()]
    builds = [instance for event, instance in events if event == "build"]
    assert len(builds) == 2 and builds[0] != builds[1], events
    assert [event for event, instance in events if instance == builds[0]] == [
        "build",
        "stop",
        "stopped",
    ], events
    assert ["stopped", builds[0]] in events[: events.index(["build", builds[1]])], events
