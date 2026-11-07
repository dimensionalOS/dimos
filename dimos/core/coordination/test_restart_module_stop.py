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
import time
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module


class SlowTeardown(Module):
    """stop() takes a while and a concurrent stop() returns at once, like NativeModule.stop()."""

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
        with self._stop_lock:
            if self._stop_started:
                return
            self._stop_started = True
        time.sleep(1.5)
        super().stop()
        self._note("stopped")


class Companion(Module):
    """Shares the worker, so unloading SlowTeardown does not shut the worker down."""


def test_restart_waits_for_the_old_instance_to_stop(tmp_path: Path) -> None:
    """The old instance is gone before the replacement is built.

    Both answer to the same RPC name, so an old instance that is still stopping
    would handle the replacement's build() as well.
    """
    record = tmp_path / "events"
    coordinator = ModuleCoordinator(g=GlobalConfig(n_workers=1, viewer="none"))
    coordinator.start()
    try:
        coordinator.load_blueprint(
            autoconnect(SlowTeardown.blueprint(record=str(record)), Companion.blueprint())
        )
        coordinator.restart_module(SlowTeardown, reload_source=False)
    finally:
        coordinator.stop()

    events = [line.split() for line in record.read_text().splitlines()]
    builds = [instance for event, instance in events if event == "build"]
    assert len(builds) == 2 and builds[0] != builds[1], events
    assert ["stopped", builds[0]] in events[: events.index(["build", builds[1]])], events
