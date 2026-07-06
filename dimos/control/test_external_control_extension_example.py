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

"""Tests for the logged external ControlCoordinator extension example."""

from __future__ import annotations

from collections.abc import Generator
import importlib
import logging
from pathlib import Path
import sys

import pytest

from dimos.control.tasks.registry import control_task_registry
from dimos.hardware.drive_trains.registry import twist_base_adapter_registry

EXAMPLE_ROOT = Path(__file__).parents[2] / "examples" / "external_control_extension"
EXAMPLE_PACKAGE = "dimos_external_control_extension"


@pytest.fixture(autouse=True)
def restore_external_example_state() -> Generator[None, None, None]:
    base_adapters = twist_base_adapter_registry._adapters.copy()
    task_paths = control_task_registry._factory_paths.copy()
    task_factories = control_task_registry._factories.copy()
    loaded_modules = {
        name: module
        for name, module in sys.modules.items()
        if name == EXAMPLE_PACKAGE or name.startswith(f"{EXAMPLE_PACKAGE}.")
    }
    try:
        yield
    finally:
        twist_base_adapter_registry._adapters = base_adapters
        control_task_registry._factory_paths = task_paths
        control_task_registry._factories = task_factories
        for name in list(sys.modules):
            if name == EXAMPLE_PACKAGE or name.startswith(f"{EXAMPLE_PACKAGE}."):
                del sys.modules[name]
        sys.modules.update(loaded_modules)


def test_external_control_extension_demo_logs_registration_and_runtime(
    caplog: pytest.LogCaptureFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.syspath_prepend(str(EXAMPLE_ROOT))
    caplog.set_level(logging.INFO, logger=EXAMPLE_PACKAGE)
    blueprints = importlib.import_module(f"{EXAMPLE_PACKAGE}.blueprints")

    blueprints.run_demo(target_writes=2, timeout=2.0)

    messages = "\n".join(caplog.messages)
    assert "[external_test_robot] registered hardware adapter: BASE/external_test_base" in messages
    assert "[external_test_robot] registered control task: external_test_drive" in messages
    assert "[external_test_robot] ExternalTestBaseAdapter.__init__" in messages
    assert "[external_test_robot] ExternalTestBaseAdapter.connect()" in messages
    assert "[external_test_robot] ExternalTestDriveTask created for task=drive_demo" in messages
    assert "[external_test_robot] ExternalTestDriveTask.compute(tick=1" in messages
    assert (
        "[external_test_robot] ExternalTestBaseAdapter.write_velocities([0.1, 0.0, 0.0])"
        in messages
    )
    assert "[external_test_robot] demo observed" in messages
