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

from collections.abc import Iterator

import pytest
from pytest_mock import MockerFixture
from structlog.testing import capture_logs

from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.global_config import GlobalConfig
from dimos.core.resource_monitor.monitor import StatsMonitor


@pytest.fixture
def worker_config() -> GlobalConfig:
    return GlobalConfig(n_workers=0, dtop=True)


@pytest.fixture
def worker_manager(
    worker_config: GlobalConfig, mocker: MockerFixture
) -> Iterator[WorkerManagerPython]:
    mocker.patch("dimos.core.transport_factory.make_transport", autospec=True)
    manager = WorkerManagerPython(worker_config)
    try:
        yield manager
    finally:
        manager.stop()


def test_dtop_logs_monitor_start_once(worker_manager: WorkerManagerPython) -> None:
    with capture_logs() as logs:
        worker_manager.start()
        worker_manager.start()

    assert [entry for entry in logs if entry["event"] == "Resource monitor started"] == [
        {
            "event": "Resource monitor started",
            "topic": "/resource_stats",
            "log_level": "info",
        }
    ]


def test_disabled_dtop_does_not_announce_monitor(
    worker_manager: WorkerManagerPython, worker_config: GlobalConfig
) -> None:
    worker_config.dtop = False

    with capture_logs() as logs:
        worker_manager.start()

    assert not any(entry["event"] == "Resource monitor started" for entry in logs)


def test_failed_monitor_start_does_not_announce_success(
    worker_manager: WorkerManagerPython, mocker: MockerFixture
) -> None:
    mocker.patch.object(StatsMonitor, "start", side_effect=RuntimeError("cannot start monitor"))

    with capture_logs() as logs, pytest.raises(RuntimeError, match="cannot start monitor"):
        worker_manager.start()

    assert not any(entry["event"] == "Resource monitor started" for entry in logs)
