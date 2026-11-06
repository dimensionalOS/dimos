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
import time
from types import SimpleNamespace
import uuid

import pytest

from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.core.run_registry import RunEntry
from dimos.evals.environments.lib.launch import launched_mcp_url, launched_run, wait_connectable


def _entry(run_id: str, pid: int, mcp_url: str | None) -> RunEntry:
    return RunEntry(
        run_id=run_id, pid=pid, blueprint="t", started_at="now", log_dir="l", mcp_url=mcp_url
    )


@pytest.fixture(autouse=True)
def _registry(mocker, tmp_path):
    mocker.patch("dimos.core.run_registry.REGISTRY_DIR", tmp_path)
    mocker.patch("dimos.evals.environments.lib.launch._REGISTRY_POLL_S", 0.01)


@pytest.mark.skipif_flaky_in_ci
def test_returns_the_endpoint_registered_by_the_launched_process() -> None:
    # Another live dimos on a custom port must not be mistaken for ours.
    _entry("20260101-000001-aaaa-other", os.getppid(), "http://127.0.0.1:9992/mcp").save()
    process = SimpleNamespace(pid=os.getpid(), poll=lambda: None)

    def register_late() -> None:
        _entry("20260101-000002-bbbb-ours", os.getpid(), "http://127.0.0.1:9993/mcp").save()

    import threading

    threading.Timer(0.05, register_late).start()
    assert launched_mcp_url(process, time.monotonic() + 5.0) == "http://127.0.0.1:9993/mcp"


def test_fails_when_the_run_has_no_mcp_server() -> None:
    _entry("20260101-000001-aaaa-ours", os.getpid(), None).save()
    with pytest.raises(RuntimeError, match="no McpServer"):
        launched_mcp_url(SimpleNamespace(pid=os.getpid(), poll=lambda: None), time.monotonic() + 5)


def test_fails_when_the_process_exits_or_the_deadline_passes() -> None:
    exited = SimpleNamespace(pid=os.getpid(), poll=lambda: 3, returncode=3)
    with pytest.raises(RuntimeError, match="exited with code 3"):
        launched_mcp_url(exited, time.monotonic() + 5)
    alive = SimpleNamespace(pid=os.getpid(), poll=lambda: None)
    with pytest.raises(TimeoutError, match="launch deadline"):
        launched_mcp_url(alive, time.monotonic())


def test_a_run_without_mcp_server_is_still_found() -> None:
    _entry("20260101-000001-aaaa-ours", os.getpid(), None).save()
    entry = launched_run(SimpleNamespace(pid=os.getpid(), poll=lambda: None), time.monotonic() + 5)
    assert entry.run_id == "20260101-000001-aaaa-ours" and entry.mcp_url is None


def test_wait_connectable_retries_until_the_coordinator_answers(mocker) -> None:
    client = mocker.Mock()
    connect = mocker.patch(
        "dimos.evals.environments.lib.launch.CoordinatorRPC.connect",
        side_effect=[TimeoutError(), client],
    )
    wait_connectable(time.monotonic() + 5)
    assert connect.call_count == 2
    client.stop.assert_called_once()


def test_wait_connectable_times_out_with_no_coordinator_running(mocker) -> None:
    # The real client: no coordinator must surface as a timeout, which is retried. A name no
    # other coordinator on the bus answers keeps a concurrent one from passing the ping.
    mocker.patch.object(CoordinatorRPC, "NAME", f"eval-test-{uuid.uuid4().hex}")
    with pytest.raises(TimeoutError, match="launch deadline"):
        wait_connectable(time.monotonic() + 1)


def test_wait_connectable_fails_when_dimos_exits_or_the_deadline_passes(mocker) -> None:
    mocker.patch(
        "dimos.evals.environments.lib.launch.CoordinatorRPC.connect", side_effect=TimeoutError()
    )
    exited = SimpleNamespace(poll=lambda: 3, returncode=3)
    with pytest.raises(RuntimeError, match="exited with code 3"):
        wait_connectable(time.monotonic() + 5, exited)
    with pytest.raises(TimeoutError, match="launch deadline"):
        wait_connectable(time.monotonic())
