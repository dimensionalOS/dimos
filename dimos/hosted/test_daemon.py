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

from dataclasses import replace
import hashlib
from pathlib import Path
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.hosted.daemon import HostDaemon, HostFragment


@pytest.fixture
def host_setup(tmp_path: Path, mocker: MockerFixture) -> dict[str, Any]:
    process = mocker.MagicMock(pid=4242, exitcode=None)
    process.is_alive.return_value = True
    ready = mocker.MagicMock()
    ready.poll.return_value = True
    ready.recv.return_value = (True, None)

    context = mocker.MagicMock()
    context.Pipe.return_value = (ready, mocker.MagicMock())
    context.Process.return_value = process
    mocker.patch("dimos.hosted.daemon.multiprocessing.get_context", return_value=context)
    kill_run_processes = mocker.patch("dimos.hosted.daemon.kill_run_processes")

    payload = b"serialized blueprint"
    fragment = HostFragment(
        run_id="run-1",
        generation=1,
        host_id="host-1",
        payload_digest=hashlib.sha256(payload).hexdigest(),
        blueprint_payload=payload,
    )
    daemon = HostDaemon(
        "host-1",
        name="lab-host",
        tags={"gpu"},
        versions={"protocol": 1},
        log_root=tmp_path,
    )
    return {
        "daemon": daemon,
        "fragment": fragment,
        "process": process,
        "context": context,
        "ready": ready,
        "kill": kill_run_processes,
    }


def test_describe_reports_available_host(host_setup: dict[str, Any]) -> None:
    descriptor = host_setup["daemon"].describe()

    assert descriptor.host_id == "host-1"
    assert descriptor.tags == frozenset({"gpu"})
    assert descriptor.versions == {"protocol": 1}
    assert descriptor.state == "available"
    assert descriptor.active_run_id is None


def test_start_is_idempotent_and_rejects_conflicts(host_setup: dict[str, Any]) -> None:
    daemon = host_setup["daemon"]
    fragment = host_setup["fragment"]
    epoch = daemon.describe().epoch

    started = daemon.start(epoch, fragment)
    repeated = daemon.start(epoch, fragment)

    assert started.state == "running"
    assert started.run_id == "run-1"
    assert repeated == started
    host_setup["context"].Process.assert_called_once()
    with pytest.raises(RuntimeError, match="is running"):
        daemon.start(epoch, replace(fragment, run_id="run-2"))
    other_payload = b"different blueprint"
    with pytest.raises(ValueError, match="digest conflicts"):
        daemon.start(
            epoch,
            replace(
                fragment,
                payload_digest=hashlib.sha256(other_payload).hexdigest(),
                blueprint_payload=other_payload,
            ),
        )


def test_start_validates_epoch_host_and_digest(host_setup: dict[str, Any]) -> None:
    daemon = host_setup["daemon"]
    fragment = host_setup["fragment"]
    epoch = daemon.describe().epoch

    with pytest.raises(ValueError, match="epoch"):
        daemon.start("stale", fragment)
    with pytest.raises(ValueError, match="targets Host"):
        daemon.start(epoch, replace(fragment, host_id="host-2"))
    with pytest.raises(ValueError, match="digest"):
        daemon.start(epoch, replace(fragment, payload_digest="wrong"))


def test_status_marks_an_exited_process_failed(host_setup: dict[str, Any]) -> None:
    daemon = host_setup["daemon"]
    fragment = host_setup["fragment"]
    epoch = daemon.describe().epoch
    daemon.start(epoch, fragment)
    host_setup["process"].exitcode = 7

    status = daemon.status(epoch, fragment.run_id)

    assert status.state == "failed"
    assert status.error == "Deployment process exited with code 7"


def test_start_reports_child_failure(host_setup: dict[str, Any]) -> None:
    daemon = host_setup["daemon"]
    fragment = host_setup["fragment"]
    host_setup["ready"].recv.return_value = (False, "module failed")
    host_setup["process"].is_alive.return_value = False

    status = daemon.start(daemon.describe().epoch, fragment)

    assert status.state == "failed"
    assert status.error == "module failed"
    host_setup["kill"].assert_called_once_with(fragment.run_id)


def test_stop_releases_host(host_setup: dict[str, Any]) -> None:
    daemon = host_setup["daemon"]
    fragment = host_setup["fragment"]
    epoch = daemon.describe().epoch
    daemon.start(epoch, fragment)
    host_setup["process"].is_alive.side_effect = [True, False]

    status = daemon.stop(epoch, fragment.run_id, 1, fragment.payload_digest)

    assert status.state == "available"
    host_setup["process"].terminate.assert_called_once_with()
    host_setup["kill"].assert_called_once_with(fragment.run_id)
