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
import os
from pathlib import Path
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV
from dimos.core.module import Module
from dimos.hosted.daemon import HostDaemon, HostFragment, _run_fragment
from dimos.hosted.fragment import PythonFragmentPayload


class DaemonModule(Module):
    pass


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
        application_name="test-application",
        application_revision="revision-1",
        payload_digest=hashlib.sha256(payload).hexdigest(),
        payload=payload,
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
    assert descriptor.active_run_ids == ()


def test_start_is_idempotent_and_rejects_same_run_conflicts(
    host_setup: dict[str, Any],
) -> None:
    daemon = host_setup["daemon"]
    fragment = host_setup["fragment"]
    epoch = daemon.describe().epoch

    started = daemon.start(epoch, fragment)
    repeated = daemon.start(epoch, fragment)

    assert started.state == "running"
    assert started.run_id == "run-1"
    assert repeated == started
    host_setup["context"].Process.assert_called_once()
    with pytest.raises(RuntimeError, match="already running generation"):
        daemon.start(epoch, replace(fragment, generation=2))
    other_payload = b"different blueprint"
    with pytest.raises(ValueError, match="digest conflicts"):
        daemon.start(
            epoch,
            replace(
                fragment,
                payload_digest=hashlib.sha256(other_payload).hexdigest(),
                payload=other_payload,
            ),
        )


def test_runs_multiple_blueprints_and_stops_them_independently(
    host_setup: dict[str, Any], mocker: MockerFixture
) -> None:
    daemon = host_setup["daemon"]
    first_fragment = host_setup["fragment"]
    second_payload = b"another serialized blueprint"
    second_fragment = replace(
        first_fragment,
        run_id="run-2",
        payload_digest=hashlib.sha256(second_payload).hexdigest(),
        payload=second_payload,
    )
    first_process = host_setup["process"]
    second_process = mocker.MagicMock(pid=4343, exitcode=None)
    second_process.is_alive.return_value = True
    first_ready = host_setup["ready"]
    second_ready = mocker.MagicMock()
    second_ready.poll.return_value = True
    second_ready.recv.return_value = (True, None)
    first_send_ready = mocker.MagicMock()
    second_send_ready = mocker.MagicMock()
    host_setup["context"].Pipe.side_effect = [
        (first_ready, first_send_ready),
        (second_ready, second_send_ready),
    ]
    host_setup["context"].Process.side_effect = [first_process, second_process]
    epoch = daemon.describe().epoch

    first_status = daemon.start(epoch, first_fragment)
    second_status = daemon.start(epoch, second_fragment)

    assert first_status.state == "running"
    assert second_status.state == "running"
    assert daemon.describe().active_run_ids == ("run-1", "run-2")
    assert daemon.status(epoch, "run-1").pid == 4242
    assert daemon.status(epoch, "run-2").pid == 4343
    assert host_setup["context"].Process.call_count == 2

    first_process.is_alive.side_effect = [True, False]
    stopped = daemon.stop(epoch, "run-1", 1, first_fragment.payload_digest)

    assert stopped.state == "available"
    descriptor = daemon.describe()
    assert descriptor.state == "running"
    assert descriptor.active_run_ids == ("run-2",)
    assert daemon.status(epoch, "run-2").state == "running"
    first_process.terminate.assert_called_once_with()
    second_process.terminate.assert_not_called()


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


def test_start_validates_application_revision(host_setup: dict[str, Any]) -> None:
    fragment = host_setup["fragment"]
    daemon = HostDaemon(
        "host-1",
        versions={"application_revision": "revision-2"},
    )

    with pytest.raises(ValueError, match="application revision"):
        daemon.start(daemon.describe().epoch, fragment)


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


def test_shutdown_cleans_up_all_active_deployments(
    host_setup: dict[str, Any], mocker: MockerFixture
) -> None:
    daemon = host_setup["daemon"]
    first_fragment = host_setup["fragment"]
    second_payload = b"another serialized blueprint"
    second_fragment = replace(
        first_fragment,
        run_id="run-2",
        payload_digest=hashlib.sha256(second_payload).hexdigest(),
        payload=second_payload,
    )
    second_process = mocker.MagicMock(pid=4343, exitcode=None)
    second_process.is_alive.side_effect = [True, False]
    host_setup["context"].Process.side_effect = [host_setup["process"], second_process]
    host_setup["process"].is_alive.side_effect = [True, False]
    epoch = daemon.describe().epoch
    daemon.start(epoch, first_fragment)
    daemon.start(epoch, second_fragment)

    daemon.shutdown()

    assert daemon.describe().state == "available"
    assert daemon.describe().active_run_ids == ()
    host_setup["process"].terminate.assert_called_once_with()
    second_process.terminate.assert_called_once_with()
    assert host_setup["kill"].call_args_list == [mocker.call("run-1"), mocker.call("run-2")]


def test_run_fragment_loads_identity_bound_payload(
    tmp_path: Path,
    mocker: MockerFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    blueprint = DaemonModule.blueprint()
    config = BlueprintConfigParser(blueprint).parse(environ={})
    fragment = HostFragment.create(
        run_id="run-1",
        generation=1,
        host_id="host-1",
        application_name="test-application",
        application_revision="revision-1",
        payload=PythonFragmentPayload(blueprint=blueprint, config=config),
    )
    coordinator = mocker.MagicMock()
    coordinator.health_check.return_value = True
    build = mocker.patch(
        "dimos.hosted.daemon.ModuleCoordinator.build",
        return_value=coordinator,
    )
    stop_requested = mocker.MagicMock()
    mocker.patch("dimos.hosted.daemon.threading.Event", return_value=stop_requested)
    mocker.patch("dimos.hosted.daemon.signal.signal")
    mocker.patch("dimos.hosted.daemon.set_run_log_dir")
    ready = mocker.MagicMock()
    monkeypatch.setenv(DIMOS_RUN_ID_ENV, "previous")

    _run_fragment(fragment, tmp_path, ready)

    built_blueprint, built_config = build.call_args.args
    built_config.assert_matches(built_blueprint)
    coordinator.start_rpc_service.assert_called_once_with()
    stop_requested.wait.assert_called_once_with()
    ready.send.assert_called_once_with((True, None))
    coordinator.stop.assert_called_once_with()
    assert os.environ[DIMOS_RUN_ID_ENV] == "run-1"
