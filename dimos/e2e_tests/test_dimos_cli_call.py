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

import subprocess
import sys

import pytest

from dimos.e2e_tests import dimos_cli_call
from dimos.e2e_tests.dimos_cli_call import DimosCliCall, wait_for_ready


@pytest.fixture
def pending_call(mocker, tmp_path):
    call = DimosCliCall()
    call.demo_args = ["coordinator-mock"]
    call.log_path = tmp_path / "startup.log"
    call.process = mocker.Mock(spec=subprocess.Popen)
    call.process.pid = 123
    call.process.poll.return_value = None
    return call


@pytest.fixture
def startup_clock(mocker):
    clock = mocker.patch.object(dimos_cli_call, "time")
    clock.monotonic.return_value = 0.0
    return clock


def test_readiness_retries_with_one_deadline_and_closes_connection(
    pending_call, startup_clock, mocker
):
    connection = mocker.Mock()

    def connect(*, timeout):
        if startup_clock.monotonic.return_value < 2.0:
            startup_clock.monotonic.return_value += timeout
            raise TimeoutError("not ready")
        return connection

    probe = mocker.patch.object(dimos_cli_call.CoordinatorRPC, "connect", side_effect=connect)

    wait_for_ready(pending_call, timeout=2.5)

    assert [call.kwargs["timeout"] for call in probe.call_args_list] == [1.0, 1.0, 0.5]
    connection.stop.assert_called_once_with()


def test_readiness_fails_as_soon_as_the_child_exits(pending_call, startup_clock, mocker):
    def connect(*, timeout):
        pending_call.process.poll.return_value = 17
        startup_clock.monotonic.return_value += timeout
        raise TimeoutError("not ready")

    probe = mocker.patch.object(dimos_cli_call.CoordinatorRPC, "connect", side_effect=connect)

    with pytest.raises(RuntimeError, match="DimOS exited before blueprint readiness") as error:
        wait_for_ready(pending_call)

    assert "returncode=17" in str(error.value)
    assert "pid=123" in str(error.value)
    assert "elapsed=1.00s" in str(error.value)
    assert str(pending_call.log_path) in str(error.value)
    probe.assert_called_once_with(timeout=1.0)


def test_readiness_timeout_identifies_the_blueprint_and_log(pending_call, startup_clock, mocker):
    def connect(*, timeout):
        startup_clock.monotonic.return_value += timeout
        raise TimeoutError("not ready")

    mocker.patch.object(dimos_cli_call.CoordinatorRPC, "connect", side_effect=connect)

    with pytest.raises(TimeoutError, match="Blueprint startup timed out") as error:
        wait_for_ready(pending_call, timeout=2.5)

    assert startup_clock.monotonic.return_value == 2.5
    assert "coordinator-mock" in str(error.value)
    assert "elapsed=2.50s" in str(error.value)
    assert str(pending_call.log_path) in str(error.value)


def test_readiness_requires_a_started_process():
    with pytest.raises(RuntimeError, match="has not been started"):
        wait_for_ready(DimosCliCall())


def test_startup_failure_cleans_up_a_live_child(tmp_path, startup_clock, monkeypatch, mocker):
    popen = subprocess.Popen

    def launch_python(args, **kwargs):
        return popen([sys.executable, "-c", "import signal; signal.pause()"], **kwargs)

    monkeypatch.setenv("DIMOS_TEST_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(dimos_cli_call.subprocess, "Popen", launch_python)

    def connect(*, timeout):
        startup_clock.monotonic.return_value += timeout
        raise TimeoutError("not ready")

    mocker.patch.object(dimos_cli_call.CoordinatorRPC, "connect", side_effect=connect)
    # stop() measures real shutdown time, independently of the startup deadline.
    startup_clock.time.side_effect = [0.0, 0.1]
    call = DimosCliCall()
    call.demo_args = ["stalled-blueprint"]
    try:
        call.start()
        process = call.process
        assert process is not None
        with pytest.raises(TimeoutError, match="Blueprint startup timed out"):
            wait_for_ready(call, timeout=1.0)
    finally:
        call.stop()
    assert process.poll() is not None
    assert call.process is None


def test_cli_keeps_large_output_and_separate_logs_after_failure(tmp_path, monkeypatch):
    popen = subprocess.Popen

    def launch_python(args, **kwargs):
        return popen(
            [
                sys.executable,
                "-c",
                "import sys; print('x' * 131072); print('startup failed', file=sys.stderr); sys.exit(17)",
            ],
            **kwargs,
        )

    monkeypatch.setenv("DIMOS_TEST_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(dimos_cli_call.subprocess, "Popen", launch_python)
    paths = []
    for _ in range(2):
        call = DimosCliCall()
        call.demo_args = ["broken-blueprint"]
        try:
            call.start()
            assert call.process is not None
            assert call.process.wait(timeout=10) == 17
            with pytest.raises(RuntimeError, match="returncode=17"):
                wait_for_ready(call)
        finally:
            call.stop()
        assert call.process is None
        assert call.log_path is not None
        assert call.log_path.parent == tmp_path
        output = call.log_path.read_text()
        assert "x" * 131072 in output
        assert "startup failed" in output
        paths.append(call.log_path)
    assert paths[0] != paths[1]
