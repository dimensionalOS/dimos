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
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
from typing import BinaryIO

import pytest
from pytest_mock import MockerFixture
from typer.testing import CliRunner

from dimos.cli.dimos import main as app
from dimos.evals.suites.lib.space import data, report
from dimos.evals.suites.lib.space.data import SpacePaths


@pytest.mark.parametrize("complete", [True, False])
def test_score_command_exit_status(tmp_path: Path, mocker: MockerFixture, complete: bool) -> None:
    expected = {"complete": complete, "official_percent": 0.0}
    score = mocker.patch.object(report, "score_run", return_value=expected)
    result = CliRunner().invoke(app, ["evals", "score", "space", str(tmp_path)])
    assert result.exit_code == (0 if complete else 1), result.output
    score.assert_called_once_with(SpacePaths(), tmp_path)
    assert json.loads(result.output) == expected


def test_setup_command(mocker: MockerFixture) -> None:
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGTERM, signal.SIGHUP)}
    setup = mocker.patch.object(data, "setup", return_value={"selected_cases": []})
    result = CliRunner().invoke(app, ["evals", "setup", "space"])
    assert result.exit_code == 0, result.output
    assert {sig: signal.getsignal(sig) for sig in previous} == previous
    setup.assert_called_once_with(SpacePaths())
    assert json.loads(result.output) == {"selected_cases": []}


def test_setup_failure_restores_handlers(mocker: MockerFixture) -> None:
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGTERM, signal.SIGHUP)}
    failure = RuntimeError("Data download failed")
    mocker.patch.object(data, "setup", side_effect=failure)
    result = CliRunner().invoke(app, ["evals", "setup", "space"])
    assert result.exit_code == 1
    assert result.exception is failure
    assert {sig: signal.getsignal(sig) for sig in previous} == previous


@pytest.mark.parametrize("args", [["setup", "unknown"], ["score", "unknown"], ["score", "space"]])
def test_invalid_command_does_not_run_backend(args: list[str], mocker: MockerFixture) -> None:
    setup = mocker.patch.object(data, "setup")
    score = mocker.patch.object(report, "score_run")
    result = CliRunner().invoke(app, ["evals", *args])
    assert result.exit_code == 2
    setup.assert_not_called()
    score.assert_not_called()


@pytest.fixture(params=["setup", "score"])
def command_process(
    tmp_path: Path, request: pytest.FixtureRequest
) -> Iterator[tuple[subprocess.Popen[bytes], BinaryIO]]:
    with socket.socket() as server:
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        server.settimeout(10)
        child = (
            "import os, socket\n"
            f"connection = socket.create_connection({server.getsockname()!r})\n"
            "connection.sendall(f'{os.getpid()}\\n'.encode())\n"
            "connection.recv(1)\n"
        )
        args = ["evals", request.param, "space"]
        if request.param == "score":
            args.append(str(tmp_path))
        script = (
            "from pathlib import Path\n"
            "import signal, sys\n"
            "from dimos.cli.dimos import main as app\n"
            "from dimos.evals.suites.lib.space import data, report\n"
            "from dimos.evals.suites.lib.space.process import run_process\n"
            "def operation(paths, *args):\n"
            f"    run_process([sys.executable, '-c', {child!r}], Path({str(tmp_path / 'worker.log')!r}), 30)\n"
            "    return {}\n"
            "data.setup = report.score_run = operation\n"
            "previous = {sig: signal.getsignal(sig) for sig in (signal.SIGTERM, signal.SIGHUP)}\n"
            "try:\n"
            f"    app({args!r})\n"
            "finally:\n"
            "    assert {sig: signal.getsignal(sig) for sig in previous} == previous\n"
        )
        process = subprocess.Popen([sys.executable, "-c", script])
        child_pid = None
        try:
            connection, _ = server.accept()
            with connection, connection.makefile("rb") as stream:
                connection.settimeout(10)
                child_pid = int(stream.readline())
                yield process, stream
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            finally:
                process.kill()
                process.wait(timeout=5)
                if child_pid is not None:
                    try:
                        os.killpg(child_pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass


@pytest.mark.parametrize("sig", [signal.SIGTERM, signal.SIGHUP])
def test_termination_signal_cleans_command_worker(
    command_process: tuple[subprocess.Popen[bytes], BinaryIO], sig: signal.Signals
) -> None:
    process, stream = command_process
    process.send_signal(sig)
    assert process.wait(timeout=10) == 128 + sig
    assert stream.read(1) == b""
