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

from dimos.evals.suites.lib.space import commands
from dimos.evals.suites.lib.space.data import SpacePaths


@pytest.mark.parametrize("complete", [True, False])
def test_score_command_exit_status(
    tmp_path: Path, mocker: MockerFixture, capsys: pytest.CaptureFixture[str], complete: bool
) -> None:
    report = {"complete": complete, "official_percent": 0.0}
    score = mocker.patch.object(commands, "score_run", return_value=report)
    assert commands.main(["score", str(tmp_path)]) == (0 if complete else 1)
    score.assert_called_once_with(SpacePaths(), tmp_path)
    assert json.loads(capsys.readouterr().out) == report


def test_setup_command(mocker: MockerFixture, capsys: pytest.CaptureFixture[str]) -> None:
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGTERM, signal.SIGHUP)}
    setup = mocker.patch.object(commands, "setup", return_value={"selected_cases": []})
    assert commands.main(["setup"]) == 0
    assert {sig: signal.getsignal(sig) for sig in previous} == previous
    setup.assert_called_once_with(SpacePaths())
    assert json.loads(capsys.readouterr().out) == {"selected_cases": []}


@pytest.fixture
def command_process(tmp_path: Path) -> Iterator[tuple[subprocess.Popen[bytes], BinaryIO]]:
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
        script = (
            "from pathlib import Path\n"
            "import signal, sys\n"
            "from dimos.evals.suites.lib.space import commands\n"
            "from dimos.evals.suites.lib.space.process import run_process\n"
            "def setup(paths):\n"
            f"    run_process([sys.executable, '-c', {child!r}], Path({str(tmp_path / 'worker.log')!r}), 30)\n"
            "    return {}\n"
            "commands.setup = setup\n"
            "previous = {sig: signal.getsignal(sig) for sig in (signal.SIGTERM, signal.SIGHUP)}\n"
            "try:\n"
            "    raise SystemExit(commands.main(['setup']))\n"
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
