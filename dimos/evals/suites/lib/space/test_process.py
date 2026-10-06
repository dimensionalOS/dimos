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

from __future__ import annotations

import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.evals.suites.lib.space.process import run_process


def test_completed_leader_cannot_leave_a_child_ignoring_term(tmp_path: Path) -> None:
    with socket.socket() as server:
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        server.settimeout(5)
        address = server.getsockname()
        child = "\n".join(
            [
                "import os, signal, socket, sys",
                "signal.signal(signal.SIGTERM, signal.SIG_IGN)",
                "connection = socket.socket()",
                f"connection.connect({address!r})",
                "connection.sendall(b'ready')",
                "os.write(int(sys.argv[1]), b'1')",
                "connection.recv(1)",
            ]
        )
        script = tmp_path / "leader.py"
        script.write_text(
            "import os, subprocess, sys\n"
            "reader, writer = os.pipe()\n"
            f"subprocess.Popen([sys.executable, '-c', {child!r}, str(writer)], pass_fds=(writer,))\n"
            "os.close(writer)\n"
            "assert os.read(reader, 1) == b'1'\n"
            "os.close(reader)\n"
            "print('Leader exits after child installed SIGTERM handler.', flush=True)\n"
        )
        assert run_process([sys.executable, str(script)], tmp_path / "process.log", 10) == 0
        connection, _ = server.accept()
        with connection:
            connection.settimeout(5)
            with connection.makefile("rb") as stream:
                assert stream.read(5) == b"ready"
                assert stream.read(1) == b""
    assert "Leader exits" in (tmp_path / "process.log").read_text()


@pytest.mark.parametrize("failure", [KeyboardInterrupt, subprocess.TimeoutExpired])
def test_failure_always_terminates_group_and_reaps(
    tmp_path: Path, mocker: MockerFixture, failure: type[BaseException]
) -> None:
    process = mocker.MagicMock()
    process.pid = 98765
    error: BaseException = KeyboardInterrupt()
    if failure is subprocess.TimeoutExpired:
        error = subprocess.TimeoutExpired("worker", 1)
    process.wait.side_effect = [error, 0, 0]
    popen = mocker.patch.object(subprocess, "Popen", return_value=process)
    kill = mocker.patch.object(os, "killpg")
    with pytest.raises(failure):
        run_process(["worker"], tmp_path / "process.log", 1)
    assert popen.call_args.kwargs["start_new_session"]
    assert kill.call_args_list == [
        mocker.call(98765, signal.SIGTERM),
        mocker.call(98765, signal.SIGKILL),
    ]
    assert process.wait.call_args_list == [
        mocker.call(timeout=1),
        mocker.call(timeout=5),
        mocker.call(timeout=5),
    ]
    assert popen.call_args.kwargs["stdout"].closed


def test_already_gone_group_is_still_reaped(tmp_path: Path, mocker: MockerFixture) -> None:
    process: Any = mocker.MagicMock()
    process.wait.return_value = 7
    process.pid = 98765
    mocker.patch.object(subprocess, "Popen", return_value=process)
    kill = mocker.patch.object(os, "killpg", side_effect=ProcessLookupError)
    assert run_process(["worker"], tmp_path / "process.log", 1) == 7
    assert kill.call_count == 2
    assert process.wait.call_count == 3
