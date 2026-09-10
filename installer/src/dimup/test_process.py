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

import io
import os
import selectors
import signal
import subprocess
import sys
import time

import pytest
from rich.console import Console

from dimup.process import Runner, SetupError


def test_capture_separates_data_from_diagnostics(tmp_path, capsys):
    runner = Runner(tmp_path / "setup.log")
    with runner.stage("Resolve SDK"):
        result = runner.run(
            "Read metadata",
            [
                sys.executable,
                "-c",
                "import sys; print('sha'); print('diagnostic', file=sys.stderr)",
            ],
            capture=True,
        )
    assert result == "sha"
    assert "diagnostic" in runner.log.read_text()
    assert "sha" not in capsys.readouterr().out


def test_captured_failure_shows_diagnostic_and_exit_code(tmp_path):
    with pytest.raises(SetupError, match="exit 7") as error:
        Runner(tmp_path / "setup.log").run(
            "Read metadata",
            [
                sys.executable,
                "-c",
                "import sys; print('bad revision', file=sys.stderr); sys.exit(7)",
            ],
            capture=True,
        )
    assert "bad revision" in str(error.value)


def test_streams_large_output_literally_to_terminal_and_log(tmp_path, monkeypatch):
    monkeypatch.setenv("NO_COLOR", "1")
    console_output = io.StringIO()
    runner = Runner(tmp_path / "setup.log")
    runner.console = Console(file=console_output, width=30, no_color=True)
    runner.run(
        "Install",
        [
            sys.executable,
            "-c",
            "import sys; print('[red]literal[/red]'); sys.stderr.write('x' * 200000)",
        ],
    )
    terminal = console_output.getvalue()
    assert "[red]literal[/red]" in terminal
    assert "x" * 200000 in terminal
    assert "x" * 200000 in runner.log.read_text()
    assert "\x1b" not in terminal
    assert "\x1b" not in runner.log.read_text()


def test_missing_executable_identifies_command_and_log(tmp_path):
    log = tmp_path / "setup.log"
    with pytest.raises(SetupError) as error:
        Runner(log).run("Install", [str(tmp_path / "missing")])
    assert "Command:" in str(error.value)
    assert str(log) in str(error.value)


def read_until(process, expected):
    output = b""
    deadline = time.monotonic() + 15
    with selectors.DefaultSelector() as selector:
        selector.register(process.stdout, selectors.EVENT_READ)
        while expected not in output:
            remaining = deadline - time.monotonic()
            assert remaining > 0 and selector.select(remaining), output.decode()
            chunk = os.read(process.stdout.fileno(), 65536)
            assert chunk, output.decode()
            output += chunk
    return output


def runner_script(log, child):
    return (
        "from pathlib import Path; import sys; from dimup.process import Runner; "
        f"Runner(Path({str(log)!r})).run('Install', [sys.executable, '-c', {child!r}])"
    )


def test_output_arrives_before_child_finishes_even_without_newline(tmp_path):
    child = (
        "import os, sys, select; os.write(1, b'EARLY'); "
        "assert select.select([sys.stdin], [], [], 15)[0]; "
        "sys.stdin.readline(); os.write(2, b'LATE')"
    )
    with subprocess.Popen(
        [sys.executable, "-c", runner_script(tmp_path / "setup.log", child)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    ) as process:
        try:
            early = read_until(process, b"EARLY")
            assert process.poll() is None
        finally:
            late, errors = process.communicate(b"continue\n", timeout=20)
    assert process.returncode == 0, errors.decode()
    assert b"LATE" in early + late
    assert "EARLYLATE" in (tmp_path / "setup.log").read_text()


def test_interrupt_terminates_child_and_preserves_log(tmp_path):
    pid_file = tmp_path / "child.pid"
    child = (
        "import os, signal; from pathlib import Path; "
        f"Path({str(pid_file)!r}).write_text(str(os.getpid())); "
        "print('WAITING', flush=True); signal.pause()"
    )
    with subprocess.Popen(
        [sys.executable, "-c", runner_script(tmp_path / "setup.log", child)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    ) as process:
        try:
            read_until(process, b"WAITING")
        finally:
            process.send_signal(signal.SIGINT)
            process.communicate(timeout=15)
    assert process.returncode != 0
    with pytest.raises(ProcessLookupError):
        os.kill(int(pid_file.read_text()), 0)
    assert "Interrupted" in (tmp_path / "setup.log").read_text()
