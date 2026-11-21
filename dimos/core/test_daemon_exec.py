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

"""Check daemon startup and the fork/exec handoff."""

from contextlib import suppress
import errno
import json
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from unittest import mock

import pytest

from dimos.core.daemon import DAEMON_STATUS_FD_ENV, fork_daemon, write_daemon_status


def test_failed_daemon_releases_startup_resources(tmp_path: Path) -> None:
    # Wait for the re-executed child itself: the launcher returns on the status reply.
    command = [sys.executable, "-m", "dimos.cli.dimos", "--viewer", "none", "--n-workers", "1"]
    with TemporaryDirectory() as scratch, (tmp_path / "status").open("w+") as status:
        result = subprocess.run(
            [*command, "run", "replay-module", "--replaymodule.dataset=", "--daemon"],
            env=os.environ
            | {
                DAEMON_STATUS_FD_ENV: str(status.fileno()),
                "TMPDIR": scratch,
                "XDG_STATE_HOME": str(tmp_path),
                "XDG_CONFIG_HOME": str(tmp_path),
            },
            pass_fds=(status.fileno(),),
            capture_output=True,
            text=True,
            timeout=60,
        )
        status.seek(0)
        assert result.returncode == 1, result.stderr
        assert json.load(status)["ok"] is False
        assert not list((tmp_path / "dimos/cache-users").glob("*.lock"))
        assert not list(Path(scratch).glob("pymp-*"))


@pytest.mark.parametrize("operation", ["set_inheritable", "fork", "waitpid"])
def test_launcher_closes_status_pipe_on_error(tmp_path: Path, operation: str) -> None:
    pipe = os.pipe()
    try:
        with (
            mock.patch("os.pipe", return_value=pipe),
            mock.patch("os.fork", return_value=123),
            mock.patch(f"os.{operation}", side_effect=OSError(errno.EAGAIN, "unavailable")),
            pytest.raises(OSError),
        ):
            fork_daemon(tmp_path)
        for fd in pipe:
            with pytest.raises(OSError) as closed:
                os.fstat(fd)
            assert closed.value.errno == errno.EBADF
    finally:
        for fd in pipe:
            with suppress(OSError):
                os.close(fd)


def test_launcher_returns_status_pipe_for_auto_reaped_child(tmp_path: Path) -> None:
    with (
        mock.patch("os.fork", return_value=123),
        mock.patch("os.waitpid", side_effect=ChildProcessError(errno.ECHILD, "auto-reaped")),
    ):
        _, fd = fork_daemon(tmp_path)
    with os.fdopen(fd) as reader:
        assert reader.read() == ""


def test_reexecuted_daemon_consumes_status_pipe_once(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    read_fd, write_fd = os.pipe()
    try:
        os.set_inheritable(write_fd, True)
        monkeypatch.setenv("_DIMOS_DAEMON_STATUS_FD", str(write_fd))
        with mock.patch("os.fork", side_effect=AssertionError("must not fork twice")):
            assert fork_daemon(tmp_path) == (0, write_fd)
        assert "_DIMOS_DAEMON_STATUS_FD" not in os.environ
        assert not os.get_inheritable(write_fd)
        write_daemon_status(write_fd, {"ok": True})
        assert json.loads(os.read(read_fd, 100)) == {"ok": True}
    finally:
        os.close(read_fd)
        os.close(write_fd)


@pytest.mark.parametrize(
    "original_argv",
    [
        [
            "python",
            "-u",
            "-m",
            "dimos.cli.dimos",
            "--replay",
            "run",
            "unitree-go2-basic",
            "--daemon",
        ],
        [
            "python",
            "-X",
            "dev",
            "/workspace with spaces/bin/dimos",
            "run",
            "demo-mcp-stress-test",
            "--daemon",
        ],
    ],
)
def test_grandchild_reexec_preserves_interpreter_and_arguments(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, original_argv: list[str]
) -> None:
    monkeypatch.setattr(sys, "orig_argv", original_argv)
    monkeypatch.delenv("_DIMOS_DAEMON_STATUS_FD", raising=False)
    monkeypatch.setenv("DIMOS_RUN_ID", "launcher-run")
    read_fd, write_fd = os.pipe()
    try:
        with (
            mock.patch("os.pipe", return_value=(read_fd, write_fd)),
            mock.patch("os.fork", side_effect=[0, 0]) as fork,
            mock.patch("os.setsid") as setsid,
            mock.patch("os.execve", side_effect=OSError("exec unavailable")) as execve,
            mock.patch("dimos.core.daemon.write_daemon_status") as report,
            mock.patch("os._exit", side_effect=SystemExit(1)) as hard_exit,
            pytest.raises(SystemExit),
        ):
            fork_daemon(tmp_path)
        hard_exit.assert_called_once_with(1)
        report.assert_called_once_with(
            write_fd, {"ok": False, "error": "OSError: exec unavailable"}
        )
        assert fork.call_count == 2
        setsid.assert_called_once_with()
        executable, argv, env = execve.call_args.args
        assert executable == sys.executable
        assert argv == [sys.executable, *original_argv[1:]]
        assert env["DIMOS_RUN_ID"] == "launcher-run"
        assert env["_DIMOS_DAEMON_STATUS_FD"] == str(write_fd)
        assert os.get_inheritable(write_fd)
        assert "_DIMOS_DAEMON_STATUS_FD" not in os.environ
    finally:
        with suppress(OSError):
            os.close(read_fd)
        os.close(write_fd)
