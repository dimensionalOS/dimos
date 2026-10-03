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

"""Check the macOS exec handoff without forking the pytest worker."""

from contextlib import suppress
import json
import os
from pathlib import Path
import sys
from unittest import mock

import pytest

from dimos.core.daemon import fork_daemon, write_daemon_status


class ExecBoundaryError(Exception):
    """Stand in for an exec call, which must not return to the forked image."""


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
def test_macos_grandchild_reexec_preserves_interpreter_and_arguments(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, original_argv: list[str]
) -> None:
    monkeypatch.setattr(sys, "platform", "darwin")
    monkeypatch.setattr(sys, "orig_argv", original_argv)
    monkeypatch.delenv("_DIMOS_DAEMON_STATUS_FD", raising=False)
    monkeypatch.setenv("DIMOS_RUN_ID", "launcher-run")
    read_fd, write_fd = os.pipe()
    try:
        with (
            mock.patch("os.pipe", return_value=(read_fd, write_fd)),
            mock.patch("os.fork", side_effect=[0, 0]) as fork,
            mock.patch("os.setsid") as setsid,
            mock.patch("os.execve", side_effect=ExecBoundaryError) as execve,
            pytest.raises(ExecBoundaryError),
        ):
            fork_daemon(tmp_path)
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


def test_linux_grandchild_retains_fork_without_exec(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.delenv("_DIMOS_DAEMON_STATUS_FD", raising=False)
    read_fd, write_fd = os.pipe()
    try:
        with (
            mock.patch("os.pipe", return_value=(read_fd, write_fd)),
            mock.patch("os.fork", side_effect=[0, 0]),
            mock.patch("os.setsid"),
            mock.patch("os.execve") as execve,
        ):
            assert fork_daemon(tmp_path) == (0, write_fd)
        execve.assert_not_called()
        assert not os.get_inheritable(write_fd)
    finally:
        with suppress(OSError):
            os.close(read_fd)
        os.close(write_fd)


def test_macos_exec_failure_does_not_continue_building(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(sys, "platform", "darwin")
    monkeypatch.delenv("_DIMOS_DAEMON_STATUS_FD", raising=False)
    read_fd, write_fd = os.pipe()
    try:
        with (
            mock.patch("os.pipe", return_value=(read_fd, write_fd)),
            mock.patch("os.fork", side_effect=[0, 0]),
            mock.patch("os.setsid"),
            mock.patch("os.execve", side_effect=OSError("exec unavailable")),
            pytest.raises(OSError, match="exec unavailable"),
        ):
            fork_daemon(tmp_path)
    finally:
        with suppress(OSError):
            os.close(read_fd)
        os.close(write_fd)
