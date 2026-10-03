# Copyright 2025-2026 Dimensional Inc.
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

import json
import os
import subprocess
import sys

from typer.testing import CliRunner

from dimos.cli.commands.network import network_app
from dimos.cli.network.model import Report, Settings


def test_json_stdout_is_parseable_and_progress_is_stderr(mocker):
    report = Report(
        "a" * 32,
        "host",
        "/bin/dimos",
        Settings(),
        "1.10.1",
        status="completed",
        cleanup="confirmed",
    )

    def run(*args, **kwargs):
        kwargs["progress"]("measuring example", report)
        return report

    mocker.patch("dimos.cli.commands.network.run_check", side_effect=run)
    result = CliRunner().invoke(
        network_app, ["check", "host", "--remote-dimos", "/bin/dimos", "--json"]
    )
    assert result.exit_code == 0, result.exception
    assert json.loads(result.stdout)["status"] == "completed"
    assert "measuring example" in result.stderr
    assert "\x1b[" not in result.stdout


def test_invalid_rate_does_not_launch_ssh(mocker):
    runner = mocker.patch("dimos.cli.commands.network.run_check")
    result = CliRunner().invoke(
        network_app, ["check", "host", "--remote-dimos", "/bin/dimos", "--max-mbps", "-1"]
    )
    assert result.exit_code == 2
    runner.assert_not_called()


def test_peer_is_not_a_public_manual_workflow():
    result = CliRunner().invoke(network_app, ["peer"])
    assert result.exit_code == 2
    assert "internal" in result.output


def test_actual_entrypoint_two_processes_with_only_ssh_launch_stub(tmp_path, monkeypatch):
    # This replaces only SSH transport; both child CLI and Zenoh TCP are real.
    remote = tmp_path / "remote environment" / "dimos"
    remote.parent.mkdir()
    remote.write_text(
        f"#!{sys.executable}\nfrom dimos.cli.entrypoint import cli_main\ncli_main()\n"
    )
    remote.chmod(0o755)
    ssh = tmp_path / "ssh"
    ssh.write_text(
        f"#!{sys.executable}\nimport os, shlex, sys\n"
        "args = shlex.split(sys.argv[-1])\nassert args[0] == 'exec'\n"
        "os.execv(args[1], args[1:])\n"
    )
    ssh.chmod(0o755)
    monkeypatch.setenv("PATH", str(tmp_path) + os.pathsep + os.environ["PATH"])
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "from dimos.cli.entrypoint import cli_main; cli_main()",
            "network",
            "check",
            "127.0.0.1",
            "--remote-dimos",
            str(remote),
            "--listen-host",
            "127.0.0.1",
            "--max-mbps",
            "1",
            "--step-seconds",
            ".3",
            "--idle-seconds",
            ".1",
            "--payload-bytes",
            "1024",
            "--max-seconds",
            "20",
            "--json",
        ],
        capture_output=True,
        text=True,
        timeout=25,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report["cleanup"] == "confirmed"
    assert report["directions"]["local_to_remote"][-1]["receiver"]["unique_received"] > 0
    assert "Starting SSH-owned peer" in result.stderr


def test_legacy_dispatch_does_not_preload_network_or_native_dependencies():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys, types; "
            "from dimos.cli.entrypoint import cli_main; "
            "assert 'zenoh' not in sys.modules; "
            "sys.modules['dimos.cli.dimos'] = types.SimpleNamespace(cli_main=lambda: None); "
            "sys.argv = ['dimos', '--help']; cli_main(); "
            "assert 'dimos.cli.commands.network' not in sys.modules; "
            "assert 'zenoh' not in sys.modules",
        ],
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )
    assert result.returncode == 0, result.stderr
