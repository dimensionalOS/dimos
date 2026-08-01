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

from subprocess import CompletedProcess

from typer.testing import CliRunner

from dimos.cli.can import app


def test_setup_configures_and_verifies_can_interface(mocker) -> None:
    mocker.patch("dimos.cli.can.os.geteuid", return_value=1000)
    run = mocker.patch(
        "dimos.cli.can.subprocess.run",
        return_value=CompletedProcess([], 0, stdout="4: follower_l: UP qlen 1000\n", stderr=""),
    )

    result = CliRunner().invoke(app, ["setup", "follower_l"])

    assert result.exit_code == 0
    assert "Running: sudo -- ip link set dev follower_l down" in result.stdout
    assert "bitrate=1000000, txqueuelen=1000" in result.stdout
    assert [call.args[0] for call in run.call_args_list] == [
        ["ip", "link", "show", "dev", "follower_l"],
        ["sudo", "--", "ip", "link", "set", "dev", "follower_l", "down"],
        [
            "sudo",
            "--",
            "ip",
            "link",
            "set",
            "dev",
            "follower_l",
            "type",
            "can",
            "bitrate",
            "1000000",
        ],
        ["sudo", "--", "ip", "link", "set", "dev", "follower_l", "txqueuelen", "1000"],
        ["sudo", "--", "ip", "link", "set", "dev", "follower_l", "up"],
        ["ip", "-details", "-statistics", "link", "show", "dev", "follower_l"],
    ]


def test_setup_rejects_nonpositive_queue_length() -> None:
    result = CliRunner().invoke(app, ["setup", "can0", "--txqueuelen", "0"])

    assert result.exit_code == 2
    assert "x>=1" in result.output
