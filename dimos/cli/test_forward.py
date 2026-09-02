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

import os
from pathlib import Path
import re
import subprocess
import sys

import pytest
from typer.testing import CliRunner

from dimos.cli import forward as forward_cli
from dimos.cli.dimos import main

runner = CliRunner()

RESERVED_RS = Path(__file__).parents[2] / "installer" / "src" / "forward.rs"


@pytest.fixture(autouse=True)
def _no_guard():
    """Every test starts and ends with the recursion guard unset."""
    os.environ.pop(forward_cli.GUARD, None)
    yield
    os.environ.pop(forward_cli.GUARD, None)


@pytest.fixture
def installer(tmp_path, mocker):
    """An installer binary that exists, standing in for ~/.local/bin/dimos."""
    path = tmp_path / "dimos"
    path.write_text("#!/bin/sh\n")
    mocker.patch.object(forward_cli, "INSTALLER", path)
    return path


@pytest.mark.parametrize(
    "argv",
    [
        ["setup"],
        ["setup", "--agent"],
        ["update", "--dry-run"],
        ["service", "setup", "--env", "A=1"],
        ["uninstall"],
        ["robot", "scan", "--lan", "--timeout", "5"],
        ["hardware", "g1", "setup", "--robot-ip", "192.168.123.161", "--interface", "eth0"],
        ["hardware", "jetson", "setup", "--dry-run"],
    ],
)
def test_each_forwarded_verb_execs_the_installer_with_the_same_argv(argv, installer, mocker):
    execv = mocker.patch.object(forward_cli.os, "execv")

    result = runner.invoke(main, argv)

    assert result.exit_code == 0, result.output
    execv.assert_called_once_with(str(installer), [str(installer), *argv])


def test_missing_installer_exits_2_and_prints_the_install_line(tmp_path, mocker):
    absent = tmp_path / "dimos"
    mocker.patch.object(forward_cli, "INSTALLER", absent)
    execv = mocker.patch.object(forward_cli.os, "execv")

    result = runner.invoke(main, ["setup"])

    assert result.exit_code == 2
    assert str(absent) in result.stderr
    assert forward_cli.INSTALL_LINE in result.stderr
    execv.assert_not_called()


def test_forwarding_twice_exits_2_instead_of_looping(installer, mocker):
    execv = mocker.patch.object(forward_cli.os, "execv")
    os.environ[forward_cli.GUARD] = "1"

    result = runner.invoke(main, ["setup"])

    assert result.exit_code == 2
    assert forward_cli.INSTALL_LINE in result.stderr
    execv.assert_not_called()


def test_the_child_carries_the_guard_so_the_second_hop_can_refuse(installer, mocker):
    seen = {}
    mocker.patch.object(forward_cli.os, "execv", side_effect=lambda *_: seen.update(os.environ))

    runner.invoke(main, ["setup"])

    assert seen[forward_cli.GUARD] == "1"


def test_forwarded_verbs_match_the_rust_reserved_list():
    """This list is the contract between the two CLIs, so it is read from both sides."""
    literal = re.search(r"RESERVED: &\[&str\] = &\[(.*?)\]", RESERVED_RS.read_text(), re.S)
    assert literal, f"{RESERVED_RS} defines RESERVED"
    assert tuple(re.findall(r'"([^"]+)"', literal.group(1))) == forward_cli.FORWARDED


def test_importing_the_forwarder_pulls_no_heavy_deps():
    """`dimos setup` must reach exec without paying for the ML stack."""
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys; import dimos.cli.forward; "
                "bad = [m for m in ('matplotlib', 'torch', 'scipy') if m in sys.modules]; "
                "assert not bad, f'Heavy deps imported: {bad}'"
            ),
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
