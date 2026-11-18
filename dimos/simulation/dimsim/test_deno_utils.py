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

from pathlib import Path
import subprocess

import pytest
from pytest_mock import MockerFixture

from dimos.simulation.dimsim.deno_utils import ensure_playwright_chromium


def test_installed_chromium_skips_installer(tmp_path: Path, mocker: MockerFixture) -> None:
    executable = tmp_path / "custom browser cache" / "chrome"
    executable.parent.mkdir()
    executable.touch()
    mocker.patch("subprocess.check_output", return_value=f"{executable}\n")
    install = mocker.patch("subprocess.run")

    ensure_playwright_chromium("deno")
    ensure_playwright_chromium("deno")

    install.assert_not_called()


@pytest.mark.parametrize("path_is_directory", [False, True])
def test_missing_chromium_runs_installer(
    tmp_path: Path, mocker: MockerFixture, path_is_directory: bool
) -> None:
    executable = tmp_path / "chrome"
    if path_is_directory:
        executable.mkdir()
    mocker.patch("subprocess.check_output", return_value=f"{executable}\n")
    install = mocker.patch("subprocess.run")

    ensure_playwright_chromium("deno")

    install.assert_called_once_with(
        ["deno", "run", "--allow-all", "npm:playwright@1.58.2", "install", "chromium"],
        check=True,
    )


def test_browser_probe_failure_is_reported(mocker: MockerFixture) -> None:
    mocker.patch("subprocess.check_output", side_effect=subprocess.CalledProcessError(1, "deno"))
    install = mocker.patch("subprocess.run")

    with pytest.raises(subprocess.CalledProcessError):
        ensure_playwright_chromium("deno")

    install.assert_not_called()
