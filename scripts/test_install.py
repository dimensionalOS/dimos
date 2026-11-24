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
import shutil
import subprocess
import sys

import pytest


@pytest.mark.parametrize("relative_root", [False, True])
def test_graph_smoke_check_runs_in_installed_project(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, relative_root: bool
) -> None:
    scripts = tmp_path / "checkout" / "scripts"
    scripts.mkdir(parents=True)
    shutil.copyfile(Path(__file__).with_name("test-install.sh"), scripts / "test-install.sh")
    # Replace installation with a no-op; this test only exercises the smoke harness.
    (scripts / "install.sh").write_text("exit 0\n")
    install_root = tmp_path / "installed project"
    project = install_root / "library"
    bin_dir = project / ".venv" / "bin"
    bin_dir.mkdir(parents=True)
    cli = bin_dir / "dimos"
    # Record the working directory at the requested graph output path.
    cli.write_text('#!/bin/bash\nset -eu\nprintf "%s\\n" "$PWD" > "$4"\n')
    cli.chmod(0o755)
    monkeypatch.setenv(
        "INSTALL_TEST_ROOT", "../installed project" if relative_root else str(install_root)
    )

    subprocess.run(
        ["bash", str(scripts / "test-install.sh"), "library"],
        cwd=scripts.parent,
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert (install_root / "logs" / "graph.svg").read_text().strip() == str(project)


@pytest.mark.parametrize(
    ("events", "exit_code", "passed"),
    [
        ([], None, False),
        (["Replay started"], None, False),
        (["Blueprint started"], None, False),
        (["Replay started", "Blueprint started"], None, True),
        ([], 0, False),
        (["Replay started", "Blueprint started"], 0, True),
        (["Replay started", "Blueprint started"], 2, False),
    ],
)
@pytest.mark.parametrize("log_level", ["INFO", "WARNING"])
def test_installer_smoke_test_requires_startup(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    events: list[str],
    exit_code: int | None,
    passed: bool,
    log_level: str,
) -> None:
    log_dir = tmp_path / "logs"
    log_dir.mkdir()
    monkeypatch.setenv("TMPDIR", str(log_dir))
    monkeypatch.setenv("DIMOS_LOG_LEVEL", log_level)
    bin_dir = tmp_path / ".venv" / "bin"
    bin_dir.mkdir(parents=True)
    (bin_dir / "python").symlink_to(sys.executable)
    cli = bin_dir / "dimos"
    cli.write_text(
        f"#!{sys.executable}\n"
        "import logging\n"
        "import os\n"
        "import signal\n"
        "import sys\n"
        "logger = logging.getLogger('smoke')\n"
        "logger.setLevel(os.environ.get('DIMOS_LOG_LEVEL', 'INFO'))\n"
        f"for index, event in enumerate({events!r}):\n"
        "    logger.handlers = [logging.StreamHandler([sys.stdout, sys.stderr][index % 2])]\n"
        "    logger.info(event)\n"
        f"exit_code = {exit_code!r}\n"
        "if exit_code is None:\n"
        "    signal.pause()\n"
        "sys.exit(exit_code)\n"
    )
    cli.chmod(0o755)

    result = subprocess.run(
        [
            "bash",
            "-c",
            """
source "$1"
INSTALL_DIR="$2"
INSTALL_DEPS=1
SKIP_TESTS=0
DRY_RUN=0
USE_NIX=0
EXTRAS=unitree
NON_INTERACTIVE=1
# Keep the real timeout and process cleanup, but shorten the test's deadline.
eval "$(declare -f run_bounded | sed '1s/run_bounded/run_bounded_original/')"
run_bounded() { shift; run_bounded_original 1 "$@"; }
run_post_install_tests
""",
            "smoke-test",
            str(Path(__file__).with_name("install.sh")),
            str(tmp_path),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert result.returncode == (0 if passed else 1), result.stdout + result.stderr
    assert ("smoke test passed" in result.stdout) == passed
    assert ("smoke test failed" in result.stderr) == (not passed)
    assert all(event in result.stdout + result.stderr for event in events)
    assert list(log_dir.iterdir()) == []
