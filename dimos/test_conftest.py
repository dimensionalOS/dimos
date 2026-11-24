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
import subprocess
import sys

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT


@pytest.mark.parametrize(
    "options, missing_marker, expected_exit, expected_summary",
    [
        ([], "web_browser", 0, "collected 1 item / 1 skipped"),
        (["--error-for-skips"], "web_browser", 2, "1 test collected, 1 error"),
        # -m cannot filter a module that skipped at import, so its source decides.
        (
            ["--error-for-skips", "-m", "self_hosted_large"],
            "web_browser",
            0,
            "collected 1 item / 1 skipped",
        ),
        (
            ["--error-for-skips", "-m", "self_hosted_large"],
            "self_hosted_large",
            2,
            "1 test collected, 1 error",
        ),
    ],
)
def test_collection_import_skips_respect_error_for_skips(
    tmp_path: Path,
    options: list[str],
    missing_marker: str,
    expected_exit: int,
    expected_summary: str,
) -> None:
    (tmp_path / "pytest.ini").write_text("[pytest]\n")
    (tmp_path / "conftest.py").symlink_to(DIMOS_PROJECT_ROOT / "dimos/conftest.py")
    (tmp_path / "test_missing.py").write_text(
        "import pytest\n"
        f"pytestmark = pytest.mark.{missing_marker}\n"
        'pytest.importorskip("missing_ticket_010_dependency")\n'
    )
    (tmp_path / "test_present.py").write_text(
        "import pytest\n"
        "pytestmark = pytest.mark.self_hosted_large\n"
        "def test_present():\n    assert True\n"
    )

    # This is a new pytest controller, not an outer xdist worker. Give it its
    # own run ID so its watchdog cannot sweep the parent session's workers.
    env = {key: value for key, value in os.environ.items() if not key.startswith("PYTEST_XDIST_")}
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "pytest",
            "-p",
            "pytest_error_for_skips",
            "--collect-only",
            "--color=no",
            "-ra",
            *options,
        ],
        cwd=tmp_path,
        env={**env, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1", "PYTEST_ADDOPTS": ""},
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == expected_exit, result.stdout + result.stderr
    assert expected_summary in result.stdout
    assert "missing_ticket_010_dependency" in result.stdout
