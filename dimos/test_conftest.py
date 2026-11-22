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
    "options, expected_exit, expected_summary",
    [
        ([], 0, "collected 1 item / 1 skipped"),
        (["--error-for-skips"], 2, "1 test collected, 1 error"),
    ],
)
def test_collection_import_skips_respect_error_for_skips(
    tmp_path: Path, options: list[str], expected_exit: int, expected_summary: str
) -> None:
    (tmp_path / "pytest.ini").write_text("[pytest]\n")
    (tmp_path / "conftest.py").symlink_to(DIMOS_PROJECT_ROOT / "dimos/conftest.py")
    (tmp_path / "test_missing.py").write_text(
        'import pytest\npytest.importorskip("missing_ticket_010_dependency")\n'
    )
    (tmp_path / "test_present.py").write_text("def test_present():\n    assert True\n")

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
        env={**os.environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1", "PYTEST_ADDOPTS": ""},
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == expected_exit, result.stdout + result.stderr
    assert expected_summary in result.stdout
    assert "missing_ticket_010_dependency" in result.stdout
