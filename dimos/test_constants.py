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
from pathlib import Path
import subprocess
import sys

import pytest

from dimos import DIMOS_PROJECT_ROOT


@pytest.mark.parametrize("checkout", [True, False], ids=["checkout", "installed"])
def test_working_files_use_the_checkout_or_state_directory(
    tmp_path: Path, checkout: bool, monkeypatch: pytest.MonkeyPatch
) -> None:
    (tmp_path / "dimos").symlink_to(DIMOS_PROJECT_ROOT / "dimos")
    if checkout:
        (tmp_path / ".git").mkdir()
    state_home = tmp_path / "state"
    monkeypatch.setenv("XDG_STATE_HOME", str(state_home))

    result = subprocess.check_output(
        [
            sys.executable,
            "-c",
            "import json\nfrom dimos import constants as c\n"
            "print(json.dumps([c.IS_CHECKOUT, c.WRITE_ROOT, c.LOG_DIR, c.RECORDINGS_DIR, "
            "c.DOWNLOADS_DIR], default=str))",
        ],
        cwd=tmp_path,
        text=True,
    )

    root = tmp_path if checkout else state_home / "dimos"
    assert json.loads(result) == [
        checkout,
        str(root),
        str(root / "logs"),
        str(root / "recordings"),
        str(root / "downloads"),
    ]
