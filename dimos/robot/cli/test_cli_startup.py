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

"""Guard against import-time regressions in the CLI entrypoint.

`dimos --help` should never pull in heavy ML/viz libraries. If it does,
startup time balloons from <2s to >5s, which is a terrible UX.
"""

import subprocess
import sys
import time

# CI runners are slower — give generous headroom but still catch gross regressions.
HELP_TIMEOUT_SECONDS = 8


def test_help_does_not_import_heavy_deps() -> None:
    """GlobalConfig import must not drag in matplotlib, torch, or scipy."""
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys; "
                "from dimos.core.global_config import GlobalConfig; "
                "bad = [m for m in ('matplotlib', 'torch', 'scipy') if m in sys.modules]; "
                "assert not bad, f'Heavy deps imported: {bad}'"
            ),
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, f"Heavy deps leaked into GlobalConfig import:\n{result.stderr}"


def test_help_startup_time() -> None:
    """`dimos --help` must finish in under {HELP_TIMEOUT_SECONDS}s."""
    start = time.monotonic()
    result = subprocess.run(
        [sys.executable, "-m", "dimos.robot.cli.dimos", "--help"],
        capture_output=True,
        text=True,
        timeout=HELP_TIMEOUT_SECONDS + 5,  # hard kill safety margin
    )
    elapsed = time.monotonic() - start
    assert result.returncode == 0, f"dimos --help failed:\n{result.stderr}"
    assert elapsed < HELP_TIMEOUT_SECONDS, (
        f"dimos --help took {elapsed:.1f}s (limit: {HELP_TIMEOUT_SECONDS}s). "
        f"Check for heavy imports in the CLI entrypoint or GlobalConfig."
    )
