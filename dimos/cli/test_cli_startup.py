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

import os
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


def test_cli_import_does_not_pull_ipython() -> None:
    """Importing the CLI must not drag in IPython (~1700 modules, ~1.3s)."""
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; import dimos.cli.dimos; assert 'IPython' not in sys.modules",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, f"IPython leaked into CLI import:\n{result.stderr}"


def test_help_startup_time() -> None:
    """`dimos --help` must finish in under {HELP_TIMEOUT_SECONDS}s."""
    start = time.monotonic()
    result = subprocess.run(
        [sys.executable, "-m", "dimos.cli.dimos", "--help"],
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


CLI_MUST_NOT_IMPORT = (
    "torch",
    "open3d",
    "cv2",
    "jax",
    "numba",
    "socketio",
    "mujoco",
    "onnxruntime",
    "rerun",
    "rclpy",
    "cyclonedds",
)
# aiortc and sounddevice are pulled in by unitree_webrtc_connect through
# dimos/robot/unitree/connection.py; deferring that import is a separate decision.
BLUEPRINT_MUST_NOT_IMPORT = (
    *CLI_MUST_NOT_IMPORT,
    "uvicorn",
    "mujoco_playground",
)


def _leaked_modules(
    import_statement: str, names: tuple[str, ...], env: dict[str, str] | None = None
) -> list[str]:
    """Run the import in a fresh interpreter and return which of the names it loaded."""
    code = (
        f"import sys; {import_statement}; print(' '.join(m for m in {names!r} if m in sys.modules))"
    )
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, timeout=120, env=env
    )
    assert result.returncode == 0, result.stderr
    return result.stdout.split()


def test_entry_module_does_not_import_the_cli() -> None:
    """Forkserver workers re-run the console script as __mp_main__; it must stay free."""
    assert _leaked_modules("import dimos.cli.entry", ("dimos.cli.dimos", "typer")) == []


def test_cli_import_does_not_pull_heavy_deps() -> None:
    assert _leaked_modules("import dimos.cli.dimos", CLI_MUST_NOT_IMPORT) == []


def test_go2_blueprint_import_does_not_pull_heavy_deps() -> None:
    # The blueprint builds an LCM() at import (vis_module); memq keeps it off the network.
    env = {**os.environ, "LCM_DEFAULT_URL": "memq://"}
    leaked = _leaked_modules(
        "import dimos.robot.unitree.go2.blueprints.smart.unitree_go2",
        BLUEPRINT_MUST_NOT_IMPORT,
        env=env,
    )
    assert leaked == []
