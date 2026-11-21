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
import threading

import pytest

# CI runners are slower — give generous headroom but still catch gross regressions.
HELP_TIMEOUT_SECONDS = 8


@pytest.mark.parametrize(
    ("args", "env", "code", "word"),
    [
        (["--simulation=mujco", "show-config"], {}, 2, "mujco"),
        (["run", "coordinator-basic", "--simulation", "status", "--help"], {}, 2, "status"),
        (["show-config"], {"SIMULATION": "mujco"}, 2, "mujco"),
        (["--simulation", "TrUe", "show-config"], {}, 0, "simulation: mujoco"),
    ],
)
def test_simulator_selection(args: list[str], env: dict[str, str], code: int, word: str) -> None:
    result = subprocess.run(
        [sys.executable, "-c", "from dimos.cli.entry import main; main()", *args],
        env=os.environ | env,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == code
    assert word in result.stdout + result.stderr


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
    with subprocess.Popen(
        [sys.executable, "-m", "dimos.cli.dimos", "--help"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    ) as proc:
        assert proc.stdout is not None
        # Wall-clock safety net for a hung child only; it never decides pass/fail.
        killer = threading.Timer(60, proc.kill)
        killer.start()
        try:
            output = proc.stdout.read()  # EOF once the child has exited
        finally:
            killer.cancel()
        # wait4 reaps the child and hands back its resource usage, which
        # Popen.wait() would discard.
        _, status, usage = os.wait4(proc.pid, 0)
        proc.returncode = os.waitstatus_to_exitcode(status)
    cpu_seconds = usage.ru_utime + usage.ru_stime
    assert proc.returncode == 0, f"dimos --help failed:\n{output}"
    assert cpu_seconds < HELP_TIMEOUT_SECONDS, (
        f"dimos --help used {cpu_seconds:.1f}s of CPU (limit: {HELP_TIMEOUT_SECONDS}s). "
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
    # Importing the blueprint builds LCM handles. memq keeps them off the network.
    env = {**os.environ, "LCM_DEFAULT_URL": "memq://"}
    leaked = _leaked_modules(
        "import dimos.robot.unitree.go2.blueprints.smart.unitree_go2",
        BLUEPRINT_MUST_NOT_IMPORT,
        env=env,
    )
    assert leaked == []
