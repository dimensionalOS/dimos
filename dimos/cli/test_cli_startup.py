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
from pathlib import Path
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
        # A bare flag given first leaves the following subcommand to run.
        (["--simulation", "run", "demo-mcp-stress-test", "--help"], {}, 0, "stresstestmodule"),
        (["--record", "show-config"], {}, 0, "record: sqlite"),
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


@pytest.mark.parametrize(
    ("setting", "command", "exit_code", "expected"),
    [
        ("N_WORKERS=[bad", "--help", 0, "Usage"),
        ("N_WORKERS=[bad", "mcp status --help", 0, "Usage"),
        ("N_WORKERS=[bad", "bake --help", 0, "--emit-config"),
        ("N_WORKERS=[bad", "top -- --help", 0, "dtop"),
        ("N_WORKERS=[bad", "--n-workers 3 show-config", 0, "n_workers: 3"),
        ("N_WORKERS=[bad", "evals list", 2, "n_workers"),
        ("N_WORKERS=[bad", "rerun-bridge --n-workers 3 --help", 0, "Blueprint"),
        ("N_WORKERS=[bad", "run coordinator-basic --help", 0, "Blueprint"),
        ("G__N_WORKERS=oops", "run coordinator-basic --help", 0, "Blueprint"),
        ("RECORD=mcap", "run coordinator-basic --help", 0, "Blueprint"),
        ("N_WORKERS=[bad", "run rerun-bridge-module --n-workers 3 --help", 0, "Blueprint"),
        ("SIMULATION=mujco", "--simulation dimsim show-config", 0, "simulation: dimsim"),
    ],
)
def test_cli_validates_environment_after_options(
    setting: str, command: str, exit_code: int, expected: str, tmp_path: Path
) -> None:
    key, value = setting.split("=", 1)
    result = subprocess.run(
        [sys.executable, "-c", "from dimos.cli.entry import main; main()", *command.split()],
        env={**os.environ, key: value},
        cwd=tmp_path,
        capture_output=True,
        text=True,
        timeout=30,
    )
    output = result.stdout + result.stderr
    assert result.returncode == exit_code, output
    assert expected in output


def test_rerun_bridge_uses_corrected_worker_count(tmp_path: Path) -> None:
    code = """
from dimos.cli.entry import main
from dimos.core.global_config import global_config
from dimos.visualization.rerun import bridge

def report(**options):
    print(global_config.n_workers, options["memory_limit"])

bridge.run_bridge = report
main()
"""
    result = subprocess.run(
        [sys.executable, "-c", code, "--n-workers", "4", "rerun-bridge", "--memory-limit", "4GB"],
        env={**os.environ, "N_WORKERS": "oops"},
        cwd=tmp_path,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    assert "4 4GB" in result.stdout.splitlines()


def test_graph_child_uses_effective_global_options(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # Capture the Graphviz input so this checks the real blueprint in its child process.
    dot = tmp_path / "dot"
    dot.write_text('#!/bin/sh\ncat > "$3"\n')
    dot.chmod(0o755)
    monkeypatch.setenv("PATH", str(tmp_path), prepend=os.pathsep)
    monkeypatch.setenv("N_WORKERS", "oops")
    monkeypatch.setenv("ROBOT_IPS", "")
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "dimos.cli.dimos",
            "--n-workers",
            "3",
            "--robot-ips",
            "127.0.0.1",
            "graph",
            "unitree-go2-multi",
        ],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    assert "GO2Connection" in (tmp_path / "unitree-go2-multi.svg").read_text()
