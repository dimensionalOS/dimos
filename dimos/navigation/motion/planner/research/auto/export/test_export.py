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

"""Self-test for the lab exporter.

Fast tier (default): export with --no-venv/--no-warm/--no-build into a
tmpdir under the ambient interpreter, then prove the tree, the trust chain,
and the tamper detection all work. Slow tier (needs cargo + uv): a real
build + 2-world battery — run manually or nightly:

    AUTORESEARCH_FULL_EXPORT=1 uv run pytest dimos/navigation/motion/planner/research/auto/export
"""

from __future__ import annotations

import json
from pathlib import Path
import py_compile
import shutil
import subprocess
import sys

import pytest

from dimos.navigation.motion.planner.research.auto.export.exporter import BENCH_TEMPLATES, HERE, run


@pytest.fixture(scope="module")
def lab(tmp_path_factory: pytest.TempPathFactory) -> Path:
    dest = tmp_path_factory.mktemp("lab") / "exported"
    run(dest, force=True, no_venv=True, no_warm=True, no_build=True)
    return dest


def test_templates_compile() -> None:
    for name in [*BENCH_TEMPLATES, "bench_guard.py"]:
        if name.endswith(".py"):
            py_compile.compile(str(HERE / "templates" / name), doraise=True)
    for name in ("run", "eval", "parity", "extcheck"):
        subprocess.run(["bash", "-n", str(HERE / "templates" / name)], check=True)


def test_tree_shape(lab: Path) -> None:
    for rel in (
        "README.md",
        "PROVENANCE.md",
        "gates.json",
        "pyproject.toml",
        ".gitignore",
        "eval",
        ".python-version",
        "referee/sim.py",
        "referee/scenarios.py",
        "referee/score.py",
        "referee/geometry.py",
        "referee/types.py",
        "referee/__main__.py",
        "referee/planners/target.py",
        "candidate/Cargo.toml",
        "candidate/Cargo.lock",
        "candidate/src/planner.rs",
        "candidate/src/python.rs",
        "candidate/src/lib.rs",
        "candidate/tests/invariants.rs",
        "bench/run",
        "bench/battery.py",
        "bench/fitness.py",
        "bench/quality.py",
        "bench/check_rules.py",
        "bench/ext_invariants.py",
        "bench/frozen.json",
        "bench/trust.lock",
        ".evo/bench_guard.py",
        ".evo/harness.lock",
        ".evo/referee.lock",
        ".evo/bench.env",
    ):
        assert (lab / rel).exists(), f"missing {rel}"
    # Nothing repo-internal leaks into the referee copy.
    for absent in ("referee/export", "referee/rust", "referee/test_gold.py"):
        assert not (lab / absent).exists(), f"{absent} should not be exported"
    assert (lab / ".git").exists(), "exported lab is a git repo"


def test_referee_imports_standalone(lab: Path) -> None:
    out = subprocess.run(
        [
            sys.executable,
            "-c",
            "import referee.sim, referee.scenarios, referee.score, referee.geometry; "
            "from referee.planners.base import REGISTRY; print(sorted(REGISTRY))",
        ],
        capture_output=True,
        text=True,
        check=True,
        cwd=lab,
        env={"PYTHONPATH": str(lab), "PATH": "/usr/bin:/bin"},
    )
    assert "gold" in out.stdout and "target" in out.stdout


def test_trust_chain_passes(lab: Path) -> None:
    subprocess.run(
        ["python3", str(lab / ".evo" / "bench_guard.py"), str(lab)],
        check=True,
        stdout=subprocess.DEVNULL,
    )
    subprocess.run(
        ["python3", str(lab / "bench" / "check_rules.py")],
        check=True,
        stdout=subprocess.DEVNULL,
    )


def test_gates_json_commands_reference_real_files(lab: Path) -> None:
    gates = json.loads((lab / "gates.json").read_text())
    assert gates["baseline"]["dq"] == 0
    for gate in gates["gates"]:
        for token in gate["command"].split():
            if "{worktree}/" in token or "{lab}/" in token:
                rel = token.split("}/", 1)[1]
                assert (lab / rel).exists(), f"{gate['name']}: {rel} missing"


def _copy_lab(lab: Path, tmp_path: Path) -> Path:
    clone = tmp_path / "clone"
    shutil.copytree(lab, clone)
    return clone


def test_tampered_referee_is_caught(lab: Path, tmp_path: Path) -> None:
    clone = _copy_lab(lab, tmp_path)
    target = clone / "referee" / "score.py"
    target.write_text(
        target.read_text().replace("SPEED_BUDGET_MS = 20.0", "SPEED_BUDGET_MS = 2000.0")
    )
    proc = subprocess.run(
        ["python3", str(clone / "bench" / "check_rules.py")],
        capture_output=True,
        text=True,
    )
    assert proc.returncode != 0
    assert "referee/score.py" in proc.stderr


def test_stray_file_beside_the_judge_is_caught(lab: Path, tmp_path: Path) -> None:
    clone = _copy_lab(lab, tmp_path)
    (clone / "referee" / "helper.py").write_text("# innocent-looking\n")
    proc = subprocess.run(
        ["python3", str(clone / "bench" / "check_rules.py")],
        capture_output=True,
        text=True,
    )
    assert proc.returncode != 0
    assert "stray file" in proc.stderr


def test_tampered_harness_is_caught(lab: Path, tmp_path: Path) -> None:
    clone = _copy_lab(lab, tmp_path)
    battery = clone / "bench" / "battery.py"
    battery.write_text(battery.read_text() + "\n# nudge\n")
    proc = subprocess.run(
        ["python3", str(clone / ".evo" / "bench_guard.py"), str(clone)],
        capture_output=True,
        text=True,
    )
    assert proc.returncode != 0
    assert "bench/battery.py" in proc.stderr


def test_banned_construct_in_candidate_is_caught(lab: Path, tmp_path: Path) -> None:
    clone = _copy_lab(lab, tmp_path)
    planner = clone / "candidate" / "src" / "planner.rs"
    planner.write_text(planner.read_text() + "\nuse std::fs::File;\n")
    proc = subprocess.run(
        ["python3", str(clone / "bench" / "check_rules.py")],
        capture_output=True,
        text=True,
    )
    assert proc.returncode != 0
    assert "filesystem access" in proc.stderr


def test_full_export_smoke(tmp_path: Path) -> None:
    """Real export: venv, warm caches, cargo build, 2-world battery, gates.

    Heavy (minutes): opt in with AUTORESEARCH_FULL_EXPORT=1."""
    import os

    if not os.environ.get("AUTORESEARCH_FULL_EXPORT"):
        pytest.skip("set AUTORESEARCH_FULL_EXPORT=1 to run the full export smoke")
    if shutil.which("cargo") is None or shutil.which("uv") is None:
        pytest.skip("needs cargo + uv")
    dest = tmp_path / "lab"
    run(dest, force=True)  # all stages, including the battery smoke
    subprocess.run(["bash", str(dest / "bench" / "extcheck")], check=True)
