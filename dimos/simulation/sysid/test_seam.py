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

"""The seam stays engine-free, in every package that has one, and a TEST holds it.

The seam has leaked before: loop 2 was once implemented directly against
``mujoco.mj_step`` inside an above-seam module, invisible to an import-time
check because the import hid inside a function. Two locks close both routes:

* importing any above-seam module must not import an engine (clean
  interpreter, so no other test's imports mask it);
* the engine's NAME may appear in above-seam source only inside a ``main()``,
  the one place an entry point is allowed to choose its engine.

Every package with a seam is listed in ``SEAMS``: the shared method, and each
robot's plant package. A robot that adds a plant adds its entry here.
"""

from __future__ import annotations

import ast
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest

# package -> (above the seam, below the seam). A new module is above the seam
# by default: it must be added HERE, not exempted. Below the seam are the
# engine bindings, the only modules allowed to import an engine.
SEAMS: dict[str, tuple[tuple[str, ...], tuple[str, ...]]] = {
    "dimos.simulation.sysid": (
        (
            "backend",
            "fit",
            "identify",
            "plant",
            "presets",
            "recording",
            "regimes",
            "replay",
            "rollouts",
            "rotations",
            "score",
        ),
        ("engines.model", "engines.mujoco"),
    ),
    "dimos.robot.unitree.go2.sim": (
        (
            "anchors",
            "plant",
            "policy",
            "ranges",
            "sysid.compliance",
            "sysid.drive",
            "sysid.fit",
            "sysid.gait",
            "sysid.ground",
            "sysid.identify",
            "sysid.ingest",
            "sysid.loop",
            "sysid.meta",
            "sysid.probe",
            "sysid.real",
            "sysid.replay",
            "sysid.select",
            "sysid.stats",
            "sysid.verify_net",
        ),
        ("engines.model", "engines.mujoco", "engines.mjx", "engines.bench"),
    ),
    "dimos.robot.unitree.g1.sim": (
        ("plant", "ranges", "sysid.fit", "sysid.identify", "sysid.ingest", "sysid.replay"),
        # groot_mujoco is the closed-loop viewer script: an engine script by nature.
        ("engines.mujoco", "groot_mujoco", "model"),
    ),
}

# The engines themselves, and every below-seam module of every package: an
# import of one above any seam is an engine import in disguise.
ENGINE_MODULES = (
    "mujoco",
    "jax",
    *(f"{pkg}.{m}" for pkg, (_, below) in SEAMS.items() for m in below),
)

HOW_TO_FIX = (
    "Above the seam the engine arrives as an argument: a Backend, a "
    "ClosedLoopBackend, a Rollouts built around one, never as an import. "
    "Construct the engine in the CLI's main() (as the robot wrappers do), or "
    "put genuinely engine-specific code in engines/ behind the Backend or "
    "LoopSession protocol."
)


def _root(package: str) -> Path:
    spec = importlib.util.find_spec(package)
    assert spec is not None and spec.submodule_search_locations, package
    return Path(next(iter(spec.submodule_search_locations)))


@pytest.mark.parametrize("package", sorted(SEAMS))
def test_every_module_is_listed(package: str):
    """A module this file does not know about is a hole in the lock."""
    above, below = SEAMS[package]
    root = _root(package)
    found = {
        str(p.relative_to(root))[: -len(".py")].replace("/", ".")
        for p in root.rglob("*.py")
        if not p.name.startswith("test_")
    }
    assert found - set(below) == set(above), (
        f"{package}: module list out of date: unlisted {sorted(found - set(below) - set(above))}, "
        f"vanished {sorted(set(above) - found)}. New modules are above the seam "
        "by default: add them to SEAMS."
    )


@pytest.mark.parametrize("package", sorted(SEAMS))
def test_importing_the_seam_never_imports_an_engine(package: str):
    """With two backends installed, a seam that imports its engine makes using
    either import both. A clean interpreter proves it; in-process, some other
    test has already imported mujoco and the check would be vacuous."""
    above, _ = SEAMS[package]
    imports = "\n".join(f"import {package}.{m}" for m in above)
    code = (
        "import sys\n"
        f"{imports}\n"
        "banned = [m for m in sys.modules if m == 'mujoco' or m.startswith('mujoco.')]\n"
        f"assert not banned, f'the seam dragged in the engine: {{banned}}. ' {HOW_TO_FIX!r}\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


@pytest.mark.parametrize("package", sorted(SEAMS))
def test_engine_imports_hide_in_no_function_but_main(package: str):
    """The leak the import check cannot see: ``import mujoco`` inside a
    function body defers the import past the check and couples the module
    anyway, exactly how loop 2 originally leaked. Parse every above-seam
    module and allow engine imports ONLY inside a function named ``main``."""
    above, _ = SEAMS[package]
    root = _root(package)
    offences: list[str] = []
    for mod in above:
        offences += _engine_imports_outside_main(root / (mod.replace(".", "/") + ".py"))
    assert not offences, f"engine import outside main(): {offences}. {HOW_TO_FIX}"


def _engine_imports_outside_main(path: Path) -> list[str]:
    offences: list[str] = []

    def walk(node: ast.AST, fn_stack: tuple[str, ...]) -> None:
        for child in ast.iter_child_nodes(node):
            if isinstance(child, ast.FunctionDef | ast.AsyncFunctionDef):
                walk(child, (*fn_stack, child.name))
                continue
            names: list[str] = []
            if isinstance(child, ast.Import):
                names = [a.name for a in child.names]
            elif isinstance(child, ast.ImportFrom):
                # level > 0 is relative: `from .mujoco import ...` parses with
                # module="mujoco" and is OUR module, not the engine.
                names = [child.module or ""] if child.level == 0 else []
            for name in names:
                if any(name == e or name.startswith(e + ".") for e in ENGINE_MODULES):
                    if "main" not in fn_stack:
                        offences.append(f"{path.name}:{child.lineno} imports {name}")
            walk(child, fn_stack)

    walk(ast.parse(path.read_text(), filename=str(path)), ())
    return offences
