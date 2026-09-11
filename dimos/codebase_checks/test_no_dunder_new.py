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

import ast
from pathlib import Path

from dimos.constants import DIMOS_PROJECT_ROOT

# Calls that match but are legitimate. Last resort — construct the object
# properly instead whenever possible.
WHITELIST = [
    # Pure threading-logic test; a real engine would need a MuJoCo model
    # compile and the `mujoco` marker, dropping it from the default CI lane.
    ("dimos/simulation/engines/test_mujoco_sim_module.py", "object.__new__(MujocoEngine)"),
]


def _is_whitelisted(rel_path: str, line: str) -> bool:
    for allowed_path, allowed_substr in WHITELIST:
        if rel_path == allowed_path and allowed_substr in line:
            return True
    return False


def find_dunder_new_calls() -> list[tuple[Path, int, str]]:
    """Return (file, line_number, line_text) for every `__new__` call in test files."""
    dimos_dir = DIMOS_PROJECT_ROOT / "dimos"
    hits: list[tuple[Path, int, str]] = []
    for path in sorted(dimos_dir.rglob("*.py")):
        if not (path.name.startswith("test_") or path.name == "conftest.py"):
            continue
        source = path.read_text(encoding="utf-8")
        lines = source.splitlines()
        rel_path = str(path.relative_to(DIMOS_PROJECT_ROOT))
        for node in ast.walk(ast.parse(source)):
            if not (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "__new__"
            ):
                continue
            line = lines[node.lineno - 1]
            if not _is_whitelisted(rel_path, line):
                hits.append((path, node.lineno, line))
    return hits


def test_no_dunder_new() -> None:
    """Fail if any test file calls `__new__` to bypass `__init__`."""
    dimos_dir = DIMOS_PROJECT_ROOT / "dimos"
    hits = find_dunder_new_calls()
    if hits:
        listing = "\n".join(
            f"  - {p.relative_to(dimos_dir)}:{lineno}: {line.strip()}" for p, lineno, line in hits
        )
        raise AssertionError(
            f"Found __new__ call(s) in test files:\n{listing}\n\n"
            "Tests must construct objects with the real constructor: __init__ is "
            "code under test too, and an object assembled by hand silently rots "
            "when the constructor changes. If __init__ does heavy work, mock the "
            "collaborators it needs instead of skipping it. Only if that is truly "
            "impossible, add the call to the WHITELIST in "
            "dimos/codebase_checks/test_no_dunder_new.py."
        )
