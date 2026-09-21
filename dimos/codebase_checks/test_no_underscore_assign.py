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


def _is_underscore(target: ast.expr) -> bool:
    return isinstance(target, ast.Name) and target.id == "_"


def _binds_underscore(node: ast.AST) -> bool:
    """True for `_ = ...` (or `_: T = ...`), but not for tuple unpacking.

    `a, _ = f()` has a `Tuple` target, not a bare `Name`, so it is allowed.
    """
    if isinstance(node, ast.Assign):
        return any(_is_underscore(t) for t in node.targets)
    if isinstance(node, ast.AnnAssign):
        return _is_underscore(node.target)
    return False


def find_underscore_assignments() -> list[tuple[Path, int]]:
    """Return (file, line_number) for every `_ = ...` binding under dimos/."""
    dimos_dir = DIMOS_PROJECT_ROOT / "dimos"
    hits: list[tuple[Path, int]] = []
    for path in sorted(dimos_dir.rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if _binds_underscore(node):
                hits.append((path, node.lineno))
    return hits


def test_no_underscore_assignment():
    """Fail if any file assigns to a bare `_`."""
    dimos_dir = DIMOS_PROJECT_ROOT / "dimos"
    hits = find_underscore_assignments()
    if hits:
        listing = "\n".join(f"  - {p.relative_to(dimos_dir)}:{lineno}" for p, lineno in hits)
        raise AssertionError(
            f"Found assignment(s) to `_` in dimos/:\n{listing}\n\n"
            "Assigning to `_` is not allowed: it hides an unused variable instead "
            "of removing it. Delete the variable. If you only need the "
            "expression's side effect, evaluate it directly with a call "
            "(`obj.method()`, `getattr(obj, 'attr')`) or log it; a bare attribute "
            "access needs `# noqa: B018`. Tuple unpacking (`a, _ = f()`) is fine "
            "and not flagged by this rule."
        )
