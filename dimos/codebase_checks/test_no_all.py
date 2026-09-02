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


def _is_all(target: ast.expr) -> bool:
    return isinstance(target, ast.Name) and target.id == "__all__"


def _defines_all(node: ast.AST) -> bool:
    if isinstance(node, ast.Assign):
        return any(_is_all(t) for t in node.targets)
    if isinstance(node, (ast.AnnAssign, ast.AugAssign)):
        return _is_all(node.target)
    return False


def find_all_definitions() -> list[tuple[Path, int]]:
    """Return (file, line_number) for every `__all__` binding under dimos/."""
    dimos_dir = DIMOS_PROJECT_ROOT / "dimos"
    hits: list[tuple[Path, int]] = []
    for path in sorted(dimos_dir.rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if _defines_all(node):
                hits.append((path, node.lineno))
    return hits


def test_no_all():
    """Fail if any file defines `__all__`."""
    dimos_dir = DIMOS_PROJECT_ROOT / "dimos"
    hits = find_all_definitions()
    if hits:
        listing = "\n".join(f"  - {p.relative_to(dimos_dir)}:{lineno}" for p, lineno in hits)
        raise AssertionError(
            f"Found __all__ definition(s) in dimos/:\n{listing}\n\n"
            "__all__ is not allowed. We don't use `from x import *`, so __all__ "
            "lists serve no purpose and are tedious to maintain. Remove them. For "
            "an import that exists purely to be re-exported, use `# noqa: F401`."
        )
