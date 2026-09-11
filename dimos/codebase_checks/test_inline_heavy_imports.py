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
from collections.abc import Iterable, Iterator

from dimos.constants import DIMOS_PROJECT_ROOT

DIMOS_DIR = DIMOS_PROJECT_ROOT / "dimos"

# Importing any of these at module level loads a heavyweight native library into
# every process that transitively imports the module - including worker
# processes that never touch it. Import them inside the function/method that
# uses them (or under `if TYPE_CHECKING:` for annotations only).
HEAVY_MODULES = ("cv2", "open3d", "rerun")


def _is_type_checking(test: ast.expr) -> bool:
    """True for the guard of `if TYPE_CHECKING:` / `if typing.TYPE_CHECKING:`."""
    if isinstance(test, ast.Name):
        return test.id == "TYPE_CHECKING"
    if isinstance(test, ast.Attribute):
        return test.attr == "TYPE_CHECKING"
    return False


def _iter_eager_nodes(nodes: Iterable[ast.AST]) -> Iterator[ast.AST]:
    """Yield nodes whose code runs at import time.

    Skips function/lambda bodies (imports there are inline, i.e. deferred) and
    the body of `if TYPE_CHECKING:` blocks (never executed at runtime). The
    `else:` branch of a TYPE_CHECKING guard does run at import time.
    """
    for node in nodes:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            continue
        if isinstance(node, ast.If) and _is_type_checking(node.test):
            yield from _iter_eager_nodes(node.orelse)
            continue
        yield node
        yield from _iter_eager_nodes(ast.iter_child_nodes(node))


def _heavy_import(node: ast.Import | ast.ImportFrom) -> str | None:
    """The heavy module an import statement pulls in, or None."""
    if isinstance(node, ast.Import):
        for alias in node.names:
            if alias.name.split(".")[0] in HEAVY_MODULES:
                return alias.name
    if isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
        if node.module.split(".")[0] in HEAVY_MODULES:
            return node.module
    return None


def find_eager_heavy_imports() -> dict[str, list[tuple[int, str]]]:
    """Map of dimos-relative file path -> [(line, module)] for eager heavy imports."""
    hits: dict[str, list[tuple[int, str]]] = {}
    for path in sorted(DIMOS_DIR.rglob("*.py")):
        if path.name.startswith("test_") or path.name == "conftest.py":
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in _iter_eager_nodes(tree.body):
            if not isinstance(node, (ast.Import, ast.ImportFrom)):
                continue
            module = _heavy_import(node)
            if module is not None:
                hits.setdefault(str(path.relative_to(DIMOS_DIR)), []).append((node.lineno, module))
    return hits


def test_heavy_imports_are_inline() -> None:
    """Fail if any file imports cv2/open3d/rerun at module level."""
    hits = find_eager_heavy_imports()
    if hits:
        listing = "\n".join(
            f"  - dimos/{f}:{line}: `{module}`"
            for f, lines in sorted(hits.items())
            for line, module in lines
        )
        raise AssertionError(
            f"Found module-level import(s) of {'/'.join(HEAVY_MODULES)}:\n{listing}\n\n"
            "These libraries load large native extensions into every process that "
            "transitively imports the module. Import them inside the function or "
            "method that uses them; imports needed only for type annotations go "
            "under `if TYPE_CHECKING:`."
        )
