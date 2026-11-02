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

"""An override of an @rpc method must carry the decorator itself.

``Module.rpcs`` reads the ``__rpc__`` marker off each attribute, so a plain override
hides the base method from the module's proxy and ``start()`` on such a module fails
at deploy time. This check reads the source instead of importing it, so modules behind
optional dependencies, test-only modules, the examples and the documentation snippets
are all covered.
"""

import ast
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
import re

from dimos.constants import DIMOS_PROJECT_ROOT

ROOTS = {"dimos.core.module.Module", "dimos.core.module.ModuleBase"}
MARKERS = {"rpc", "skill"}
PYTHON_DIRS = ("dimos", "examples")
FENCE = re.compile(r"^```(?:python|py)\b[^\n]*\n(.*?)^```", re.DOTALL | re.MULTILINE)


@dataclass
class _ClassDef:
    path: Path
    bases: list[str]
    # method name -> (line, carries @rpc or @skill)
    methods: dict[str, tuple[int, bool]]


def _name_of(node: ast.expr) -> str:
    if isinstance(node, ast.Call):
        node = node.func
    if isinstance(node, ast.Attribute):
        return node.attr
    return node.id if isinstance(node, ast.Name) else ""


def _nodes(node: ast.AST) -> Iterator[ast.AST]:
    """Every node below ``node``, skipping the bodies of ``if TYPE_CHECKING:`` blocks."""
    for child in ast.iter_child_nodes(node):
        if isinstance(child, ast.If) and _name_of(child.test) == "TYPE_CHECKING":
            children = child.orelse
        else:
            children = [child]
        for grandchild in children:
            yield grandchild
            yield from _nodes(grandchild)


def _imports(tree: ast.Module) -> dict[str, str]:
    """Local name -> dotted name for the module's imports; relative imports are not used."""
    names: dict[str, str] = {}
    for node in _nodes(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                top = alias.name.split(".")[0]
                names[alias.asname or top] = alias.name if alias.asname else top
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            for alias in node.names:
                names[alias.asname or alias.name] = f"{node.module}.{alias.name}"
    return names


def _resolve(node: ast.expr, imports: dict[str, str], module: str) -> str | None:
    """Dotted name of a base class expression, or None when it is not a name."""
    if isinstance(node, ast.Subscript):
        node = node.value
    if isinstance(node, ast.Name):
        return imports.get(node.id, f"{module}.{node.id}")
    if isinstance(node, ast.Attribute):
        head = _resolve(node.value, imports, module)
        return None if head is None else f"{head}.{node.attr}"
    return None


def _classes_in(
    tree: ast.Module, module: str, path: Path, line_offset: int = 0
) -> dict[str, _ClassDef]:
    imports = _imports(tree)
    found: dict[str, _ClassDef] = {}
    for node in _nodes(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        methods = {
            item.name: (
                item.lineno + line_offset,
                any(_name_of(d) in MARKERS for d in item.decorator_list),
            )
            for item in node.body
            if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        bases = [b for b in (_resolve(base, imports, module) for base in node.bases) if b]
        found[f"{module}.{node.name}"] = _ClassDef(path, bases, methods)
    return found


def _module_like(table: dict[str, _ClassDef]) -> set[str]:
    """Classes that descend from Module or ModuleBase."""
    module_like = set(ROOTS)
    changed = True
    while changed:
        changed = False
        for key, cls in table.items():
            if key not in module_like and any(base in module_like for base in cls.bases):
                module_like.add(key)
                changed = True
    return module_like


def _python_classes() -> dict[str, _ClassDef]:
    table: dict[str, _ClassDef] = {}
    for directory in PYTHON_DIRS:
        for path in sorted((DIMOS_PROJECT_ROOT / directory).rglob("*.py")):
            module = ".".join(path.relative_to(DIMOS_PROJECT_ROOT).with_suffix("").parts)
            module = module.removesuffix(".__init__")
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            table.update(_classes_in(tree, module, path))
    return table


def _doc_classes(python_table: dict[str, _ClassDef]) -> dict[str, _ClassDef]:
    """Classes in fenced python blocks; bare base names fall back to same-named repo modules."""
    by_name = {
        key.rsplit(".", 1)[1]: key
        for key in sorted(_module_like(python_table))
        if key in python_table
    }
    table: dict[str, _ClassDef] = {}
    paths = sorted(DIMOS_PROJECT_ROOT.glob("*.md")) + sorted(
        (DIMOS_PROJECT_ROOT / "docs").rglob("*.md")
    )
    for path in paths:
        text = path.read_text(encoding="utf-8")
        for match in FENCE.finditer(text):
            try:
                tree = ast.parse(match.group(1))
            except SyntaxError:
                continue
            line_offset = text.count("\n", 0, match.start(1))
            module = f"{path.relative_to(DIMOS_PROJECT_ROOT)}:{line_offset}"
            for key, cls in _classes_in(tree, module, path, line_offset).items():
                cls.bases = [
                    base if base in python_table else by_name.get(base.rsplit(".", 1)[1], base)
                    for base in cls.bases
                ]
                table[key] = cls
    return table


def _marked_ancestor(table: dict[str, _ClassDef], key: str, method: str) -> str | None:
    """Nearest ancestor of ``key`` that defines ``method`` with @rpc or @skill."""
    queue = list(table[key].bases)
    seen: set[str] = set()
    while queue:
        base = queue.pop(0)
        if base in seen or base not in table:
            continue
        seen.add(base)
        info = table[base].methods.get(method)
        if info is not None and info[1]:
            return base
        queue.extend(table[base].bases)
    return None


def _plain_overrides(table: dict[str, _ClassDef]) -> list[str]:
    hits: list[str] = []
    for key in sorted(_module_like(table) & table.keys()):
        cls = table[key]
        for method, (line, marked) in cls.methods.items():
            if marked or method.startswith("_"):
                continue
            ancestor = _marked_ancestor(table, key, method)
            if ancestor is not None:
                hits.append(
                    f"{cls.path.relative_to(DIMOS_PROJECT_ROOT)}:{line}: "
                    f"{key.rsplit('.', 1)[1]}.{method} overrides {ancestor.rsplit('.', 1)[1]}.{method}"
                )
    return hits


def test_rpc_overrides_keep_the_decorator() -> None:
    table = _python_classes()
    table.update(_doc_classes(table))
    hits = _plain_overrides(table)
    assert not hits, (
        "These methods override an @rpc method without the decorator, so Module.rpcs and the "
        "module's proxy cannot see them. Add @rpc (or @skill) to each:\n  - " + "\n  - ".join(hits)
    )
