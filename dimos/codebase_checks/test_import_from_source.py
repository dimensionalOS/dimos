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

from __future__ import annotations

import ast
from pathlib import Path
import re

from dimos.constants import DIMOS_PROJECT_ROOT

DIMOS_DIR = DIMOS_PROJECT_ROOT / "dimos"

_NOQA = re.compile(r"#\s*noqa(?::\s*(?P<codes>[A-Z0-9, ]+))?", re.IGNORECASE)


class _Module:
    """Indexed view of one source file: what it defines and what it imports."""

    def __init__(self, name: str, path: Path, is_package: bool) -> None:
        self.name = name
        self.path = path
        self.is_package = is_package
        self.defined: set[str] = set()
        # bound name -> (source module, original name, marked as a re-export)
        self.imports: dict[str, tuple[str, str, bool]] = {}


def _module_name(path: Path) -> str:
    parts = path.relative_to(DIMOS_PROJECT_ROOT).with_suffix("").parts
    if parts[-1] == "__init__":
        parts = parts[:-1]
    return ".".join(parts)


def _resolve_from(module: _Module, level: int, target: str | None) -> str:
    """Absolute module a `from ... import` refers to (handles relative imports)."""
    if not level:
        return target or ""
    parts = module.name.split(".")
    anchor = parts if module.is_package else parts[:-1]
    if level > 1:
        anchor = anchor[: len(anchor) - (level - 1)]
    return ".".join(anchor + (target.split(".") if target else []))


def _is_reexport(alias: ast.alias, stmt: ast.ImportFrom, lines: list[str]) -> bool:
    """True if this imported name is deliberately re-exported by the module."""
    if alias.asname is not None and alias.asname == alias.name:
        return True  # `import Y as Y` is the explicit re-export convention
    for lineno in {getattr(alias, "lineno", stmt.lineno), stmt.lineno}:
        if 1 <= lineno <= len(lines):
            m = _NOQA.search(lines[lineno - 1])
            if m and (m.group("codes") is None or "F401" in m.group("codes").upper()):
                return True
    return False


def _build_index() -> dict[str, _Module]:
    modules: dict[str, _Module] = {}
    for path in sorted(DIMOS_DIR.rglob("*.py")):
        modules[_module_name(path)] = _Module(_module_name(path), path, path.name == "__init__.py")
    for mod in modules.values():
        lines = mod.path.read_text(encoding="utf-8").splitlines()
        for node in ast.parse("\n".join(lines), filename=str(mod.path)).body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                mod.defined.add(node.name)
            elif isinstance(node, ast.Assign):
                mod.defined.update(t.id for t in node.targets if isinstance(t, ast.Name))
            elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
                mod.defined.add(node.target.id)
            elif isinstance(node, ast.Import):
                # `import a.b.c [as d]` binds a module, not a re-exportable name.
                mod.defined.update(a.asname or a.name.split(".")[0] for a in node.names)
            elif isinstance(node, ast.ImportFrom):
                src = _resolve_from(mod, node.level, node.module)
                for alias in node.names:
                    if alias.name == "*":
                        continue
                    mod.imports[alias.asname or alias.name] = (
                        src,
                        alias.name,
                        _is_reexport(alias, node, lines),
                    )
    return modules


def _definition_site(modules: dict[str, _Module], mod: str, name: str) -> str:
    """Follow the import chain to where `name` is actually defined (best effort)."""
    seen: set[tuple[str, str]] = set()
    while mod in modules and (mod, name) not in seen:
        seen.add((mod, name))
        m = modules[mod]
        if name in m.defined or name not in m.imports:
            return mod
        mod, name, _ = m.imports[name]
    return mod


def find_reexport_imports() -> list[tuple[Path, int, str, str, str]]:
    """Return (file, line, name, imported_from, defined_in) for each violation."""
    modules = _build_index()
    violations: list[tuple[Path, int, str, str, str]] = []
    for consumer in modules.values():
        tree = ast.parse(consumer.path.read_text(encoding="utf-8"), filename=str(consumer.path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.ImportFrom):
                continue
            src = _resolve_from(consumer, node.level, node.module)
            if src not in modules or src == consumer.name:
                continue  # only internal modules; a module re-importing itself is moot
            origin = modules[src]
            for alias in node.names:
                name = alias.name
                if name == "*" or f"{src}.{name}" in modules:
                    continue  # star import, or `from pkg import submodule`
                if name in origin.defined:
                    continue  # imported from its definition site -- correct
                rec = origin.imports.get(name)
                if rec is None or rec[2]:
                    continue  # not an import of X's (dynamic/star), or a marked re-export
                violations.append(
                    (consumer.path, node.lineno, name, src, _definition_site(modules, src, name))
                )
    return violations


def test_import_from_source() -> None:
    """Fail if any name is imported from a module that only re-imported it."""
    violations = find_reexport_imports()
    if violations:
        listing = "\n".join(
            f"  - {p.relative_to(DIMOS_PROJECT_ROOT)}:{line}: `{name}` imported from "
            f"{src}, but defined in {origin}"
            for p, line, name, src, origin in sorted(violations)
        )
        raise AssertionError(
            f"Found import(s) that pull a name from a re-exporter:\n{listing}\n\n"
            "Import each name straight from the module that defines it (shown above). "
            "If a module re-exports a name on purpose, mark its import with "
            "`# noqa: F401` or the `from x import Y as Y` form, and that re-export "
            "will be allowed."
        )
