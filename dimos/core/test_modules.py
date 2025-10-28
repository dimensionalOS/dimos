# Copyright 2025 Dimensional Inc.
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

"""Test that all Module subclasses implement required resource management methods."""

import ast
import inspect
from pathlib import Path

import pytest

from dimos.core.module import Module


class ModuleVisitor(ast.NodeVisitor):
    """AST visitor to find classes and their base classes."""

    def __init__(self, filepath: str) -> None:
        self.filepath = filepath
        self.classes: list[
            tuple[str, list[str], set[str]]
        ] = []  # (class_name, base_classes, methods)

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        """Visit a class definition."""
        # Get base class names
        base_classes = []
        for base in node.bases:
            if isinstance(base, ast.Name):
                base_classes.append(base.id)
            elif isinstance(base, ast.Attribute):
                # Handle cases like dimos.core.Module
                parts = []
                current = base
                while isinstance(current, ast.Attribute):
                    parts.append(current.attr)
                    current = current.value
                if isinstance(current, ast.Name):
                    parts.append(current.id)
                base_classes.append(".".join(reversed(parts)))

        # Get method names defined in this class
        methods = set()
        for item in node.body:
            if isinstance(item, ast.FunctionDef):
                methods.add(item.name)

        self.classes.append((node.name, base_classes, methods))
        self.generic_visit(node)


def get_import_aliases(tree: ast.AST) -> dict[str, str]:
    """Extract import aliases from the AST."""
    aliases = {}

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                key = alias.asname if alias.asname else alias.name
                aliases[key] = alias.name
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            for alias in node.names:
                key = alias.asname if alias.asname else alias.name
                full_name = f"{module}.{alias.name}" if module else alias.name
                aliases[key] = full_name

    return aliases


def is_module_subclass(
    base_classes: list[str],
    aliases: dict[str, str],
    class_hierarchy: dict[str, list[str]] | None = None,
    current_module_path: str | None = None,
) -> bool:
    """Check if any base class is or resolves to dimos.core.Module or its variants (recursively)."""
    target_classes = {
        "Module",
        "ModuleBase",
        "DaskModule",
        "dimos.core.Module",
        "dimos.core.ModuleBase",
        "dimos.core.DaskModule",
        "dimos.core.module.Module",
        "dimos.core.module.ModuleBase",
        "dimos.core.module.DaskModule",
    }

    def find_qualified_name(base: str, context_module: str | None = None) -> str:
        """Find the qualified name for a base class, using import context if available."""
        if not class_hierarchy:
            return base

        # First try exact match (already fully qualified or in hierarchy)
        if base in class_hierarchy:
            return base

        # Check if it's in our aliases (from imports)
        if base in aliases:
            resolved = aliases[base]
            if resolved in class_hierarchy:
                return resolved
            # The resolved name might be a qualified name that exists
            return resolved

        # If we have a context module and base is a simple name,
        # try to find it in the same module first (for local classes)
        if context_module and "." not in base:
            same_module_qualified = f"{context_module}.{base}"
            if same_module_qualified in class_hierarchy:
                return same_module_qualified

        # Otherwise return the base as-is
        return base

    def check_base(
        base: str, visited: set[str] | None = None, context_module: str | None = None
    ) -> bool:
        if visited is None:
            visited = set()

        # Avoid infinite recursion
        if base in visited:
            return False
        visited.add(base)

        # Check direct match
        if base in target_classes:
            return True

        # Check if it's an alias
        if base in aliases:
            resolved = aliases[base]
            if resolved in target_classes:
                return True
            # Continue checking with resolved name
            base = resolved

        # If we have a class hierarchy, recursively check parent classes
        if class_hierarchy:
            # Resolve the base class name to a qualified name
            qualified_name = find_qualified_name(base, context_module)

            if qualified_name in class_hierarchy:
                # Check all parent classes
                for parent_base in class_hierarchy[qualified_name]:
                    if check_base(parent_base, visited, None):  # Parent lookups don't use context
                        return True

        return False

    for base in base_classes:
        if check_base(base, context_module=current_module_path):
            return True

    return False


def scan_file(
    filepath: Path,
    class_hierarchy: dict[str, list[str]] | None = None,
    root_path: Path | None = None,
) -> list[tuple[str, str, bool, bool, set[str]]]:
    """
    Scan a Python file for Module subclasses.

    Returns:
        List of (class_name, filepath, has_start, has_stop, forbidden_methods)
    """
    forbidden_method_names = {"acquire", "release", "open", "close", "shutdown", "clean", "cleanup"}

    try:
        with open(filepath, encoding="utf-8") as f:
            content = f.read()

        tree = ast.parse(content, filename=str(filepath))
        aliases = get_import_aliases(tree)

        visitor = ModuleVisitor(str(filepath))
        visitor.visit(tree)

        # Get module path for this file to properly resolve base classes
        current_module_path = None
        if root_path:
            try:
                rel_path = filepath.relative_to(root_path.parent)
                module_parts = list(rel_path.parts[:-1])
                if rel_path.stem != "__init__":
                    module_parts.append(rel_path.stem)
                current_module_path = ".".join(module_parts)
            except ValueError:
                pass

        results = []
        for class_name, base_classes, methods in visitor.classes:
            if is_module_subclass(base_classes, aliases, class_hierarchy, current_module_path):
                has_start = "start" in methods
                has_stop = "stop" in methods
                forbidden_found = methods & forbidden_method_names
                results.append((class_name, str(filepath), has_start, has_stop, forbidden_found))

        return results

    except (SyntaxError, UnicodeDecodeError):
        # Skip files that can't be parsed
        return []


def build_class_hierarchy(root_path: Path) -> dict[str, list[str]]:
    """Build a complete class hierarchy by scanning all Python files."""
    hierarchy = {}

    for filepath in sorted(root_path.rglob("*.py")):
        # Skip __pycache__ and other irrelevant directories
        if "__pycache__" in filepath.parts or ".venv" in filepath.parts:
            continue

        try:
            with open(filepath, encoding="utf-8") as f:
                content = f.read()

            tree = ast.parse(content, filename=str(filepath))
            visitor = ModuleVisitor(str(filepath))
            visitor.visit(tree)

            # Convert filepath to module path (e.g., dimos/core/module.py -> dimos.core.module)
            try:
                rel_path = filepath.relative_to(root_path.parent)
            except ValueError:
                # If we can't get relative path, skip this file
                continue

            # Convert path to module notation
            module_parts = list(rel_path.parts[:-1])  # Exclude filename
            if rel_path.stem != "__init__":
                module_parts.append(rel_path.stem)  # Add filename without .py
            module_name = ".".join(module_parts)

            for class_name, base_classes, _ in visitor.classes:
                # Use fully qualified name as key to avoid conflicts
                qualified_name = f"{module_name}.{class_name}" if module_name else class_name
                hierarchy[qualified_name] = base_classes

        except (SyntaxError, UnicodeDecodeError):
            # Skip files that can't be parsed
            continue

    return hierarchy


def scan_directory(root_path: Path) -> list[tuple[str, str, bool, bool, set[str]]]:
    """Scan all Python files in the directory tree."""
    # First, build the complete class hierarchy
    class_hierarchy = build_class_hierarchy(root_path)

    # Then scan for Module subclasses using the complete hierarchy
    results = []

    for filepath in sorted(root_path.rglob("*.py")):
        # Skip __pycache__ and other irrelevant directories
        if "__pycache__" in filepath.parts or ".venv" in filepath.parts:
            continue

        file_results = scan_file(filepath, class_hierarchy, root_path)
        results.extend(file_results)

    return results


def get_all_module_subclasses():
    """Find all Module subclasses in the dimos codebase."""
    # Get the dimos package directory
    dimos_file = inspect.getfile(Module)
    dimos_path = Path(dimos_file).parent.parent  # Go up from dimos/core/module.py to dimos/

    results = scan_directory(dimos_path)

    # Filter out test modules and base classes
    filtered_results = []
    for class_name, filepath, has_start, has_stop, forbidden_methods in results:
        # Skip base module classes themselves
        if class_name in ("Module", "ModuleBase", "DaskModule", "SkillModule"):
            continue

        # Skip test-only modules (those defined in test_ files)
        if "test_" in Path(filepath).name:
            continue

        filtered_results.append((class_name, filepath, has_start, has_stop, forbidden_methods))

    return filtered_results


@pytest.mark.parametrize(
    "class_name,filepath,has_start,has_stop,forbidden_methods",
    get_all_module_subclasses(),
    ids=lambda val: val[0] if isinstance(val, str) else str(val),
)
def test_module_has_start_and_stop(
    class_name: str, filepath, has_start, has_stop, forbidden_methods
) -> None:
    """Test that Module subclasses implement start and stop methods and don't use forbidden methods."""
    # Get relative path for better error messages
    try:
        rel_path = Path(filepath).relative_to(Path.cwd())
    except ValueError:
        rel_path = filepath

    errors = []

    # Check for missing required methods
    if not has_start:
        errors.append("missing required method: start")
    if not has_stop:
        errors.append("missing required method: stop")

    # Check for forbidden methods
    if forbidden_methods:
        forbidden_list = ", ".join(sorted(forbidden_methods))
        errors.append(f"has forbidden method(s): {forbidden_list}")

    assert not errors, f"{class_name} in {rel_path} has issues:\n  - " + "\n  - ".join(errors)
