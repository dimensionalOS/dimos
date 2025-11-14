#!/usr/bin/env python3
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

"""
Convert all relative imports to absolute imports in the dimos package.
Uses the rope library for Python refactoring.
"""

import os
from pathlib import Path
import re

from rope.base.project import Project
from rope.refactor.importutils import ImportOrganizer


def get_absolute_module_path(file_path: str, base_dir: str) -> str:
    """
    Convert a file path to its absolute module path.

    Example:
        /home/p/pro/dimensional/dimos/dimos/simulation/isaac/__init__.py
        -> dimos.simulation.isaac
    """
    # Get relative path from base_dir
    rel_path = os.path.relpath(file_path, base_dir)

    # Remove .py extension and __init__.py
    if rel_path.endswith("__init__.py"):
        rel_path = os.path.dirname(rel_path)
    elif rel_path.endswith(".py"):
        rel_path = rel_path[:-3]

    # Convert path separators to dots
    module_path = rel_path.replace(os.sep, ".")

    # Add 'dimos' prefix
    if module_path:
        return f"dimos.{module_path}"
    return "dimos"


def resolve_relative_import(current_module: str, relative_import: str, import_level: int) -> str:
    """
    Resolve a relative import to an absolute import.

    Args:
        current_module: Current module path (e.g., 'dimos.simulation.isaac')
        relative_import: The imported module (e.g., 'simulator' or 'base.stream_base')
        import_level: Number of dots (1 for '.', 2 for '..', etc.)

    Returns:
        Absolute import path
    """
    # Split current module into parts
    parts = current_module.split(".")

    # Go up the hierarchy based on import_level
    # import_level=1 means current package, so we remove the module name if present
    # import_level=2 means parent package, etc.
    if import_level > 0:
        # Remove (import_level - 1) parts from the end
        parts = parts[: -(import_level - 1)] if import_level > 1 else parts

    # Add the relative import path
    if relative_import:
        absolute = ".".join(parts + relative_import.split("."))
    else:
        absolute = ".".join(parts)

    return absolute


def convert_relative_to_absolute(file_path: str, base_dir: str) -> tuple[str, bool]:
    """
    Convert relative imports in a file to absolute imports.

    Returns:
        (modified_content, was_modified)
    """
    with open(file_path, encoding="utf-8") as f:
        content = f.read()

    original_content = content
    lines = content.split("\n")
    modified_lines = []

    # Get the current module path
    current_module = get_absolute_module_path(file_path, base_dir)
    if current_module.endswith(".__init__"):
        current_module = current_module[:-9]  # Remove .__init__

    # Pattern to match relative imports
    # Examples: from . import foo, from .. import bar, from .module import baz
    relative_import_pattern = re.compile(r"^(\s*from\s+)(\.+)([a-zA-Z0-9_.]*)(\s+import\s+.+)$")

    for line in lines:
        match = relative_import_pattern.match(line)
        if match:
            indent = match.group(1)
            dots = match.group(2)
            relative_module = match.group(3)
            import_clause = match.group(4)

            # Calculate import level (number of dots)
            import_level = len(dots)

            # Resolve to absolute import
            absolute_module = resolve_relative_import(current_module, relative_module, import_level)

            # Construct new import line
            new_line = f"{indent}{absolute_module}{import_clause}"
            modified_lines.append(new_line)

            print(f"  {line.strip()}")
            print(f"  -> {new_line.strip()}")
        else:
            modified_lines.append(line)

    modified_content = "\n".join(modified_lines)
    was_modified = modified_content != original_content

    return modified_content, was_modified


def main():
    # Project root
    dimos_dir = "/home/p/pro/dimensional/dimos/dimos"

    print(f"Converting relative imports to absolute imports in: {dimos_dir}\n")

    # Find all Python files
    python_files = []
    for root, dirs, files in os.walk(dimos_dir):
        # Skip __pycache__ and .git directories
        dirs[:] = [d for d in dirs if d not in ["__pycache__", ".git", ".pytest_cache"]]

        for file in files:
            if file.endswith(".py"):
                python_files.append(os.path.join(root, file))

    print(f"Found {len(python_files)} Python files\n")

    modified_count = 0

    for file_path in sorted(python_files):
        rel_path = os.path.relpath(file_path, dimos_dir)

        # Check if file has relative imports
        with open(file_path, encoding="utf-8") as f:
            content = f.read()
            if not re.search(r"^\s*from\s+\.", content, re.MULTILINE):
                continue

        print(f"\nProcessing: {rel_path}")

        modified_content, was_modified = convert_relative_to_absolute(file_path, dimos_dir)

        if was_modified:
            # Write back to file
            with open(file_path, "w", encoding="utf-8") as f:
                f.write(modified_content)
            modified_count += 1
            print("  ✓ Modified")
        else:
            print("  - No changes needed")

    print(f"\n{'=' * 60}")
    print("Conversion complete!")
    print(f"Modified {modified_count} files out of {len(python_files)} total files")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
