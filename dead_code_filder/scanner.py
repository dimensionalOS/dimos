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
Dead code finder - Python code dependency scanner
Scans Python files to build a dependency graph of functions and classes
"""

import ast
from collections import defaultdict
import importlib.util
import json
import os
from pathlib import Path
import sys
from typing import Dict, List, Optional, Set, Tuple


class ImportVisitor(ast.NodeVisitor):
    """AST visitor to extract imports from a Python file"""

    def __init__(self):
        self.imports = []  # List of (module, name, alias) tuples
        self.from_imports = []  # List of (module, [(name, alias), ...]) tuples

    def visit_Import(self, node):
        for alias in node.names:
            self.imports.append((alias.name, alias.name, alias.asname))
        self.generic_visit(node)

    def visit_ImportFrom(self, node):
        if node.module is None:
            # Relative import like "from . import something"
            module = ""
        else:
            module = node.module

        names = []
        for alias in node.names:
            names.append((alias.name, alias.asname))
        self.from_imports.append((module, names, node.level))
        self.generic_visit(node)


class DefinitionExtractor:
    """Extract top-level function and class definitions from Python files"""

    def extract_definitions(self, file_path: str) -> list[str]:
        """Extract top-level function and class names from a Python file"""
        try:
            with open(file_path, encoding="utf-8") as f:
                content = f.read()
        except (UnicodeDecodeError, FileNotFoundError):
            return []

        try:
            tree = ast.parse(content)
        except SyntaxError:
            return []

        definitions = []
        for node in ast.iter_child_nodes(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                definitions.append(node.name)
            elif isinstance(node, ast.ClassDef):
                definitions.append(node.name)

        return definitions

    def extract_uses(
        self, file_path: str, all_definitions: dict[str, list[str]]
    ) -> list[tuple[str, str]]:
        """Extract which functions/classes from other files are used"""
        try:
            with open(file_path, encoding="utf-8") as f:
                content = f.read()
        except (UnicodeDecodeError, FileNotFoundError):
            return []

        try:
            tree = ast.parse(content)
        except SyntaxError:
            return []

        # Get all imports
        visitor = ImportVisitor()
        visitor.visit(tree)

        # Track what names are used from imports
        used_items = []
        os.path.dirname(file_path)

        # Process from imports
        for module, names, level in visitor.from_imports:
            # Resolve the module path
            resolved_module = self._resolve_import(file_path, module, level)
            if resolved_module:
                for name, _alias in names:
                    # Find where this name is actually defined
                    source_file = self._find_definition_source(
                        name, resolved_module, all_definitions
                    )
                    if source_file:
                        used_items.append((source_file, name))

        # Process regular imports (import module)
        for _module_name, _original_name, _alias in visitor.imports:
            # For now, skip regular imports as they don't directly import functions/classes
            pass

        # Also check for usage of imported names in the code
        # This would require more complex analysis of the AST

        return used_items

    def _resolve_import(self, file_path: str, module: str, level: int) -> str | None:
        """Resolve an import to a file path"""
        root_dir = "/home/p/pro/dimensional/dimos"
        file_dir = os.path.dirname(file_path)

        if level > 0:
            # Relative import
            current_dir = file_dir
            for _ in range(level - 1):
                current_dir = os.path.dirname(current_dir)

            if module:
                module_path = os.path.join(current_dir, module.replace(".", "/"))
            else:
                module_path = current_dir
        else:
            # Absolute import
            if module:
                # Try to resolve as a path from root
                module_path = os.path.join(root_dir, module.replace(".", "/"))
            else:
                return None

        # Check if it's a Python file or package
        if os.path.isfile(module_path + ".py"):
            return module_path + ".py"
        elif os.path.isdir(module_path) and os.path.isfile(
            os.path.join(module_path, "__init__.py")
        ):
            return os.path.join(module_path, "__init__.py")

        return None

    def _find_definition_source(
        self, name: str, module_file: str, all_definitions: dict[str, list[str]]
    ) -> str | None:
        """Find where a name is actually defined, tracing through re-exports"""
        if not module_file:
            return None

        # Make path relative to root
        root_dir = "/home/p/pro/dimensional/dimos/"
        if module_file.startswith(root_dir):
            rel_path = module_file[len(root_dir) :]
        else:
            rel_path = module_file

        # Check if this file defines the name
        if rel_path in all_definitions and name in all_definitions[rel_path]:
            return rel_path

        # Otherwise, check if this file re-exports it from somewhere else
        # This would require parsing the file's imports
        # For now, return the module file if it exists
        if os.path.exists(module_file):
            # Try to trace one level of re-export
            try:
                with open(module_file, encoding="utf-8") as f:
                    content = f.read()
                tree = ast.parse(content)
                visitor = ImportVisitor()
                visitor.visit(tree)

                for from_module, names, level in visitor.from_imports:
                    for import_name, alias in names:
                        if import_name == name or alias == name:
                            # Found a re-export, trace it
                            resolved = self._resolve_import(module_file, from_module, level)
                            if resolved:
                                return self._find_definition_source(
                                    import_name, resolved, all_definitions
                                )
            except:
                pass

        return None


class DependencyScanner:
    """Main scanner to build dependency graph of Python code"""

    def __init__(self, root_dir: str):
        self.root_dir = root_dir
        self.extractor = DefinitionExtractor()

    def scan_directory(self) -> dict[str, dict]:
        """Scan all Python files and build dependency graph"""
        print(f"Scanning directory: {self.root_dir}")

        # Find all Python files
        python_files = self._find_python_files()
        print(f"Found {len(python_files)} Python files")

        # First pass: extract all definitions
        all_definitions = {}
        for file_path in python_files:
            rel_path = os.path.relpath(file_path, self.root_dir)
            definitions = self.extractor.extract_definitions(file_path)
            all_definitions[rel_path] = definitions

        print(f"Extracted definitions from {len(all_definitions)} files")

        # Second pass: extract uses (imports)
        import_structure = {}
        for file_path in python_files:
            rel_path = os.path.relpath(file_path, self.root_dir)
            uses = self.extractor.extract_uses(file_path, all_definitions)

            import_structure[rel_path] = {
                "defines": all_definitions.get(rel_path, []),
                "uses": uses,
            }

        return import_structure

    def _find_python_files(self) -> list[str]:
        """Find all Python files in the directory, excluding specified paths"""
        python_files = []
        exclude_dirs = [
            os.path.join(self.root_dir, "dimos", "models", "Detic"),
            os.path.join(self.root_dir, "docker", "navigation", "ros-navigation-autonomy-stack"),
            os.path.join(self.root_dir, "private"),
        ]

        for root, dirs, files in os.walk(self.root_dir):
            # Skip excluded directories
            if any(root.startswith(exclude_dir) for exclude_dir in exclude_dirs):
                continue

            # Skip hidden directories and __pycache__
            dirs[:] = [d for d in dirs if not d.startswith(".") and d != "__pycache__"]

            for file in files:
                if file.endswith(".py"):
                    python_files.append(os.path.join(root, file))

        return python_files

    def save_to_json(self, import_structure: dict, output_file: str):
        """Save the import structure to a JSON file"""
        with open(output_file, "w") as f:
            json.dump(import_structure, f, indent=2, sort_keys=True)
        print(f"Saved import structure to {output_file}")


def main():
    root_dir = "/home/p/pro/dimensional/dimos"
    output_file = "/home/p/pro/dimensional/dimos/dead_code_filder/import_structure.json"

    scanner = DependencyScanner(root_dir)
    import_structure = scanner.scan_directory()
    scanner.save_to_json(import_structure, output_file)

    # Print some statistics
    total_files = len(import_structure)
    total_definitions = sum(len(data["defines"]) for data in import_structure.values())
    total_uses = sum(len(data["uses"]) for data in import_structure.values())

    print("\nStatistics:")
    print(f"  Total files: {total_files}")
    print(f"  Total definitions: {total_definitions}")
    print(f"  Total uses: {total_uses}")

    # Find potentially dead code (files with definitions but no uses)
    unused_definitions = []
    for file_path, data in import_structure.items():
        if data["defines"]:
            # Check if any of these definitions are used elsewhere
            for definition in data["defines"]:
                is_used = False
                for _other_file, other_data in import_structure.items():
                    if any(
                        use[0] == file_path and use[1] == definition for use in other_data["uses"]
                    ):
                        is_used = True
                        break
                if not is_used:
                    unused_definitions.append((file_path, definition))

    if unused_definitions:
        print(f"\nPotentially unused definitions: {len(unused_definitions)}")
        for file_path, definition in unused_definitions[:10]:  # Show first 10
            print(f"  {file_path}: {definition}")
        if len(unused_definitions) > 10:
            print(f"  ... and {len(unused_definitions) - 10} more")


if __name__ == "__main__":
    main()
