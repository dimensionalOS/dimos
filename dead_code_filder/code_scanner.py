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
Dead code finder - Enhanced Python code dependency scanner
Scans Python files to build a comprehensive dependency graph with proper import tracing
"""

import ast
from collections import defaultdict
import json
import os
from pathlib import Path
import sys
import traceback
from typing import Dict, List, Optional, Set, Tuple


class PythonFileAnalyzer:
    """Analyzes Python files to extract definitions and track imports"""

    def __init__(self, root_dir: str):
        self.root_dir = root_dir
        # Cache for parsed files
        self.file_cache = {}
        # Cache for module definitions
        self.definition_cache = {}

    def get_file_content(self, file_path: str) -> str | None:
        """Read and cache file content"""
        if file_path in self.file_cache:
            return self.file_cache[file_path]

        try:
            with open(file_path, encoding="utf-8") as f:
                content = f.read()
                self.file_cache[file_path] = content
                return content
        except (UnicodeDecodeError, FileNotFoundError, PermissionError):
            self.file_cache[file_path] = None
            return None

    def get_ast(self, file_path: str) -> ast.Module | None:
        """Parse file and return AST"""
        content = self.get_file_content(file_path)
        if content is None:
            return None

        try:
            return ast.parse(content)
        except SyntaxError:
            return None

    def extract_top_level_definitions(self, file_path: str) -> list[str]:
        """Extract all top-level function and class definitions from a file"""
        if file_path in self.definition_cache:
            return self.definition_cache[file_path]

        tree = self.get_ast(file_path)
        if tree is None:
            self.definition_cache[file_path] = []
            return []

        definitions = []
        for node in tree.body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                definitions.append(node.name)
            elif isinstance(node, ast.ClassDef):
                definitions.append(node.name)

        self.definition_cache[file_path] = definitions
        return definitions

    def resolve_import_path(self, from_file: str, module_name: str, level: int = 0) -> str | None:
        """Resolve an import statement to an actual file path"""
        # Handle relative imports
        if level > 0:
            # Start from the directory containing the importing file
            current_path = Path(from_file).parent

            # Go up directories based on level
            for _ in range(level - 1):
                current_path = current_path.parent

            if module_name:
                # Append the module path
                parts = module_name.split(".")
                for part in parts:
                    current_path = current_path / part

            # Check for .py file or __init__.py in package
            if current_path.with_suffix(".py").exists():
                return str(current_path.with_suffix(".py"))
            elif (current_path / "__init__.py").exists():
                return str(current_path / "__init__.py")

        else:
            # Absolute import - try to resolve from root
            if module_name:
                parts = module_name.split(".")
                current_path = Path(self.root_dir)

                for part in parts:
                    current_path = current_path / part

                # Check for .py file or __init__.py in package
                if current_path.with_suffix(".py").exists():
                    return str(current_path.with_suffix(".py"))
                elif (current_path / "__init__.py").exists():
                    return str(current_path / "__init__.py")

        return None

    def trace_definition_source(
        self, name: str, file_path: str, visited: set[str] | None = None
    ) -> str | None:
        """Trace where a name is actually defined, following imports"""
        if visited is None:
            visited = set()

        # Avoid infinite loops
        if file_path in visited:
            return None
        visited.add(file_path)

        # Check if this file defines the name
        definitions = self.extract_top_level_definitions(file_path)
        if name in definitions:
            return file_path

        # Check if this file imports and re-exports the name
        tree = self.get_ast(file_path)
        if tree is None:
            return None

        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                # Check if this import includes our name
                for alias in node.names:
                    if alias.name == name or alias.name == "*":
                        # Resolve where this import comes from
                        module_name = node.module if node.module else ""
                        resolved_path = self.resolve_import_path(file_path, module_name, node.level)

                        if resolved_path and resolved_path != file_path:
                            # Recursively trace to the actual source
                            source = self.trace_definition_source(name, resolved_path, visited)
                            if source:
                                return source

        return None

    def analyze_file_dependencies(self, file_path: str) -> list[tuple[str, str]]:
        """Analyze a file and return list of (source_file, name) dependencies"""
        tree = self.get_ast(file_path)
        if tree is None:
            return []

        dependencies = []
        processed_imports = set()  # Track what we've already processed

        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                module_name = node.module if node.module else ""
                level = node.level

                # Resolve the import to a file
                imported_file = self.resolve_import_path(file_path, module_name, level)
                if not imported_file:
                    continue

                # Process each imported name
                for alias in node.names:
                    if alias.name == "*":
                        # Import * - import all definitions from that file
                        definitions = self.extract_top_level_definitions(imported_file)
                        for def_name in definitions:
                            # Find the actual source of this definition
                            source = self.trace_definition_source(def_name, imported_file)
                            if source:
                                key = (source, def_name)
                                if key not in processed_imports:
                                    processed_imports.add(key)
                                    dependencies.append(key)
                    else:
                        # Specific import - trace to actual definition source
                        source = self.trace_definition_source(alias.name, imported_file)
                        if source:
                            key = (source, alias.name)
                            if key not in processed_imports:
                                processed_imports.add(key)
                                dependencies.append(key)

        return dependencies


class DeadCodeScanner:
    """Main scanner to find dead code in Python projects"""

    def __init__(self, root_dir: str):
        self.root_dir = root_dir
        self.analyzer = PythonFileAnalyzer(root_dir)
        self.exclude_patterns = [
            os.path.join(root_dir, "dimos", "models", "Detic"),
            os.path.join(root_dir, "docker", "navigation", "ros-navigation-autonomy-stack"),
            os.path.join(root_dir, "private"),
            "__pycache__",
            ".git",
            ".venv",
            "venv",
            ".pytest_cache",
        ]

    def should_exclude(self, path: str) -> bool:
        """Check if a path should be excluded from scanning"""
        for pattern in self.exclude_patterns:
            if pattern in path:
                return True
        return False

    def find_all_python_files(self) -> list[str]:
        """Find all Python files in the project"""
        python_files = []

        for root, dirs, files in os.walk(self.root_dir):
            # Skip excluded directories
            if self.should_exclude(root):
                continue

            # Filter out hidden directories
            dirs[:] = [d for d in dirs if not d.startswith(".")]

            for file in files:
                if file.endswith(".py") and not file.startswith("."):
                    file_path = os.path.join(root, file)
                    if not self.should_exclude(file_path):
                        python_files.append(file_path)

        return sorted(python_files)

    def build_import_structure(self) -> dict[str, dict]:
        """Build the complete import structure for all Python files"""
        print(f"Scanning {self.root_dir}...")
        python_files = self.find_all_python_files()
        print(f"Found {len(python_files)} Python files to analyze")

        import_structure = {}
        errors = []

        for i, file_path in enumerate(python_files, 1):
            # Make path relative to root
            rel_path = os.path.relpath(file_path, self.root_dir)

            if i % 100 == 0:
                print(f"  Processing file {i}/{len(python_files)}: {rel_path}")

            try:
                # Extract definitions
                definitions = self.analyzer.extract_top_level_definitions(file_path)

                # Extract dependencies
                dependencies = self.analyzer.analyze_file_dependencies(file_path)

                # Convert dependencies to relative paths
                uses = []
                for source_file, name in dependencies:
                    source_rel = os.path.relpath(source_file, self.root_dir)
                    uses.append([source_rel, name])

                import_structure[rel_path] = {"defines": definitions, "uses": uses}

            except Exception as e:
                errors.append(f"Error processing {rel_path}: {e!s}")
                import_structure[rel_path] = {"defines": [], "uses": []}

        if errors:
            print(f"\nEncountered {len(errors)} errors during processing")
            for error in errors[:5]:  # Show first 5 errors
                print(f"  {error}")

        return import_structure

    def analyze_dead_code(self, import_structure: dict) -> dict:
        """Analyze the import structure to find potentially dead code"""
        analysis = {
            "total_files": len(import_structure),
            "total_definitions": 0,
            "total_uses": 0,
            "unused_files": [],
            "unused_definitions": [],
            "most_used": [],
            "most_dependent": [],
        }

        # Count total definitions and uses
        for file_path, data in import_structure.items():
            analysis["total_definitions"] += len(data["defines"])
            analysis["total_uses"] += len(data["uses"])

        # Find unused definitions
        definition_usage = defaultdict(int)
        for file_path, data in import_structure.items():
            for use_file, use_name in data["uses"]:
                definition_usage[(use_file, use_name)] += 1

        # Check each definition
        for file_path, data in import_structure.items():
            for definition in data["defines"]:
                usage_count = definition_usage.get((file_path, definition), 0)
                if usage_count == 0:
                    # Check if it might be a main script or test file
                    if not (
                        definition == "main"
                        or definition.startswith("test_")
                        or "test" in file_path.lower()
                        or file_path.endswith("__main__.py")
                    ):
                        analysis["unused_definitions"].append(
                            {"file": file_path, "definition": definition}
                        )

            # Check if entire file is unused (no imports from it)
            if data["defines"] and not any(
                use[0] == file_path
                for other_data in import_structure.values()
                for use in other_data["uses"]
            ):
                if not (
                    "test" in file_path.lower()
                    or file_path.endswith("__main__.py")
                    or file_path.endswith("setup.py")
                ):
                    analysis["unused_files"].append(file_path)

        # Find most used definitions
        most_used = sorted(
            [(k, v) for k, v in definition_usage.items() if v > 0], key=lambda x: x[1], reverse=True
        )[:20]
        analysis["most_used"] = [
            {"file": file, "definition": name, "usage_count": count}
            for (file, name), count in most_used
        ]

        # Find files with most dependencies
        dependency_counts = [
            (file, len(data["uses"]))
            for file, data in import_structure.items()
            if len(data["uses"]) > 0
        ]
        dependency_counts.sort(key=lambda x: x[1], reverse=True)
        analysis["most_dependent"] = [
            {"file": file, "dependency_count": count} for file, count in dependency_counts[:20]
        ]

        return analysis


def main():
    root_dir = "/home/p/pro/dimensional/dimos"
    output_dir = "/home/p/pro/dimensional/dimos/dead_code_filder"

    scanner = DeadCodeScanner(root_dir)

    # Build the import structure
    print("Building import structure...")
    import_structure = scanner.build_import_structure()

    # Save to JSON
    output_file = os.path.join(output_dir, "import_structure.json")
    with open(output_file, "w") as f:
        json.dump(import_structure, f, indent=2, sort_keys=True)
    print(f"\nSaved import structure to {output_file}")

    # Analyze for dead code
    print("\nAnalyzing for dead code...")
    analysis = scanner.analyze_dead_code(import_structure)

    # Save analysis
    analysis_file = os.path.join(output_dir, "analysis.json")
    with open(analysis_file, "w") as f:
        json.dump(analysis, f, indent=2)
    print(f"Saved analysis to {analysis_file}")

    # Print summary
    print("\n" + "=" * 50)
    print("ANALYSIS SUMMARY")
    print("=" * 50)
    print(f"Total files analyzed: {analysis['total_files']}")
    print(f"Total definitions found: {analysis['total_definitions']}")
    print(f"Total import uses: {analysis['total_uses']}")
    print(f"\nPotentially unused files: {len(analysis['unused_files'])}")
    if analysis["unused_files"]:
        for file in analysis["unused_files"][:10]:
            print(f"  - {file}")
        if len(analysis["unused_files"]) > 10:
            print(f"  ... and {len(analysis['unused_files']) - 10} more")

    print(f"\nPotentially unused definitions: {len(analysis['unused_definitions'])}")
    if analysis["unused_definitions"]:
        for item in analysis["unused_definitions"][:10]:
            print(f"  - {item['file']}: {item['definition']}")
        if len(analysis["unused_definitions"]) > 10:
            print(f"  ... and {len(analysis['unused_definitions']) - 10} more")

    print("\nMost used definitions:")
    for item in analysis["most_used"][:5]:
        print(f"  - {item['file']}: {item['definition']} ({item['usage_count']} uses)")

    print("\nFiles with most dependencies:")
    for item in analysis["most_dependent"][:5]:
        print(f"  - {item['file']} ({item['dependency_count']} dependencies)")


if __name__ == "__main__":
    main()
