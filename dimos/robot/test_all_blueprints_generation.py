# Copyright 2025-2026 Dimensional Inc.
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
from collections.abc import Generator
import difflib
import os
from pathlib import Path
import subprocess

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT

IGNORED_FILES: set[str] = {
    "dimos/robot/all_blueprints.py",
    "dimos/robot/get_all_blueprints.py",
    "dimos/robot/test_all_blueprints.py",
    "dimos/robot/test_all_blueprints_generation.py",
    "dimos/core/blueprints.py",
    "dimos/core/test_blueprints.py",
}
BLUEPRINT_METHODS = {"transports", "global_config", "remappings", "requirements"}


def test_all_blueprints_is_current() -> None:
    root = DIMOS_PROJECT_ROOT / "dimos"
    all_blueprints, all_modules = _scan_for_blueprints(root)
    generated_content = _generate_all_blueprints_content(all_blueprints, all_modules)

    file_path = root / "robot" / "all_blueprints.py"

    if "CI" in os.environ:
        if not file_path.exists():
            pytest.fail(f"all_blueprints.py does not exist at {file_path}")

        current_content = file_path.read_text()
        if current_content != generated_content:
            diff = difflib.unified_diff(
                current_content.splitlines(keepends=True),
                generated_content.splitlines(keepends=True),
                fromfile="all_blueprints.py (current)",
                tofile="all_blueprints.py (generated)",
            )
            diff_str = "".join(diff)
            pytest.fail(
                f"all_blueprints.py is out of date. Run "
                f"`pytest dimos/robot/test_all_blueprints_generation.py` locally to update.\n\n"
                f"Diff:\n{diff_str}"
            )
    else:
        file_path.write_text(generated_content)

        if _check_for_uncommitted_changes(file_path):
            pytest.fail(
                "all_blueprints.py was updated and has uncommitted changes. "
                "Please commit the changes."
            )


def _scan_for_blueprints(root: Path) -> tuple[dict[str, str], dict[str, str]]:
    all_blueprints: dict[str, str] = {}
    all_modules: dict[str, str] = {}

    for file_path in sorted(_get_all_python_files(root)):
        module_name = _path_to_module_name(file_path, root)
        blueprint_vars, module_vars = _find_blueprints_in_file(file_path)

        for var_name in blueprint_vars:
            full_path = f"{module_name}:{var_name}"
            cli_name = var_name.replace("_", "-")
            all_blueprints[cli_name] = full_path

        for var_name in module_vars:
            full_path = f"{module_name}:{var_name}"
            all_modules[var_name] = module_name

    return all_blueprints, all_modules


def _generate_all_blueprints_content(
    all_blueprints: dict[str, str],
    all_modules: dict[str, str],
) -> str:
    lines = [
        "# Copyright 2025-2026 Dimensional Inc.",
        "#",
        '# Licensed under the Apache License, Version 2.0 (the "License");',
        "# you may not use this file except in compliance with the License.",
        "# You may obtain a copy of the License at",
        "#",
        "#     http://www.apache.org/licenses/LICENSE-2.0",
        "#",
        "# Unless required by applicable law or agreed to in writing, software",
        '# distributed under the License is distributed on an "AS IS" BASIS,',
        "# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.",
        "# See the License for the specific language governing permissions and",
        "# limitations under the License.",
        "",
        "# This file is auto-generated. Do not edit manually.",
        "# Run `pytest dimos/robot/test_all_blueprints_generation.py` to regenerate.",
        "",
        "all_blueprints = {",
    ]

    for name in sorted(all_blueprints.keys()):
        lines.append(f'    "{name}": "{all_blueprints[name]}",')

    lines.append("}\n\n")
    lines.append("all_modules = {")

    for name in sorted(all_modules.keys()):
        lines.append(f'    "{name}": "{all_modules[name]}",')

    lines.append("}\n")

    return "\n".join(lines)


def _check_for_uncommitted_changes(file_path: Path) -> bool:
    try:
        result = subprocess.run(
            ["git", "diff", "--quiet", str(file_path)],
            capture_output=True,
            cwd=file_path.parent,
        )
        return result.returncode != 0
    except Exception:
        return False


def _get_all_python_files(root: Path) -> Generator[Path, None, None]:
    for path in root.rglob("*.py"):
        rel_path = str(path.relative_to(root.parent))
        if "__pycache__" in str(path) or rel_path in IGNORED_FILES:
            continue
        yield path


def _path_to_module_name(path: Path, root: Path) -> str:
    parts = list(path.relative_to(root.parent).parts)
    parts[-1] = parts[-1].removesuffix(".py")
    return ".".join(parts)


def _find_blueprints_in_file(file_path: Path) -> tuple[list[str], list[str]]:
    blueprint_vars: list[str] = []
    module_vars: list[str] = []

    try:
        source = file_path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(file_path))
    except Exception:
        return [], []

    # Only look at top-level statements (direct children of the Module node)
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue

        # Get the variable name(s)
        for target in node.targets:
            if not isinstance(target, ast.Name):
                continue
            var_name = target.id

            if var_name.startswith("_"):
                continue

            # Check if it's a blueprint (ModuleBlueprintSet instance)
            if _is_autoconnect_call(node.value) or _ends_with_blueprint_method(node.value):
                blueprint_vars.append(var_name)
            # Check if it's a module factory (SomeModule.blueprint)
            elif _is_blueprint_factory(node.value):
                module_vars.append(var_name)

    return blueprint_vars, module_vars


def _is_autoconnect_call(node: ast.expr) -> bool:
    if isinstance(node, ast.Call):
        func = node.func
        # Direct call: autoconnect(...)
        if isinstance(func, ast.Name) and func.id == "autoconnect":
            return True
        # Attribute call: module.autoconnect(...)
        if isinstance(func, ast.Attribute) and func.attr == "autoconnect":
            return True
    return False


def _ends_with_blueprint_method(node: ast.expr) -> bool:
    if isinstance(node, ast.Call):
        func = node.func
        if isinstance(func, ast.Attribute) and func.attr in BLUEPRINT_METHODS:
            return True
    return False


def _is_blueprint_factory(node: ast.expr) -> bool:
    if isinstance(node, ast.Attribute):
        return node.attr == "blueprint"
    return False
