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
import re
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
BLUEPRINT_METHODS = {"transports", "global_config", "remappings", "requirements", "configurators"}
_EXCLUDED_MODULE_NAMES = {"Module", "ModuleBase", "StreamModule"}


def test_all_blueprints_is_current() -> None:
    root = DIMOS_PROJECT_ROOT / "dimos"
    all_blueprints, all_modules = _scan_for_blueprints(root)

    common = set(all_blueprints.keys()) & set(all_modules.keys())
    assert not common, (
        f"Names must be unique across blueprints and modules, "
        f"but these appear in both: {sorted(common)}"
    )

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


def _camel_to_snake(name: str) -> str:
    """Convert CamelCase class name to snake_case."""
    s = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", name)
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s)
    return s.lower()


def _get_base_class_names(node: ast.ClassDef) -> list[str]:
    """Extract base class names from a ClassDef, handling Name, Attribute, and Subscript."""
    names: list[str] = []
    for base in node.bases:
        if isinstance(base, ast.Name):
            names.append(base.id)
        elif isinstance(base, ast.Attribute):
            names.append(base.attr)
        elif isinstance(base, ast.Subscript):
            # Handle Generic[T] style: class Module(ModuleBase[ConfigT])
            v = base.value
            if isinstance(v, ast.Name):
                names.append(v.id)
            elif isinstance(v, ast.Attribute):
                names.append(v.attr)
    return names


def _build_module_class_set(root: Path) -> set[str]:
    """Build the set of all class names that are Module subclasses.

    Uses the same transitive-closure approach as dimos.core.test_modules:
    start from {"Module", "ModuleBase"} and iteratively add any class whose
    base appears in the known set until convergence.
    """
    known: set[str] = {"Module", "ModuleBase"}
    all_classes: list[tuple[str, list[str]]] = []

    for path in sorted(root.rglob("*.py")):
        if "__pycache__" in str(path):
            continue
        try:
            tree = ast.parse(path.read_text("utf-8"), str(path))
        except Exception:
            continue
        for node in tree.body:
            if isinstance(node, ast.ClassDef):
                all_classes.append((node.name, _get_base_class_names(node)))

    changed = True
    while changed:
        changed = False
        for name, bases in all_classes:
            if name not in known and any(b in known for b in bases):
                known.add(name)
                changed = True

    return known


def _is_production_module_file(file_path: Path, root: Path) -> bool:
    """Return True if this file should contribute to the all_modules registry.

    Excludes test helpers, deprecated code, and framework base classes in core/.
    """
    rel = str(file_path.relative_to(root))
    stem = file_path.stem
    return not (
        stem.startswith("test_")
        or "_test_" in stem
        or stem.endswith("_test")
        or stem.startswith("fake_")
        or stem.startswith("mock_")
        or "deprecated" in rel
        or "/testing/" in rel
        or rel.startswith("core/")
    )


def _scan_for_blueprints(root: Path) -> tuple[dict[str, str], dict[str, str]]:
    all_blueprints: dict[str, str] = {}
    all_modules: dict[str, str] = {}

    module_classes = _build_module_class_set(root)

    for file_path in sorted(_get_all_python_files(root)):
        module_name = _path_to_module_name(file_path, root)
        blueprint_vars, module_vars = _find_blueprints_in_file(file_path, module_classes)

        for var_name in blueprint_vars:
            full_path = f"{module_name}:{var_name}"
            cli_name = var_name.replace("_", "-")
            all_blueprints[cli_name] = full_path

        # Only register modules from production files (skip test, deprecated, core)
        if _is_production_module_file(file_path, root):
            for var_name, class_name in module_vars:
                cli_name = var_name.replace("_", "-")
                all_modules[cli_name] = f"{module_name}.{class_name}"

    # Blueprints take priority when names collide (e.g. a pre-configured
    # blueprint named "mid360" vs the raw Mid360 Module class).
    for key in set(all_modules) & set(all_blueprints):
        del all_modules[key]

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


def _find_blueprints_in_file(
    file_path: Path, module_classes: set[str] | None = None
) -> tuple[list[str], list[tuple[str, str]]]:
    blueprint_vars: list[str] = []
    module_vars: list[tuple[str, str]] = []

    try:
        source = file_path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(file_path))
    except Exception:
        return [], []

    # Only look at top-level statements (direct children of the Module node)
    for node in tree.body:
        if isinstance(node, ast.Assign):
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

        # Detect Module subclasses by checking base classes against the known set
        elif isinstance(node, ast.ClassDef) and module_classes:
            if node.name.startswith("_") or node.name in _EXCLUDED_MODULE_NAMES:
                continue
            if any(b in module_classes for b in _get_base_class_names(node)):
                module_vars.append((_camel_to_snake(node.name), node.name))

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
