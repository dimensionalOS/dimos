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
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.deps.bundles import BUNDLES_PATH
from dimos.robot.get_all_blueprints import class_name_to_registry_key

if sys.version_info >= (3, 11):
    import tomllib
else:  # pytest depends on tomli below 3.11
    import tomli as tomllib

IGNORED_FILES: set[str] = {
    "dimos/robot/all_blueprints.py",
    "dimos/robot/get_all_blueprints.py",
    "dimos/robot/test_all_blueprints.py",
    "dimos/robot/test_all_blueprints_generation.py",
    "dimos/core/blueprints.py",
    "dimos/core/test_blueprints.py",
}
# Terminal builder methods that mark a top-level blueprint expression. "blueprint"
# is included so a bare single-module `X.blueprint(...)` (no transports/remappings
# override needed) is still discovered as a runnable blueprint.
BLUEPRINT_METHODS = {
    "blueprint",
    "transports",
    "global_config",
    "remappings",
    "requirements",
    "configurators",
}
_EXCLUDED_MODULE_NAMES = {"Module", "ModuleBase", "StreamModule"}
# Module-level literal naming the dependency bundle of a file's blueprints and modules;
# dimos/deps/bundles.json is generated from it. A file without one inherits the literal of
# the nearest ancestor directory's DEPENDENCY_BUNDLE_FILE.
DEPENDENCY_BUNDLE_NAME = "DEPENDENCY_BUNDLE"
DEPENDENCY_BUNDLE_FILE = "dependency_bundle.py"


def test_all_blueprints_is_current() -> None:
    root = DIMOS_PROJECT_ROOT / "dimos"
    all_blueprints, all_modules = _scan_for_blueprints(root)

    common = set(all_blueprints.keys()) & set(all_modules.keys())
    assert not common, (
        f"Names must be unique across blueprints and modules, "
        f"but these appear in both: {sorted(common)}"
    )

    _sync_generated_file(
        root / "robot" / "all_blueprints.py",
        _generate_all_blueprints_content(all_blueprints, all_modules),
    )


def test_dependency_bundles_are_declared_and_current() -> None:
    """Every file with a registered entry declares DEPENDENCY_BUNDLE; bundles.json mirrors them."""
    root = DIMOS_PROJECT_ROOT / "dimos"
    _, _, bundles, undeclared = _scan_registry(root)

    assert not undeclared, (
        "files that define a registered blueprint or module without a "
        f"{DEPENDENCY_BUNDLE_NAME} literal, in the file or in a {DEPENDENCY_BUNDLE_FILE} of an "
        f"ancestor directory: {undeclared}"
    )
    with (DIMOS_PROJECT_ROOT / "pyproject.toml").open("rb") as f:
        extras = tomllib.load(f)["project"]["optional-dependencies"]
    unknown = sorted(set(bundles.values()) - set(extras))
    assert not unknown, (
        f"{DEPENDENCY_BUNDLE_NAME} names extras missing from pyproject.toml: {unknown}"
    )

    _sync_generated_file(BUNDLES_PATH, _generate_bundles_content(bundles))


def _sync_generated_file(file_path: Path, generated_content: str) -> None:
    """Regenerate locally; in CI only compare, so a stale generated file fails the build."""
    if "CI" in os.environ:
        if not file_path.exists():
            pytest.fail(f"{file_path.name} does not exist at {file_path}")

        current_content = file_path.read_text()
        if current_content != generated_content:
            diff = difflib.unified_diff(
                current_content.splitlines(keepends=True),
                generated_content.splitlines(keepends=True),
                fromfile=f"{file_path.name} (current)",
                tofile=f"{file_path.name} (generated)",
            )
            diff_str = "".join(diff)
            pytest.fail(
                f"{file_path.name} is out of date. Run "
                f"`pytest dimos/robot/test_all_blueprints_generation.py` locally to update.\n\n"
                f"Diff:\n{diff_str}"
            )
    else:
        file_path.write_text(generated_content)

        if _check_for_uncommitted_changes(file_path):
            pytest.fail(
                f"{file_path.name} was updated and has uncommitted changes. "
                "Please commit the changes."
            )


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

    for path in sorted(_get_all_python_files(root)):
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
    relative_path = file_path.relative_to(root)
    rel = str(relative_path)
    stem = file_path.stem
    return not (
        stem.startswith("test_")
        or "_test_" in stem
        or stem.endswith("_test")
        or stem.startswith("tool_")
        or stem.startswith("fake_")
        or stem.startswith("mock_")
        or "deprecated" in rel
        or "/testing/" in rel
        or "example" in relative_path.parts
        or relative_path == Path("experimental/isolated_python/module.py")
        or rel.startswith("core/")
    )


@pytest.mark.parametrize(
    "relative_path",
    [
        "experimental/isolated_python/example/contract.py",
        "experimental/isolated_python/example/support.py",
        "experimental/isolated_python/module.py",
    ],
)
def test_isolated_python_framework_is_not_a_production_module(
    tmp_path: Path,
    relative_path: str,
) -> None:
    assert _is_production_module_file(tmp_path / relative_path, tmp_path) is False


def _scan_for_blueprints(root: Path) -> tuple[dict[str, str], dict[str, str]]:
    all_blueprints, all_modules, _, _ = _scan_registry(root)
    return all_blueprints, all_modules


def _scan_registry(
    root: Path,
) -> tuple[dict[str, str], dict[str, str], dict[str, str], list[str]]:
    """Registry entries, each entry's dependency bundle, and files missing a declaration."""
    all_blueprints: dict[str, str] = {}
    all_modules: dict[str, str] = {}
    blueprint_bundles: dict[str, str] = {}
    module_bundles: dict[str, str] = {}
    undeclared: list[str] = []

    module_classes = _build_module_class_set(root)
    directory_bundles: dict[Path, str | None] = {}

    for file_path in sorted(_get_all_python_files(root)):
        module_name = _path_to_module_name(file_path, root)
        blueprint_vars, module_vars = _find_blueprints_in_file(file_path, module_classes)
        if not _is_production_module_file(file_path, root):
            # Only register modules from production files (skip test, deprecated, core)
            module_vars = []
        if not blueprint_vars and not module_vars:
            continue

        bundle = _dependency_bundle_for(file_path, root, directory_bundles)
        if bundle is None:
            undeclared.append(str(file_path.relative_to(root.parent)))

        for var_name in blueprint_vars:
            cli_name = var_name.replace("_", "-")
            all_blueprints[cli_name] = f"{module_name}:{var_name}"
            if bundle is not None:
                blueprint_bundles[cli_name] = bundle
        for class_name in module_vars:
            key = class_name_to_registry_key(class_name)
            all_modules[key] = f"{module_name}.{class_name}"
            if bundle is not None:
                module_bundles[key] = bundle

    # Blueprints take priority when names collide (e.g. a pre-configured
    # blueprint named "mid360" vs the raw Mid360 Module class).
    for key in set(all_modules) & set(all_blueprints):
        del all_modules[key]
        module_bundles.pop(key, None)

    return all_blueprints, all_modules, {**module_bundles, **blueprint_bundles}, undeclared


def _dependency_bundle_for(
    file_path: Path, root: Path, directory_bundles: dict[Path, str | None]
) -> str | None:
    """The file's own literal, else the nearest ancestor directory's marker file, else None."""
    own = _dependency_bundle_in_file(file_path)
    if own is not None:
        return own
    directory = file_path.parent
    while True:
        if directory not in directory_bundles:
            marker = directory / DEPENDENCY_BUNDLE_FILE
            directory_bundles[directory] = (
                _dependency_bundle_in_file(marker) if marker.is_file() else None
            )
        inherited = directory_bundles[directory]
        if inherited is not None or directory == root:
            return inherited
        directory = directory.parent


def _dependency_bundle_in_file(file_path: Path) -> str | None:
    """The module-level ``DEPENDENCY_BUNDLE`` string literal, or None when absent."""
    tree = ast.parse(file_path.read_text(encoding="utf-8"), filename=str(file_path))
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if not any(
            isinstance(target, ast.Name) and target.id == DEPENDENCY_BUNDLE_NAME
            for target in node.targets
        ):
            continue
        value = node.value
        if isinstance(value, ast.Constant) and isinstance(value.value, str) and value.value:
            return value.value
        raise ValueError(
            f"{file_path}: {DEPENDENCY_BUNDLE_NAME} must be a non-empty string literal"
        )
    return None


def _generate_bundles_content(bundles: dict[str, str]) -> str:
    """bundles.json: each bundle extra -> the sorted registry names it covers."""
    grouped = {
        bundle: sorted(name for name, assigned in bundles.items() if assigned == bundle)
        for bundle in sorted(set(bundles.values()))
    }
    return json.dumps(grouped, indent=2) + "\n"


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
    for directory, children, filenames in os.walk(root):
        parent = Path(directory)
        children[:] = [
            name
            for name in children
            if name != "__pycache__" and not (parent / name / "pyproject.toml").is_file()
        ]
        for name in filenames:
            path = parent / name
            if path.suffix == ".py" and str(path.relative_to(root.parent)) not in IGNORED_FILES:
                yield path


def _path_to_module_name(path: Path, root: Path) -> str:
    parts = list(path.relative_to(root.parent).parts)
    parts[-1] = parts[-1].removesuffix(".py")
    return ".".join(parts)


def _find_blueprints_in_file(
    file_path: Path, module_classes: set[str] | None = None
) -> tuple[list[str], list[str]]:
    blueprint_vars: list[str] = []
    module_vars: list[str] = []

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
                module_vars.append(node.name)

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


def test_nested_projects_do_not_contribute_modules_or_blueprints(tmp_path: Path) -> None:
    root = tmp_path / "dimos"
    runtime = root / "provider/python"
    runtime.mkdir(parents=True)
    (runtime / "pyproject.toml").write_text('[project]\nname = "runtime"\n')
    (root / "provider/contract.py").write_text(
        "class HostContract(Module): pass\n"
        "class RuntimeOnlyBase: pass\n"
        "class NotAModule(RuntimeOnlyBase): pass\n"
        "host_blueprint = HostContract.blueprint()\n"
    )
    (runtime / "runtime.py").write_text(
        "class PublicRuntime(HostContract): pass\n"
        "class RuntimeOnlyBase(Module): pass\n"
        "runtime_blueprint = PublicRuntime.blueprint()\n"
    )

    blueprints, modules = _scan_for_blueprints(root)

    assert blueprints == {"host-blueprint": "dimos.provider.contract:host_blueprint"}
    assert modules == {"host-contract": "dimos.provider.contract.HostContract"}


def test_dependency_bundles_follow_the_file_declaration(tmp_path: Path) -> None:
    root = tmp_path / "dimos"
    root.mkdir()
    (root / "declared.py").write_text(
        'DEPENDENCY_BUNDLE = "runtime-drone"\n'
        "class Declared(Module): pass\n"
        "declared_blueprint = Declared.blueprint()\n"
    )
    (root / "undeclared.py").write_text("class Undeclared(Module): pass\n")

    _, _, bundles, undeclared = _scan_registry(root)

    assert bundles == {"declared-blueprint": "runtime-drone", "declared": "runtime-drone"}
    assert undeclared == ["dimos/undeclared.py"]
    assert _generate_bundles_content(bundles) == (
        '{\n  "runtime-drone": [\n    "declared",\n    "declared-blueprint"\n  ]\n}\n'
    )


def test_dependency_bundles_inherit_from_the_nearest_directory_marker(tmp_path: Path) -> None:
    root = tmp_path / "dimos"
    (root / "family/sub").mkdir(parents=True)
    (root / "other").mkdir()
    (root / "family/dependency_bundle.py").write_text('DEPENDENCY_BUNDLE = "runtime-drone"\n')
    (root / "family/a.py").write_text("class A(Module): pass\n")
    (root / "family/sub/dependency_bundle.py").write_text('DEPENDENCY_BUNDLE = "runtime-spot"\n')
    (root / "family/sub/b.py").write_text("class B(Module): pass\n")
    (root / "family/sub/c.py").write_text(
        'DEPENDENCY_BUNDLE = "runtime-common"\nclass C(Module): pass\n'
    )
    (root / "other/d.py").write_text("class D(Module): pass\n")

    _, _, bundles, undeclared = _scan_registry(root)

    assert bundles == {"a": "runtime-drone", "b": "runtime-spot", "c": "runtime-common"}
    assert undeclared == ["dimos/other/d.py"]


def test_dependency_bundle_must_be_a_string_literal(tmp_path: Path) -> None:
    path = tmp_path / "computed.py"
    path.write_text('BUNDLE = "runtime-common"\nDEPENDENCY_BUNDLE = BUNDLE\n')

    with pytest.raises(ValueError, match="non-empty string literal"):
        _dependency_bundle_in_file(path)
