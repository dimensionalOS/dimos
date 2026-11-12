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

"""PEP 517/660 build backend for message packages in ordinary Python projects.

A project that ran ``dimos package add messages`` declares this module as its
``build-backend``. Every build hook then:

1. regenerates the Python message packages from ``interfaces/`` into
   ``build/message-source`` (never into the source tree), and
2. hands setuptools the project's own packages plus the generated packages, so
   handwritten modules and generated namespaces ship in the same distribution.

The generation itself is the published ``dimos-cdrgen`` pipeline
(``configuration``/``generate``) plus this repository's message provider
metadata; there is no second message generator here.
"""

from __future__ import annotations

import os
from pathlib import Path
import shutil
from typing import Any

from dimos_cdrgen import source_package
from dimos_cdrgen.generate import generate
from dimos_cdrgen.ownership import Dependency
from setuptools import find_namespace_packages, setup
import setuptools.build_meta as _setuptools_build

BACKEND = "dimos.message_codegen.backend"
SOURCE = "interfaces"
OUTPUT = Path("build") / "message-source"

PROVIDER_GROUP = "dimos.messages"


class MessagePackageError(RuntimeError):
    """A build input is missing or contradictory; the message is user-facing."""


def _message_configuration(project: Path) -> dict[str, Any]:
    try:
        return source_package.configuration(project)
    except ValueError as error:
        raise MessagePackageError(str(error)) from error


def installed_providers(*, exclude: tuple[str, ...] = ()) -> tuple[Dependency, ...]:
    """Message packages installed in the build environment, read from their manifests.

    The build depends on the provider that owns every imported type (the built-in
    ``dimos_messages`` package for ``std_msgs/Header``), so the versions are taken
    from the environment that builds the wheel instead of a project-maintained pin.
    The project's own provider is skipped: its entry point is declared in the
    metadata being built, but its generated module does not exist yet.
    """
    from importlib.metadata import entry_points
    from importlib.util import find_spec

    providers: list[Dependency] = []
    for entry in entry_points(group=PROVIDER_GROUP):
        module = entry.value.split(":", 1)[0]
        if entry.name in exclude or module.split(".")[0] in exclude:
            continue
        spec = find_spec(module)
        if spec is None or spec.origin is None:
            raise MessagePackageError(
                f"Installed message provider {entry.name!r} points at missing module {module!r}"
            )
        providers.append(Dependency.load(Path(spec.origin).parent / "package"))
    if not providers:
        raise MessagePackageError(
            "No installed dimOS message packages were found; install dimos in the build "
            "environment so imported types such as std_msgs/Header can be resolved"
        )
    return tuple(providers)


def generate_sources(project: Path) -> tuple[dict[str, Any], Path]:
    """Regenerate the message packages and return the configuration and output root."""
    project = project.resolve()
    config = _message_configuration(project)
    own = (config["module"], config["python_module"], config["python_module"] + "_schemas")
    dependencies = config["dependencies"] or installed_providers(exclude=own)
    output = project / OUTPUT
    # A definition deleted from interfaces/ must not survive in the build output.
    if output.exists():
        shutil.rmtree(output)
    generate(
        [config["source"]],
        output,
        config["roots"],
        config["module"],
        python_module=config["python_module"],
        distribution=config["distribution"],
        version=config["version"],
        dependencies=dependencies,
        shared=True,
    )
    python_root = output / "python"
    resources = python_root / (config["python_module"].replace(".", "/") + "_schemas") / "package"
    for path in config["resources"]:
        shutil.copyfile(path, resources / path.name)
    return config, python_root


def _project_packages(project: Path, config: dict[str, Any]) -> tuple[list[str], list[str]]:
    """The packages and modules this project installs besides the generated ones.

    ``interfaces/`` and the build directory are excluded: they are build inputs
    and output, not importable project packages.
    """
    from setuptools.discovery import FlatLayoutModuleFinder, FlatLayoutPackageFinder

    source = config["source"].relative_to(project).parts[0]
    excluded = [source, "build"]

    class PackageFinder(FlatLayoutPackageFinder):
        ALWAYS_EXCLUDE = (*FlatLayoutPackageFinder.ALWAYS_EXCLUDE, *excluded)

    class ModuleFinder(FlatLayoutModuleFinder):
        ALWAYS_EXCLUDE = (*FlatLayoutModuleFinder.ALWAYS_EXCLUDE, *excluded)

    packages = list(PackageFinder.find(str(project)))
    modules = list(ModuleFinder.find(str(project)))
    if not packages and not modules and (project / "src").is_dir():
        raise MessagePackageError(
            "This project looks like a src-layout; the dimos message backend discovers "
            "flat-layout projects only. Move the package to the project root or keep "
            "using setuptools with setup(**setup_arguments(...))"
        )
    return packages, modules


def _package_data(project: Path, config: dict[str, Any]) -> dict[str, list[str]]:
    module = config["python_module"]
    generated = {
        module: ["py.typed", "*.pyi", "**/*.pyi"],
        module + "_schemas": ["schemas.json", "schemas/**/*", "package/**/*"],
    }
    declared = _declared_package_data(project)
    merged: dict[str, list[str]] = {key: list(value) for key, value in declared.items()}
    for key, patterns in generated.items():
        merged[key] = sorted({*merged.get(key, []), *patterns})
    return merged


def _declared_package_data(project: Path) -> dict[str, list[str]]:
    import tomllib

    data = tomllib.loads((project / "pyproject.toml").read_text())
    section = data.get("tool", {}).get("setuptools", {}).get("package-data", {})
    if not isinstance(section, dict):
        return {}
    return {key: list(value) for key, value in section.items() if isinstance(value, list)}


def _merge_init(path: Path, user: Path) -> None:
    """Keep the project's ``__init__.py`` body and append the generated metadata.

    The user's file goes first so a leading ``from __future__`` import stays
    valid; the generated imports and version markers only need to run.
    """
    generated = [
        line
        for line in path.read_text().splitlines(keepends=True)
        if not line.startswith("from __future__ ")
    ]
    path.write_text(user.read_text().rstrip("\n") + "\n" + "".join(generated))


def _overlay(python_root: Path, project: Path, config: dict[str, Any]) -> None:
    """Merge the project's own files into the generated namespace package.

    Generated message code lives in the same top-level package as the project's
    handwritten modules (``acme_telemetry/demo_msgs`` next to
    ``acme_telemetry/app.py``) and imports its ``_types`` sibling. Linking the
    project's entries into the build output keeps one package directory per
    top-level name without touching the source tree.
    """
    module = config["python_module"]
    relative = Path(*module.split("."))
    generated = python_root / relative
    user = project / relative
    if not user.is_dir() or not generated.is_dir():
        return
    for entry in sorted(user.iterdir()):
        if entry.name == "__pycache__" or entry.suffix in (".pyc", ".pyo"):
            continue
        target = generated / entry.name
        if entry.name == "__init__.py" and target.is_file():
            _merge_init(target, entry)
            continue
        if target.exists() or target.is_symlink():
            if target.is_file() and entry.is_file() and target.read_bytes() == entry.read_bytes():
                continue
            raise MessagePackageError(
                f"{entry.relative_to(project)} collides with the generated message file "
                f"{target.relative_to(python_root)}; rename or move the project file, or move "
                f"the schemas to another module with [tool.dimos.messages] module = ..."
            )
        try:
            target.symlink_to(os.path.relpath(entry, generated), target_is_directory=entry.is_dir())
        except OSError:
            if entry.is_dir():
                shutil.copytree(entry, target)
            else:
                shutil.copyfile(entry, target)


def build_arguments(project: Path) -> dict[str, Any]:
    """The setuptools configuration for one build of ``project``."""
    config, python_root = generate_sources(project)
    _overlay(python_root, project, config)
    module = config["python_module"]
    packages, py_modules = _project_packages(project, config)
    package_dir: dict[str, str] = {"": "."}
    generated = find_namespace_packages(
        str(python_root),
        include=[module, module + ".*", module + "_schemas", module + "_schemas.*"],
    )
    merged_root = (python_root / module.replace(".", "/")).relative_to(project)
    for name in generated:
        if name == module and (project / module.replace(".", "/")).is_dir():
            # The merged build tree carries the project files plus the schemas.
            package_dir[name] = str(merged_root)
            continue
        if name in packages:
            # The project owns this package directory; the generated tree only
            # contributes the schema subpackages mapped below.
            continue
        packages.append(name)
        package_dir[name] = str((python_root / name.replace(".", "/")).relative_to(project))
    if not packages:
        raise MessagePackageError(
            "No Python packages were found to build; add a module to the project or a "
            "message under interfaces/<schema>/msg/"
        )
    arguments: dict[str, Any] = {
        "packages": packages,
        "package_dir": package_dir,
        "package_data": _package_data(project, config),
        "cmdclass": {"sdist": source_package.SourceDistribution},
    }
    if py_modules:
        arguments["py_modules"] = py_modules
    return arguments


class _Backend(_setuptools_build._BuildMetaBackend):  # type: ignore[attr-defined]
    """setuptools' backend, with generation as a pre-setup step."""

    def run_setup(self, setup_script: str = "setup.py") -> None:
        project = Path.cwd()
        if (project / FORBIDDEN_SCRIPT).exists():
            raise MessagePackageError(
                "This project declares the dimos message backend but still has a setup.py; "
                "keep setup(**setup_arguments(...)) with setuptools.build_meta or remove the "
                "file and build through the dimos backend"
            )
        setup(**build_arguments(project))


FORBIDDEN_SCRIPT = "setup.py"

_backend = _Backend()

get_requires_for_build_wheel = _backend.get_requires_for_build_wheel
get_requires_for_build_sdist = _backend.get_requires_for_build_sdist
get_requires_for_build_editable = _backend.get_requires_for_build_editable
prepare_metadata_for_build_wheel = _backend.prepare_metadata_for_build_wheel
prepare_metadata_for_build_editable = _backend.prepare_metadata_for_build_editable
build_wheel = _backend.build_wheel
build_editable = _backend.build_editable
build_sdist = _backend.build_sdist
