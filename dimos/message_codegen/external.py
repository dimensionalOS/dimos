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

"""One-time setup of message packaging in an existing external Python project.

``dimos package add messages`` calls :func:`plan` and :func:`apply`. The plan is
only built after every refusal condition has been checked, and applying it edits
the project's ``pyproject.toml`` in place through ``tomlkit``, which keeps
comments, ordering, quoting and every unrelated value untouched.

Nothing is written before the planned document has been re-parsed and compared
with the parsed input, so an unsupported project becomes an :class:`AddError`
and leaves the tree exactly as it was.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib.metadata import PackageNotFoundError, version
import keyword
from pathlib import Path
import re
from typing import Any

import tomlkit
from tomlkit.container import OutOfOrderTableProxy
from tomlkit.items import Array, Table
from tomlkit.toml_document import TOMLDocument

BACKEND = "dimos.message_codegen.backend"
SETUPTOOLS_BACKEND = "setuptools.build_meta"
DEFAULT_REQUIRES = ("setuptools>=70", "wheel")
PROVIDER_GROUP = "dimos.messages"
SOURCE = "interfaces"
UNSUPPORTED_SETUPTOOLS_KEYS = ("packages", "package-dir", "py-modules")
# Arrays and keys this command rewrites; every other value is compared before/after.
REWRITTEN_PATHS = (
    "build-system.requires",
    "build-system.build-backend",
    "project.dependencies",
)

_VERSION = re.compile(r"^[0-9]+(?:\.[0-9]+){2}$")


class AddError(RuntimeError):
    """The project cannot be set up automatically; the message explains why."""


@dataclass(frozen=True)
class Addition:
    project: Path
    module: str
    text: str
    changes: tuple[str, ...]
    notes: tuple[str, ...]

    @property
    def changed(self) -> bool:
        return bool(self.changes)


def default_module(name: str) -> str:
    """The Python namespace generated messages land in, from the project name."""
    module = name.replace("-", "_")
    if not all(part.isidentifier() and not keyword.iskeyword(part) for part in module.split(".")):
        raise AddError(
            f"Project name {name!r} does not map to a Python package name; add "
            '"[tool.dimos.messages] module = ..." with a valid module name manually'
        )
    return module


def requirement(dimos_version: str | None = None) -> str:
    if dimos_version is None:
        try:
            dimos_version = version("dimos")
        except PackageNotFoundError:
            return "dimos"
    return f"dimos>={dimos_version}"


def requirement_name(text: str) -> str | None:
    """The distribution name a PEP 508 requirement string asks for."""
    if not isinstance(text, str):
        return None
    match = re.match(r"\s*([A-Za-z0-9][A-Za-z0-9._-]*)", text)
    if match is None:
        return None
    return re.sub(r"[-_.]+", "-", match.group(1)).lower()


# --- document access ------------------------------------------------------


def _dotted(path: tuple[str, ...]) -> str:
    return "[" + ".".join(path) + "]"


def _parse(path: Path, text: str) -> TOMLDocument:
    try:
        return tomlkit.parse(text)
    except tomlkit.exceptions.ParseError as error:
        raise AddError(f"{path} is not valid TOML: {error}") from error


def _load(project: Path) -> tuple[str, TOMLDocument, dict[str, Any]]:
    path = project / "pyproject.toml"
    if not path.is_file():
        raise AddError(f"No pyproject.toml in {project}; run this from a Python project directory")
    text = path.read_text()
    document = _parse(path, text)
    return text, document, document.unwrap()


def _nested(data: Any, path: tuple[str, ...]) -> Any:
    node = data
    for key in path:
        if not isinstance(node, dict):
            return None
        node = node.get(key)
    return node


def _first_body_item(document: TOMLDocument) -> Any:
    """The first thing the document declares, skipping comments and blank lines."""
    for key, item in document.body:
        if key is not None:
            return item
    return None


def _explicitly_declared(document: TOMLDocument, name: str) -> bool:
    """Whether ``name`` has a real ``[name]`` header rather than only dotted keys."""
    for key, item in document.body:
        if key is None or not isinstance(item, Table):
            continue
        if str(key) == name and not item.is_super_table():
            return True
    return False


def _table_at(document: TOMLDocument, path: tuple[str, ...]) -> Table | OutOfOrderTableProxy:
    """The table at ``path``, created when absent."""
    node: Any = document
    for depth in range(len(path)):
        key = path[depth]
        here = path[: depth + 1]
        try:
            current = node[key]
        except KeyError:
            current = None
        if current is None:
            current = tomlkit.table()
            node[key] = current
        elif not isinstance(current, (Table, OutOfOrderTableProxy)):
            raise AddError(f"{_dotted(here)} is not a table; edit pyproject.toml by hand")
        node = current
    if not isinstance(node, (Table, OutOfOrderTableProxy)):  # pragma: no cover - defensive
        raise AddError(f"{_dotted(path)} is not a table; edit pyproject.toml by hand")
    return node


def _array(table: Table | OutOfOrderTableProxy, key: str, owner: str) -> Array:
    """The string array ``key`` of ``owner``, refusing anything else."""
    try:
        value = table[key]
    except KeyError:
        raise AddError(
            f"{owner} has no {key} array to extend; add the dimos requirement by hand"
        ) from None
    if not isinstance(value, Array) or not all(isinstance(item, str) for item in value):
        raise AddError(
            f"{owner} {key} must be an array of requirement strings; edit pyproject.toml by hand"
        )
    return value


# --- validation -----------------------------------------------------------


def _validate(data: dict[str, Any], project: Path) -> dict[str, Any]:
    project_table = data.get("project")
    if not isinstance(project_table, dict):
        raise AddError("pyproject.toml has no [project] table; add one before running this command")
    name = project_table.get("name")
    if not isinstance(name, str) or not name:
        raise AddError('[project] requires a "name"')
    message_version = project_table.get("version")
    if not isinstance(message_version, str):
        raise AddError(
            '[project] requires a literal "version"; generated message metadata is derived from it'
        )
    if not _VERSION.fullmatch(message_version):
        raise AddError(
            f"[project] version {message_version!r} is not a X.Y.Z release version; message "
            "packages derive that exact form (for example 0.1.0)"
        )
    for legacy in ("setup.py", "setup.cfg"):
        if (project / legacy).is_file():
            raise AddError(
                f"{legacy} exists, so this project already has a hand-written build entry. "
                "External projects that call setup(**setup_arguments(...)) keep their current "
                "build; remove the file yourself if you want the dimos backend instead"
            )
    build_system = data.get("build-system", {})
    backend = build_system.get("build-backend", SETUPTOOLS_BACKEND)
    if backend not in (SETUPTOOLS_BACKEND, BACKEND):
        raise AddError(
            f"build-backend {backend!r} is not setuptools.build_meta; this command only "
            "extends a setuptools project and never replaces another build backend"
        )
    setuptools_table = _nested(data, ("tool", "setuptools"))
    if isinstance(setuptools_table, dict):
        present = [key for key in UNSUPPORTED_SETUPTOOLS_KEYS if key in setuptools_table]
        if present:
            raise AddError(
                "[tool.setuptools] already declares "
                + ", ".join(sorted(present))
                + "; the dimos backend owns package discovery for generated messages. "
                "Move that configuration into ordinary package layout first"
            )
    return project_table


def _configured(data: dict[str, Any]) -> bool:
    return isinstance(_nested(data, ("tool", "dimos", "messages")), dict)


def _has_dimos(values: list[str]) -> bool:
    return any(requirement_name(value) == "dimos" for value in values)


def _unchanged(before: Any, after: Any, path: tuple[str, ...] = ()) -> None:
    """Every value outside the rewritten arrays must survive the edit."""
    dotted = ".".join(path)
    if dotted in REWRITTEN_PATHS:
        return
    if isinstance(before, dict):
        if not isinstance(after, dict) or not before.keys() <= after.keys():
            raise AddError(f"internal error: {dotted or 'the document'} lost keys; not written")
        for key, value in before.items():
            _unchanged(value, after[key], (*path, key))
    elif isinstance(before, list):
        if not isinstance(after, list):
            raise AddError(f"internal error: {dotted} changed type; not written")
    elif before != after:
        raise AddError(f"internal error: {dotted} changed; not written")


def _verify(
    before: dict[str, Any], text: str, path: Path, requirement_text: str, module: str
) -> None:
    """Re-parse the planned document and prove it says exactly what we meant."""
    try:
        after = tomlkit.parse(text).unwrap()
    except tomlkit.exceptions.ParseError as error:
        raise AddError(f"internal error: generated pyproject.toml is invalid: {error}") from error
    _unchanged(before, after)
    old_build = before.get("build-system", {})
    old_requires = old_build.get("requires", [])
    if not isinstance(old_requires, list):  # pragma: no cover - checked by the caller
        raise AddError("internal error: cannot read [build-system] requires; not written")
    expected_requires = (
        old_requires if _has_dimos(old_requires) else [*old_requires, requirement_text]
    )
    if before.get("build-system") is None:
        expected_requires = [*DEFAULT_REQUIRES, requirement_text]
    new_build = after.get("build-system", {})
    if new_build.get("requires") != expected_requires:
        raise AddError("internal error: [build-system] requires changed unexpectedly; not written")
    if new_build.get("build-backend") != BACKEND:
        raise AddError("internal error: build-backend was not set; not written")
    old_project = before["project"]
    dynamic = old_project.get("dynamic") or []
    if "dependencies" not in dynamic:
        old_dependencies = old_project.get("dependencies", [])
        expected_dependencies = (
            old_dependencies
            if _has_dimos(old_dependencies)
            else [*old_dependencies, requirement_text]
        )
        if after["project"].get("dependencies") != expected_dependencies:
            raise AddError(
                "internal error: [project] dependencies changed unexpectedly; not written"
            )
    settings = _nested(after, ("tool", "dimos", "messages"))
    if settings != {"source": SOURCE}:
        raise AddError("internal error: [tool.dimos.messages] is incomplete; not written")
    entry_points = _nested(after, ("project", "entry-points", PROVIDER_GROUP))
    if not isinstance(entry_points, dict) or entry_points.get(module) != (
        f"{module}_schemas.provider"
    ):
        raise AddError("internal error: the message provider entry point is missing; not written")


# --- plan -----------------------------------------------------------------


def _plan_build_system(
    document: TOMLDocument,
    data: dict[str, Any],
    text: str,
    path: Path,
    requirement_text: str,
    changes: list[str],
) -> TOMLDocument:
    build_system = data.get("build-system")
    if build_system is None:
        system = tomlkit.table()
        system["requires"] = [*DEFAULT_REQUIRES, requirement_text]
        system["build-backend"] = BACKEND
        changes.append("added [build-system]")
        first = _first_body_item(document)
        if isinstance(first, Table) and not first.is_super_table():
            # The file already starts with a table header, so a new leading table
            # cannot change the scope of anything that follows.
            header = tomlkit.document()
            header["build-system"] = system
            return _parse(path, tomlkit.dumps(header) + "\n" + text)
        document["build-system"] = system
        return document
    table = _table_at(document, ("build-system",))
    requires = build_system.get("requires")
    if not isinstance(requires, list):
        raise AddError(
            "[build-system] requires must be an array of requirement strings; "
            "edit pyproject.toml by hand"
        )
    if not all(isinstance(value, str) for value in requires):
        raise AddError(
            "[build-system] requires contains a non-string entry; edit pyproject.toml by hand"
        )
    if not _has_dimos(requires):
        _array(table, "requires", "[build-system]").append(requirement_text)
        changes.append(f"added {requirement_text} to [build-system] requires")
    if build_system.get("build-backend", SETUPTOOLS_BACKEND) != BACKEND:
        table["build-backend"] = BACKEND
        changes.append(f"set build-backend to {BACKEND}")
    return document


def _plan_dependencies(
    document: TOMLDocument,
    project_table: dict[str, Any],
    requirement_text: str,
    changes: list[str],
    notes: list[str],
) -> None:
    dynamic = project_table.get("dynamic") or []
    if "dependencies" in dynamic:
        notes.append(
            f'[project] declares dynamic "dependencies"; add {requirement_text!r} through '
            "the mechanism that produces them"
        )
        return
    table = _table_at(document, ("project",))
    dependencies = project_table.get("dependencies")
    if dependencies is None:
        table["dependencies"] = [requirement_text]
        changes.append(f"added {requirement_text} to [project] dependencies")
        return
    if not isinstance(dependencies, list):
        raise AddError(
            "[project] dependencies must be an array of requirement strings; "
            "edit pyproject.toml by hand"
        )
    if not all(isinstance(value, str) for value in dependencies):
        raise AddError(
            "[project] dependencies contains a non-string entry; edit pyproject.toml by hand"
        )
    if not _has_dimos(dependencies):
        _array(table, "dependencies", "[project]").append(requirement_text)
        changes.append(f"added {requirement_text} to [project] dependencies")


def _plan_messages(
    document: TOMLDocument, data: dict[str, Any], module: str, changes: list[str]
) -> None:
    """Declare the message source table and the provider entry point."""
    messages = tomlkit.table()
    messages.add(
        tomlkit.comment(f"`.msg` definitions live under {SOURCE}/; every pip build regenerates the")
    )
    messages.add(tomlkit.comment("Python packages below `build/message-source` from them."))
    messages["source"] = SOURCE
    _table_at(document, ("tool", "dimos"))["messages"] = messages
    changes.append(f"declared [tool.dimos.messages] source = {SOURCE!r}")

    existing = _nested(data, ("project", "entry-points", PROVIDER_GROUP))
    if isinstance(existing, dict) and module in existing:
        return
    if not isinstance(existing, dict) and not _explicitly_declared(document, "project"):
        raise AddError(
            "[project] is declared with dotted keys; add "
            f'[project.entry-points."{PROVIDER_GROUP}"] with {module} = '
            f'"{module}_schemas.provider" by hand'
        )
    provider = _table_at(document, ("project", "entry-points", PROVIDER_GROUP))
    provider[module] = f"{module}_schemas.provider"
    changes.append(f"registered the {module!r} message provider")


def _plan_package_data(
    document: TOMLDocument, data: dict[str, Any], module: str, changes: list[str]
) -> None:
    """Declare the generated schema data where setuptools reads package data.

    A project that already has ``[tool.setuptools.package-data]`` keeps its own
    entries: build-time ``package_data`` arguments do not override that table.
    """
    required = {
        module: ["py.typed", "*.pyi", "**/*.pyi"],
        module + "_schemas": ["schemas.json", "schemas/**/*", "package/**/*"],
    }
    table = _table_at(document, ("tool", "setuptools", "package-data"))
    declared = _nested(data, ("tool", "setuptools", "package-data"))
    if not isinstance(declared, dict):
        for key, values in required.items():
            table[key] = values
        changes.append("declared the generated schema package data")
        return
    added = False
    for key, values in required.items():
        existing = declared.get(key)
        if existing is None:
            table[key] = values
            added = True
            continue
        if not isinstance(existing, list) or not all(isinstance(value, str) for value in existing):
            raise AddError(
                f"[tool.setuptools.package-data] {key} must be an array of strings; "
                "edit pyproject.toml by hand"
            )
        array = _array(table, key, "[tool.setuptools.package-data]")
        for value in values:
            if value not in existing:
                array.append(value)
                added = True
    if added:
        changes.append("extended [tool.setuptools.package-data] with the generated schemas")


def plan(project: Path, dimos_requirement: str | None = None) -> Addition:
    """Work out the minimal edit that turns ``project`` into a message package."""
    project = project.resolve()
    text, document, data = _load(project)
    project_table = _validate(data, project)
    module = default_module(project_table["name"])
    if _configured(data):
        return Addition(project, module, text, (), ("already configured; no files changed",))

    requirement_text = dimos_requirement or requirement()
    if requirement_name(requirement_text) != "dimos":
        raise AddError(f"{requirement_text!r} is not a dimos requirement")
    changes: list[str] = []
    notes: list[str] = []
    document = _plan_build_system(
        document, data, text, project / "pyproject.toml", requirement_text, changes
    )
    _plan_dependencies(document, project_table, requirement_text, changes, notes)
    _plan_messages(document, data, module, changes)
    _plan_package_data(document, data, module, changes)

    planned = tomlkit.dumps(document)
    _verify(data, planned, project / "pyproject.toml", requirement_text, module)
    return Addition(project, module, planned, tuple(changes), tuple(notes))


def apply(addition: Addition) -> list[str]:
    """Write the planned edit; returns the project files created or updated."""
    if not addition.changed:
        return []
    written: list[str] = []
    source = addition.project / SOURCE
    if not source.is_dir():
        source.mkdir(parents=True)
        written.append(str(source.relative_to(addition.project)) + "/")
    pyproject = addition.project / "pyproject.toml"
    pyproject.write_text(addition.text)
    written.append(str(pyproject.relative_to(addition.project)))
    return written
