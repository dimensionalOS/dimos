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
the project's ``pyproject.toml`` in place, preserving comments, ordering and all
unrelated configuration.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib.metadata import PackageNotFoundError, version
import keyword
from pathlib import Path
import re

import tomllib

BACKEND = "dimos.message_codegen.backend"
SETUPTOOLS_BACKEND = "setuptools.build_meta"
PROVIDER_GROUP = "dimos.messages"
SOURCE = "interfaces"
UNSUPPORTED_SETUPTOOLS_KEYS = ("packages", "package-dir", "py-modules")

_VERSION = re.compile(r"^[0-9]+\.[0-9]+\.[0-9]+$")
_TABLE = re.compile(r"^\[(.+)\]\s*$")


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


def _load(project: Path) -> tuple[str, dict]:
    path = project / "pyproject.toml"
    if not path.is_file():
        raise AddError(f"No pyproject.toml in {project}; run this from a Python project directory")
    text = path.read_text()
    try:
        return text, tomllib.loads(text)
    except tomllib.TOMLDecodeError as error:
        raise AddError(f"{path} is not valid TOML: {error}") from error


def _validate(data: dict, project: Path) -> dict:
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
    setuptools_table = data.get("tool", {}).get("setuptools", {})
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


def _configured(data: dict) -> bool:
    settings = data.get("tool", {}).get("dimos", {}).get("messages")
    return isinstance(settings, dict)


def _table_span(text: str, header: str) -> tuple[int, int] | None:
    """Character span of a table body (excluding its header line)."""
    lines = text.splitlines(keepends=True)
    offset = 0
    start: int | None = None
    for line in lines:
        stripped = line.strip()
        if start is None:
            if stripped == header:
                start = offset + len(line)
            offset += len(line)
            continue
        if _TABLE.match(stripped) and not stripped.startswith("[["):
            return start, offset
        offset += len(line)
    if start is None:
        return None
    return start, len(text)


def _set_requires(text: str, requirement_text: str) -> tuple[str, bool]:
    start, end = _table_span(text, "[build-system]")  # type: ignore[misc]
    body = text[start:end]
    match = re.search(r"requires\s*=\s*\[(.*?)\]", body, re.DOTALL)
    if match is None:
        raise AddError("[build-system] has no requires array to extend")
    entries = [entry.strip().strip("\"'") for entry in match.group(1).split(",") if entry.strip()]
    if requirement_text in entries or any(
        entry.split(">")[0].split("=")[0].strip() == "dimos" for entry in entries
    ):
        return text, False
    entries.append(requirement_text)
    replacement = "requires = [\n" + "".join(f'    "{entry}",\n' for entry in entries) + "]"
    body = body[: match.start()] + replacement + body[match.end() :]
    return text[:start] + body + text[end:], True


def _set_backend(text: str) -> tuple[str, bool]:
    start, end = _table_span(text, "[build-system]")  # type: ignore[misc]
    body = text[start:end]
    match = re.search(r'^build-backend[ \t]*=[ \t]*"[^"]*"[ \t]*$', body, re.MULTILINE)
    if match is None:
        raise AddError("[build-system] has no build-backend line to replace")
    if match.group(0).strip().endswith(f'"{BACKEND}"'):
        return text, False
    body = body[: match.start()] + f'build-backend = "{BACKEND}"' + body[match.end() :]
    return text[:start] + body + text[end:], True


def _add_dependency(text: str, requirement_text: str) -> tuple[str, bool, str | None]:
    project_table = tomllib.loads(text)["project"]
    if "dependencies" in (project_table.get("dynamic") or []):
        return (
            text,
            False,
            (
                f'[project] declares dynamic "dependencies"; add {requirement_text!r} through '
                "the mechanism that produces them"
            ),
        )
    start, end = _table_span(text, "[project]")  # type: ignore[misc]
    body = text[start:end]
    match = re.search(r"^dependencies\s*=\s*\[", body, re.MULTILINE)
    if match is None:
        insertion = f'\ndependencies = ["{requirement_text}"]\n'
        body = insertion + body
        return text[:start] + body + text[end:], True, None
    close = body.find("]", match.end())
    if close < 0:
        raise AddError("[project] dependencies array is not closed")
    existing = body[match.end() : close]
    entries = [entry.strip().strip("\"'") for entry in existing.split(",") if entry.strip()]
    if any(entry.split(">")[0].split("=")[0].strip() == "dimos" for entry in entries):
        return text, False, None
    if "\n" in existing:
        body = body[:close] + f'    "{requirement_text}",\n' + body[close:]
    else:
        separator = ", " if existing.strip() else ""
        body = body[:close] + f'{separator}"{requirement_text}"' + body[close:]
    return text[:start] + body + text[end:], True, None


def _message_sections(text: str, module: str) -> tuple[str, list[str]]:
    """Add the message source table and the provider entry point."""
    changes: list[str] = []
    if not text.endswith("\n"):
        text += "\n"
    if _table_span(text, "[tool.dimos.messages]") is None:
        text += (
            "\n[tool.dimos.messages]\n"
            f"# `.msg` definitions live under {SOURCE}/; every pip build regenerates the\n"
            "# Python packages below `build/message-source` from them.\n"
            f'source = "{SOURCE}"\n'
        )
        changes.append(f"declared [tool.dimos.messages] source = {SOURCE!r}")
    entry = f'{module} = "{module}_schemas.provider"\n'
    span = _table_span(text, f'[project.entry-points."{PROVIDER_GROUP}"]')
    if span is None:
        text += f'\n[project.entry-points."{PROVIDER_GROUP}"]\n{entry}'
        changes.append(f"registered the {module!r} message provider")
        return text, changes
    start, end = span
    if not re.search(rf"^{re.escape(module)}\s*=", text[start:end], re.MULTILINE):
        text = text[:end] + entry + text[end:]
        changes.append(f"registered the {module!r} message provider")
    return text, changes


def _extend_array(body: str, key: str, values: list[str]) -> tuple[str, list[str]]:
    """Add ``values`` to the ``key = [...]`` array inside a table body."""
    match = re.search(rf"^{re.escape(key)}[ \t]*=[ \t]*\[", body, re.MULTILINE)
    if match is None:
        return body, []
    close = body.find("]", match.end())
    if close < 0:
        raise AddError(f"{key} array is not closed")
    existing = body[match.end() : close]
    entries = [entry.strip().strip("\"'") for entry in existing.split(",") if entry.strip()]
    missing = [value for value in values if value not in entries]
    if not missing:
        return body, []
    addition = "".join(f'"{value}", ' for value in missing)
    if "\n" in existing:
        addition = "".join(f'    "{value}",\n' for value in missing)
        return body[:close] + addition + body[close:], missing
    separator = ", " if existing.strip() else ""
    return body[:close] + separator + addition.rstrip(", ") + body[close:], missing


def _package_data_sections(text: str, module: str) -> tuple[str, list[str]]:
    """Declare the generated schema data where setuptools reads package data.

    A project that already has ``[tool.setuptools.package-data]`` keeps its own
    entries: build-time ``package_data`` arguments do not override that table.
    """
    required = {
        module: ["py.typed", "*.pyi", "**/*.pyi"],
        module + "_schemas": ["schemas.json", "schemas/**/*", "package/**/*"],
    }
    changes: list[str] = []
    span = _table_span(text, "[tool.setuptools.package-data]")
    if span is None:
        if not text.endswith("\n"):
            text += "\n"
        lines = ["\n[tool.setuptools.package-data]"]
        for key, values in required.items():
            lines.append(f"{key} = [{', '.join(f'{value!r}' for value in values)}]")
        text += "\n".join(lines) + "\n"
        changes.append("declared the generated schema package data")
        return text, changes
    start, end = span
    body = text[start:end]
    added = False
    for key, values in required.items():
        body, missing = _extend_array(body, key, values)
        if missing:
            added = True
        elif not re.search(rf"^{re.escape(key)}[ \t]*=", body, re.MULTILINE):
            body = (
                body.rstrip("\n") + f"\n{key} = [{', '.join(f'{value!r}' for value in values)}]\n"
            )
            added = True
    if added:
        changes.append("extended [tool.setuptools.package-data] with the generated schemas")
    return text[:start] + body + text[end:], changes


def plan(project: Path, dimos_requirement: str | None = None) -> Addition:
    """Work out the minimal edit that turns ``project`` into a message package."""
    project = project.resolve()
    text, data = _load(project)
    project_table = _validate(data, project)
    module = default_module(project_table["name"])
    if _configured(data):
        return Addition(project, module, text, (), ("already configured; no files changed",))

    requirement_text = dimos_requirement or requirement()
    start_end = _table_span(text, "[build-system]")
    changes: list[str] = []
    if start_end is None:
        text = f'[build-system]\nrequires = ["setuptools>=70", "wheel", "{requirement_text}"]\n'
        text += f'build-backend = "{BACKEND}"\n\n' + text
        changes.append("added [build-system]")
    else:
        text, requires_changed = _set_requires(text, requirement_text)
        text, backend_changed = _set_backend(text)
        if requires_changed:
            changes.append(f"added {requirement_text} to [build-system] requires")
        if backend_changed:
            changes.append(f"set build-backend to {BACKEND}")

    text, dependency_changed, note = _add_dependency(text, requirement_text)
    notes: list[str] = []
    if dependency_changed:
        changes.append(f"added {requirement_text} to [project] dependencies")
    if note:
        notes.append(note)
    text, section_changes = _message_sections(text, module)
    changes.extend(section_changes)
    text, data_changes = _package_data_sections(text, module)
    changes.extend(data_changes)
    try:
        tomllib.loads(text)
    except tomllib.TOMLDecodeError as error:  # pragma: no cover - defensive
        raise AddError(f"internal error: generated pyproject.toml is invalid: {error}") from error
    return Addition(project, module, text, tuple(changes), tuple(notes))


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
