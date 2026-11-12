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

Edits are derived from a structural scan of the document (tables, keys and value
spans, string/array/inline-table aware) rather than from line or comma based text
matching, so ordinary requirement strings such as ``"setuptools>=70,<81"`` or
``"requests[socks]>=2"`` keep their exact meaning. Before a plan is returned the
edited text is parsed again and compared with the parsed input; a mismatch is an
:class:`AddError` and nothing is written.
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


# --- structural scan ------------------------------------------------------


class _ScanError(Exception):
    """The document uses TOML this command cannot edit safely."""


@dataclass(frozen=True)
class _Array:
    open: int
    close: int
    items: tuple[tuple[int, int], ...]


@dataclass(frozen=True)
class _Statement:
    path: tuple[str, ...]
    value_start: int
    value_end: int
    array: _Array | None


@dataclass(frozen=True)
class _Table:
    path: tuple[str, ...]
    header_start: int
    body_start: int
    body_end: int
    statements: tuple[_Statement, ...]


def _skip_inline_ws(text: str, index: int) -> int:
    while index < len(text) and text[index] in " \t":
        index += 1
    return index


def _skip_comment(text: str, index: int) -> int:
    newline = text.find("\n", index)
    return len(text) if newline < 0 else newline


def _skip_blank(text: str, index: int) -> int:
    """Skip whitespace and comments, including newlines."""
    while index < len(text):
        char = text[index]
        if char in " \t\r\n":
            index += 1
        elif char == "#":
            index = _skip_comment(text, index)
        else:
            return index
    return index


def _scan_string(text: str, index: int) -> int:
    """Return the index just past the string literal starting at ``index``."""
    quote = text[index]
    if text.startswith(quote * 3, index):
        cursor = index + 3
        while cursor < len(text):
            if text[cursor] == "\\" and quote == '"':
                cursor += 2
                continue
            if text.startswith(quote * 3, cursor):
                return cursor + 3
            cursor += 1
        raise _ScanError("unterminated multi-line string")
    cursor = index + 1
    while cursor < len(text):
        char = text[cursor]
        if char == "\n":
            break
        if char == "\\" and quote == '"':
            cursor += 2
            continue
        if char == quote:
            return cursor + 1
        cursor += 1
    raise _ScanError("unterminated string")


def _scan_array(text: str, index: int) -> tuple[_Array, int]:
    items: list[tuple[int, int]] = []
    cursor = index + 1
    while True:
        cursor = _skip_blank(text, cursor)
        if cursor >= len(text):
            raise _ScanError("unterminated array")
        if text[cursor] == "]":
            return _Array(index, cursor, tuple(items)), cursor + 1
        if text[cursor] == ",":
            raise _ScanError(f"missing array element (offset {cursor})")
        end, _ = _scan_value(text, cursor)
        items.append((cursor, end))
        cursor = _skip_blank(text, end)
        if cursor < len(text) and text[cursor] == ",":
            cursor += 1
            continue
        if cursor < len(text) and text[cursor] == "]":
            return _Array(index, cursor, tuple(items)), cursor + 1
        raise _ScanError(f"expected ',' or ']' in an array (offset {cursor})")


def _scan_inline_table(text: str, index: int) -> int:
    cursor = index + 1
    while True:
        cursor = _skip_blank(text, cursor)
        if cursor >= len(text):
            raise _ScanError("unterminated inline table")
        if text[cursor] == "}":
            return cursor + 1
        _, cursor = _scan_key(text, cursor)
        cursor = _skip_inline_ws(text, cursor)
        if cursor >= len(text) or text[cursor] != "=":
            raise _ScanError(f"expected '=' in an inline table (offset {cursor})")
        cursor, _ = _scan_value(text, cursor + 1)
        cursor = _skip_blank(text, cursor)
        if cursor < len(text) and text[cursor] == ",":
            cursor += 1
            continue
        if cursor < len(text) and text[cursor] == "}":
            return cursor + 1
        raise _ScanError(f"expected ',' or '}}' in an inline table (offset {cursor})")


def _scan_value(text: str, index: int) -> tuple[int, _Array | None]:
    """Return the end of the value at ``index`` and its array span, if any."""
    index = _skip_inline_ws(text, index)
    if index >= len(text):
        raise _ScanError("key without a value")
    char = text[index]
    if char in "\"'":
        return _scan_string(text, index), None
    if char == "[":
        array, end = _scan_array(text, index)
        return end, array
    if char == "{":
        return _scan_inline_table(text, index), None
    cursor = index
    while cursor < len(text) and text[cursor] not in "\r\n#":
        cursor += 1
    while cursor > index and text[cursor - 1] in " \t":
        cursor -= 1
    if cursor == index:
        raise _ScanError("empty value")
    return cursor, None


def _literal_value(raw: str) -> str:
    """Decode one TOML string literal (used for quoted keys)."""
    try:
        value = tomllib.loads(f"value = {raw}")["value"]
    except tomllib.TOMLDecodeError as error:  # pragma: no cover - defensive
        raise _ScanError(f"cannot read {raw!r}: {error}") from error
    if not isinstance(value, str):  # pragma: no cover - defensive
        raise _ScanError(f"{raw!r} is not a string")
    return value


def _scan_key(text: str, index: int) -> tuple[tuple[str, ...], int]:
    """Return the dotted key path starting at ``index`` and the index after it."""
    parts: list[str] = []
    while True:
        index = _skip_inline_ws(text, index)
        if index >= len(text):
            raise _ScanError("dangling key")
        if text[index] in "\"'":
            end = _scan_string(text, index)
            parts.append(_literal_value(text[index:end]))
            index = end
        else:
            start = index
            while index < len(text) and text[index] not in ".= \t\r\n":
                index += 1
            if start == index:
                raise _ScanError(f"expected a key (offset {index})")
            parts.append(text[start:index])
        index = _skip_inline_ws(text, index)
        if index < len(text) and text[index] == ".":
            index += 1
            continue
        return tuple(parts), index


def _end_of_line(text: str, index: int) -> int:
    """Require only an optional comment after ``index``; return the next line start."""
    index = _skip_inline_ws(text, index)
    if index < len(text) and text[index] == "#":
        index = _skip_comment(text, index)
    if index < len(text) and text[index] not in "\r\n":
        raise _ScanError(f"unexpected text (offset {index})")
    if index < len(text) and text[index] == "\r":
        index += 1
    if index < len(text) and text[index] == "\n":
        index += 1
    return index


def _scan_document(text: str) -> tuple[_Table, ...]:
    """Split the document into tables and their key/value statements."""
    tables: list[_Table] = []
    path: tuple[str, ...] = ()
    header_start = 0
    body_start = 0
    statements: list[_Statement] = []
    cursor = 0
    while True:
        cursor = _skip_blank(text, cursor)
        if cursor >= len(text):
            break
        if text[cursor] == "[":
            tables.append(_Table(path, header_start, body_start, cursor, tuple(statements)))
            array_table = text.startswith("[[", cursor)
            start = cursor + (2 if array_table else 1)
            scan = start
            while True:
                if scan >= len(text) or text[scan] in "\r\n":
                    raise _ScanError("unterminated table header")
                if text[scan] in "\"'":
                    scan = _scan_string(text, scan)
                    continue
                if text[scan] == "]":
                    if array_table and not text.startswith("]]", scan):
                        raise _ScanError("malformed array-of-tables header")
                    break
                scan += 1
            parts, end = _scan_key(text[start:scan], 0)
            if _skip_inline_ws(text[start:scan], end) != scan - start:
                raise _ScanError("unexpected text in a table header")
            path = tuple(parts)
            header_end = scan + (2 if array_table else 1)
            header_start = text.rfind("\n", 0, cursor) + 1
            body_start = _end_of_line(text, header_end)
            statements = []
            cursor = body_start
            continue
        parts, after_key = _scan_key(text, cursor)
        cursor = _skip_inline_ws(text, after_key)
        if cursor >= len(text) or text[cursor] != "=":
            raise _ScanError(f"expected '=' after a key (offset {cursor})")
        value_start = _skip_inline_ws(text, cursor + 1)
        value_end, array = _scan_value(text, value_start)
        statements.append(_Statement(tuple(parts), value_start, value_end, array))
        cursor = _end_of_line(text, value_end)
    tables.append(_Table(path, header_start, body_start, len(text), tuple(statements)))
    return tuple(tables)


def _scan(text: str, path: Path) -> tuple[_Table, ...]:
    try:
        return _scan_document(text)
    except _ScanError as error:
        raise AddError(
            f"{path} uses TOML this command cannot edit safely ({error}); "
            "add the message packaging configuration by hand"
        ) from error


def _find(tables: tuple[_Table, ...], path: tuple[str, ...]) -> _Statement | None:
    for table in tables:
        for statement in table.statements:
            if table.path + statement.path == path:
                return statement
    return None


def _table(tables: tuple[_Table, ...], path: tuple[str, ...]) -> _Table | None:
    for table in tables:
        if table.path == path:
            return table
    return None


# --- edits ----------------------------------------------------------------


@dataclass(frozen=True)
class _Edit:
    start: int
    end: int
    replacement: str


def _apply_edits(text: str, edits: list[_Edit]) -> str:
    for edit in sorted(edits, key=lambda item: (item.start, item.end), reverse=True):
        if not 0 <= edit.start <= edit.end <= len(text):
            raise AddError("internal error: planned edit is outside pyproject.toml")
        text = text[: edit.start] + edit.replacement + text[edit.end :]
    return text


def _string(value: str) -> str:
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def _array_values(text: str, statement: _Statement) -> list[str]:
    """The string values of an array statement, refusing anything else."""
    array = statement.array
    if array is None:
        raise AddError(
            f"{'.'.join(statement.path)} must be an array of requirement strings; "
            "edit pyproject.toml by hand"
        )
    values: list[str] = []
    for start, end in array.items:
        try:
            value = tomllib.loads(f"value = {text[start:end]}")["value"]
        except tomllib.TOMLDecodeError as error:  # pragma: no cover - defensive
            raise AddError(f"cannot read an array entry: {error}") from error
        if not isinstance(value, str):
            raise AddError(
                f"{'.'.join(statement.path)} contains a non-string entry; "
                "edit pyproject.toml by hand"
            )
        values.append(value)
    return values


def _append_array_item(text: str, statement: _Statement, literal: str) -> list[_Edit]:
    """Place ``literal`` after the last element, leaving existing text intact."""
    array = statement.array
    if array is None:  # pragma: no cover - callers check this first
        raise AddError(f"{'.'.join(statement.path)} is not an array")
    close = array.close
    if not array.items:
        return [_Edit(close, close, literal)]
    last_end = array.items[-1][1]
    trailing_comma = False
    cursor = last_end
    while cursor < close:
        char = text[cursor]
        if char in " \t\r\n":
            cursor += 1
        elif char == "#":
            cursor = _skip_comment(text, cursor)
        elif char == ",":
            trailing_comma = True
            cursor += 1
        else:  # pragma: no cover - the scanner guarantees this cannot happen
            raise AddError("internal error: unexpected text inside an array")
    close_line_start = text.rfind("\n", 0, close) + 1
    if "\n" in text[array.open : close] and not text[close_line_start:close].strip():
        item_line_start = text.rfind("\n", 0, array.items[-1][0]) + 1
        indent = text[item_line_start : array.items[-1][0]]
        if indent.strip():
            indent = "    "
        edits = [_Edit(close_line_start, close_line_start, f"{indent}{literal},\n")]
        if not trailing_comma:
            edits.append(_Edit(last_end, last_end, ","))
        return edits
    separator = "" if text[close - 1] in " \t" else " "
    comma = "" if trailing_comma else ","
    return [_Edit(close, close, comma + separator + literal)]


def _insert_line(text: str, position: int, line: str) -> _Edit:
    """Insert a statement line at ``position``, keeping lines separated."""
    if position > 0 and text[position - 1] not in "\r\n":
        line = "\n" + line
    return _Edit(position, position, line)


# --- validation -----------------------------------------------------------


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


def _has_dimos(values: list[str]) -> bool:
    return any(requirement_name(value) == "dimos" for value in values)


def _unchanged(before: dict, after: dict, path: tuple[str, ...] = ()) -> None:
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


def _verify(before: dict, text: str, requirement_text: str, module: str) -> None:
    """Re-parse the planned document and prove it says exactly what we meant."""
    try:
        after = tomllib.loads(text)
    except tomllib.TOMLDecodeError as error:
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
    settings = after.get("tool", {}).get("dimos", {}).get("messages")
    if settings != {"source": SOURCE}:
        raise AddError("internal error: [tool.dimos.messages] is incomplete; not written")
    entry_points = after.get("project", {}).get("entry-points", {}).get(PROVIDER_GROUP)
    if not isinstance(entry_points, dict) or entry_points.get(module) != (
        f"{module}_schemas.provider"
    ):
        raise AddError("internal error: the message provider entry point is missing; not written")


# --- plan -----------------------------------------------------------------


def _package_data_changes(
    text: str, tables: tuple[_Table, ...], module: str
) -> tuple[list[_Edit], list[str], list[str]]:
    """Declare the generated schema data where setuptools reads package data.

    A project that already has ``[tool.setuptools.package-data]`` keeps its own
    entries: build-time ``package_data`` arguments do not override that table.
    """
    required = {
        module: ("py.typed", "*.pyi", "**/*.pyi"),
        module + "_schemas": ("schemas.json", "schemas/**/*", "package/**/*"),
    }
    edits: list[_Edit] = []
    changes: list[str] = []
    blocks: list[str] = []
    table = _table(tables, ("tool", "setuptools", "package-data"))
    if table is None:
        lines = ["[tool.setuptools.package-data]"]
        for key, values in required.items():
            lines.append(f"{_string(key)} = [{', '.join(_string(value) for value in values)}]")
        blocks.append("\n".join(lines) + "\n")
        changes.append("declared the generated schema package data")
        return edits, blocks, changes
    added = False
    missing_lines: list[str] = []
    for key, values in required.items():
        statement = _find(tables, ("tool", "setuptools", "package-data", key))
        if statement is None:
            missing_lines.append(
                f"{_string(key)} = [{', '.join(_string(value) for value in values)}]\n"
            )
            added = True
            continue
        existing = _array_values(text, statement)
        missing = [value for value in values if value not in existing]
        if missing:
            edits.extend(
                _append_array_item(text, statement, ", ".join(_string(value) for value in missing))
            )
            added = True
    if missing_lines:
        edits.append(_insert_line(text, table.body_end, "".join(missing_lines)))
    if added:
        changes.append("extended [tool.setuptools.package-data] with the generated schemas")
    return edits, blocks, changes


def _message_sections(
    text: str, tables: tuple[_Table, ...], module: str
) -> tuple[list[_Edit], list[str], list[str]]:
    """Add the message source table and the provider entry point."""
    edits: list[_Edit] = []
    changes: list[str] = []
    blocks: list[str] = []
    if _table(tables, ("tool", "dimos", "messages")) is None:
        blocks.append(
            "[tool.dimos.messages]\n"
            f"# `.msg` definitions live under {SOURCE}/; every pip build regenerates the\n"
            "# Python packages below `build/message-source` from them.\n"
            f'source = "{SOURCE}"\n'
        )
        changes.append(f"declared [tool.dimos.messages] source = {SOURCE!r}")
    entry = f'{module} = "{module}_schemas.provider"\n'
    existing_entry = _find(tables, ("project", "entry-points", PROVIDER_GROUP, module))
    if existing_entry is not None:
        return edits, blocks, changes
    table = _table(tables, ("project", "entry-points", PROVIDER_GROUP))
    if table is None:
        blocks.append(f'[project.entry-points."{PROVIDER_GROUP}"]\n{entry}')
    else:
        edits.append(_insert_line(text, table.body_end, entry))
    changes.append(f"registered the {module!r} message provider")
    return edits, blocks, changes


def plan(project: Path, dimos_requirement: str | None = None) -> Addition:
    """Work out the minimal edit that turns ``project`` into a message package."""
    project = project.resolve()
    text, data = _load(project)
    project_table = _validate(data, project)
    module = default_module(project_table["name"])
    if _configured(data):
        return Addition(project, module, text, (), ("already configured; no files changed",))

    requirement_text = dimos_requirement or requirement()
    if requirement_name(requirement_text) != "dimos":
        raise AddError(f"{requirement_text!r} is not a dimos requirement")
    tables = _scan(text, project / "pyproject.toml")
    edits: list[_Edit] = []
    blocks: list[str] = []
    changes: list[str] = []

    build_system = data.get("build-system")
    if build_system is None:
        edits.append(
            _Edit(
                0,
                0,
                "[build-system]\n"
                f"requires = [{', '.join(_string(value) for value in (*DEFAULT_REQUIRES, requirement_text))}]\n"
                f'build-backend = "{BACKEND}"\n\n',
            )
        )
        changes.append("added [build-system]")
    else:
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
            statement = _find(tables, ("build-system", "requires"))
            if statement is None:
                raise AddError(
                    "[build-system] has no requires array to extend; add the dimos requirement "
                    "by hand"
                )
            edits.extend(_append_array_item(text, statement, _string(requirement_text)))
            changes.append(f"added {requirement_text} to [build-system] requires")
        if build_system.get("build-backend", SETUPTOOLS_BACKEND) != BACKEND:
            statement = _find(tables, ("build-system", "build-backend"))
            if statement is None:
                header = _table(tables, ("build-system",))
                if header is None:
                    raise AddError(
                        "[build-system] is declared with dotted keys; add "
                        f'build-backend = "{BACKEND}" by hand'
                    )
                edits.append(
                    _insert_line(text, header.body_start, f'build-backend = "{BACKEND}"\n')
                )
            else:
                edits.append(_Edit(statement.value_start, statement.value_end, _string(BACKEND)))
            changes.append(f"set build-backend to {BACKEND}")

    notes: list[str] = []
    dynamic = project_table.get("dynamic") or []
    if "dependencies" in dynamic:
        notes.append(
            f'[project] declares dynamic "dependencies"; add {requirement_text!r} through '
            "the mechanism that produces them"
        )
    else:
        dependencies = project_table.get("dependencies")
        if dependencies is None:
            header = _table(tables, ("project",))
            if header is None:
                raise AddError(
                    "[project] is declared with dotted keys; add a dependencies array by hand"
                )
            edits.append(
                _insert_line(
                    text, header.body_start, f"dependencies = [{_string(requirement_text)}]\n"
                )
            )
            changes.append(f"added {requirement_text} to [project] dependencies")
        elif not isinstance(dependencies, list):
            raise AddError(
                "[project] dependencies must be an array of requirement strings; "
                "edit pyproject.toml by hand"
            )
        elif not all(isinstance(value, str) for value in dependencies):
            raise AddError(
                "[project] dependencies contains a non-string entry; edit pyproject.toml by hand"
            )
        elif not _has_dimos(dependencies):
            statement = _find(tables, ("project", "dependencies"))
            if statement is None:  # pragma: no cover - dotted keys are found by the scan
                raise AddError(
                    "[project] dependencies could not be located; edit pyproject.toml by hand"
                )
            edits.extend(_append_array_item(text, statement, _string(requirement_text)))
            changes.append(f"added {requirement_text} to [project] dependencies")

    section_edits, section_blocks, section_changes = _message_sections(text, tables, module)
    edits.extend(section_edits)
    blocks.extend(section_blocks)
    changes.extend(section_changes)
    data_edits, data_blocks, data_changes = _package_data_changes(text, tables, module)
    edits.extend(data_edits)
    blocks.extend(data_blocks)
    changes.extend(data_changes)

    if any(edit.start > len(text) for edit in edits):
        raise AddError("internal error: planned edit is outside pyproject.toml")

    new_text = _apply_edits(text, edits)
    if blocks:
        new_text = new_text.rstrip("\n") + "\n" + "".join("\n" + block for block in blocks)
    _verify(data, new_text, requirement_text, module)
    return Addition(project, module, new_text, tuple(changes), tuple(notes))


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
