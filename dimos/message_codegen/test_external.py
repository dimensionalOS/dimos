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

"""`dimos package add messages`: minimal, idempotent, refusing unsafe projects."""

from pathlib import Path
import subprocess
import sys

import pytest

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from dimos.message_codegen import backend
from dimos.message_codegen.external import AddError, apply, default_module, plan

PYPROJECT = """\
[build-system]
requires = ["setuptools>=70", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "acme-telemetry"
version = "0.1.0"
description = "Acme robot telemetry"  # keep this comment
dependencies = [
    "requests>=2.32",  # existing runtime dependency, keep it
]

[tool.setuptools.package-data]
acme_telemetry = ["data/*.json"]
"""


def write_project(tmp_path: Path, text: str = PYPROJECT, name: str = "acme-telemetry") -> Path:
    project = tmp_path / name
    project.mkdir()
    (project / "pyproject.toml").write_text(text)
    return project


def snapshot(project: Path) -> dict[Path, bytes]:
    return {path: path.read_bytes() for path in sorted(project.rglob("*")) if path.is_file()}


def test_add_is_minimal_and_preserves_project_configuration(tmp_path: Path) -> None:
    project = write_project(tmp_path)

    addition = plan(project, dimos_requirement="dimos>=0.0.15")
    assert addition.changed
    assert addition.module == "acme_telemetry"
    written = apply(addition)

    assert written == ["interfaces/", "pyproject.toml"]
    assert (project / "interfaces").is_dir()
    text = (project / "pyproject.toml").read_text()
    assert "# keep this comment" in text
    assert '"requests>=2.32",  # existing runtime dependency, keep it' in text
    assert 'build-backend = "dimos.message_codegen.backend"' in text
    assert '"dimos>=0.0.15",' in text
    assert "[tool.dimos.messages]" in text
    assert 'source = "interfaces"' in text
    assert '[project.entry-points."dimos.messages"]' in text
    assert 'acme_telemetry = "acme_telemetry_schemas.provider"' in text
    assert 'acme_telemetry = ["data/*.json", "py.typed", "*.pyi", "**/*.pyi"]' in text
    assert "python-module" not in text
    assert "setup.py" not in written


def test_second_add_is_a_no_op(tmp_path: Path) -> None:
    project = write_project(tmp_path)
    apply(plan(project, dimos_requirement="dimos>=0.0.15"))
    before = (project / "pyproject.toml").read_text()
    user = project / "interfaces/demo_msgs/msg"
    user.mkdir(parents=True)
    (user / "Reading.msg").write_text("std_msgs/Header header\nfloat64 value\n")

    addition = plan(project, dimos_requirement="dimos>=0.0.15")
    assert not addition.changed
    assert apply(addition) == []
    assert (project / "pyproject.toml").read_text() == before
    assert (user / "Reading.msg").is_file()


@pytest.mark.parametrize(
    ("text", "message"),
    [
        (PYPROJECT.replace('version = "0.1.0"', 'version = "0.1"'), "not a X.Y.Z"),
        (
            PYPROJECT.replace(
                'build-backend = "setuptools.build_meta"', 'build-backend = "hatchling.build"'
            ),
            "never replaces another build backend",
        ),
        (
            PYPROJECT + '\n[tool.setuptools]\npackages = ["acme_telemetry"]\n',
            "already declares packages",
        ),
    ],
)
def test_unsupported_projects_are_refused_without_changes(
    tmp_path: Path, text: str, message: str
) -> None:
    project = write_project(tmp_path, text)
    before = snapshot(project)

    with pytest.raises(AddError, match=message):
        plan(project, dimos_requirement="dimos>=0.0.15")

    assert snapshot(project) == before


def test_legacy_setup_py_project_is_refused(tmp_path: Path) -> None:
    project = write_project(tmp_path)
    (project / "setup.py").write_text("from setuptools import setup\nsetup()\n")

    with pytest.raises(AddError, match="setup.py exists"):
        plan(project, dimos_requirement="dimos>=0.0.15")


def test_default_module_follows_the_distribution_name() -> None:
    assert default_module("acme-telemetry") == "acme_telemetry"
    with pytest.raises(AddError, match="does not map to a Python package name"):
        default_module("acme telemetry")


def test_existing_provider_entry_point_is_extended_not_replaced(tmp_path: Path) -> None:
    text = PYPROJECT + (
        '\n[project.entry-points."dimos.messages"]\nother = "other_schemas.provider"\n'
    )
    project = write_project(tmp_path, text)

    apply(plan(project, dimos_requirement="dimos>=0.0.15"))

    provider = (
        (project / "pyproject.toml").read_text().split('[project.entry-points."dimos.messages"]')[1]
    )
    assert 'other = "other_schemas.provider"' in provider
    assert 'acme_telemetry = "acme_telemetry_schemas.provider"' in provider


def test_package_add_does_not_import_the_build_backend(tmp_path: Path) -> None:
    script = (
        "import sys, dimos.cli.commands.package; "
        "assert 'dimos.message_codegen.backend' not in sys.modules"
    )
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, cwd=tmp_path
    )
    assert result.returncode == 0, result.stderr


def _generated(project: Path, module: str) -> tuple[Path, dict]:
    python_root = project / "build/message-source/python"
    directory = python_root / module
    directory.mkdir(parents=True)
    (directory / "__init__.py").write_text("__dimos_version__ = '0.1.0'\nfrom . import demo_msgs\n")
    (directory / "_types.py").write_text("store = object()\n")
    return python_root, {"python_module": module}


def test_overlay_merges_project_files_and_keeps_init_body_first(tmp_path: Path) -> None:
    project = tmp_path / "project"
    user = project / "acme_telemetry"
    user.mkdir(parents=True)
    (user / "__init__.py").write_text("from __future__ import annotations\nVERSION = '1.0'\n")
    (user / "app.py").write_text("def config(): return 'acme'\n")
    python_root, config = _generated(project, "acme_telemetry")

    backend._overlay(python_root, project, config)

    merged = (python_root / "acme_telemetry/__init__.py").read_text()
    compile(merged, "__init__.py", "exec")
    assert merged.startswith("from __future__ import annotations")
    assert "VERSION = '1.0'" in merged
    assert "__dimos_version__" in merged
    assert (python_root / "acme_telemetry/app.py").resolve() == (user / "app.py").resolve()
    assert (python_root / "acme_telemetry/_types.py").is_file()


def test_overlay_refuses_to_drop_a_project_file(tmp_path: Path) -> None:
    project = tmp_path / "project"
    user = project / "acme_telemetry"
    user.mkdir(parents=True)
    (user / "_types.py").write_text("raise AssertionError('ours')\n")
    python_root, config = _generated(project, "acme_telemetry")

    with pytest.raises(backend.MessagePackageError, match="collides with the generated"):
        backend._overlay(python_root, project, config)

    assert (user / "_types.py").read_text() == "raise AssertionError('ours')\n"


def test_overlay_accepts_identical_files(tmp_path: Path) -> None:
    project = tmp_path / "project"
    user = project / "acme_telemetry"
    user.mkdir(parents=True)
    (user / "py.typed").write_text("")
    python_root, config = _generated(project, "acme_telemetry")
    (python_root / "acme_telemetry/py.typed").write_text("")

    backend._overlay(python_root, project, config)

    assert (python_root / "acme_telemetry/py.typed").is_file()


COMMA_REQUIREMENT = """\
[build-system]
requires = ["setuptools>=70,<81", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "acme-telemetry"
version = "0.1.0"
"""

EXTRAS_DEPENDENCY = """\
[build-system]
requires = ["setuptools>=70", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "acme-telemetry"
version = "0.1.0"
dependencies = ["requests[socks]>=2"]
"""

MULTILINE_COMMENTED = """\
[build-system]
requires = [
    # Pinned build toolchain, kept as one specifier set.
    "setuptools>=70,<81",
    "wheel",  # needed for bdist_wheel
]
build-backend = "setuptools.build_meta"

[project]
name = "acme-telemetry"
version = "0.1.0"
dependencies = [
    "requests[socks]>=2",  # proxy support
    "numpy>=1.26",
]
"""

PROJECT_ONLY = """\
[project]
name = "acme-telemetry"
version = "0.1.0"
"""

# The reviewer's fixture: valid unrelated mypy configuration that a comma- or
# bracket-based pyproject edit corrupted or refused. Kept verbatim.
MYPY_OVERRIDES = """\
[build-system]
requires = ["setuptools>=70", "wheel"]
build-backend = "setuptools.build_meta"
[project]
name = "probe"
version = "0.1.0"

[tool.mypy]
overrides = [{module = "optional_sdk.*", ignore_missing_imports = true}]
"""

NESTED_SCALARS = """\
[build-system]
requires = ["setuptools>=70", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "acme-telemetry"
version = "0.1.0"

[tool.mypy]
strict = true
overrides = [
    {module = "optional_sdk.*", ignore_missing_imports = true},
    {module = "legacy.*", ignore_errors = false},
]

[tool.coverage.run]
branch = true
parallel = false

[tool.pytest.ini_options]
markers = ["slow: 30", "flaky: 1.5e-3"]

[tool.deploy]
ports = [8080, 9090.5, -12, true, false]
nested = [[1, 2], [true, false]]
"""

UNRELATED_TOOL_TABLES = ("mypy", "coverage", "pytest", "deploy")
"""Unrelated configuration that must survive ``package add messages`` untouched."""


@pytest.mark.parametrize(
    ("text", "requires", "dependencies"),
    [
        (COMMA_REQUIREMENT, ["setuptools>=70,<81", "wheel", "dimos>=0.0.15"], ["dimos>=0.0.15"]),
        (
            EXTRAS_DEPENDENCY,
            ["setuptools>=70", "wheel", "dimos>=0.0.15"],
            ["requests[socks]>=2", "dimos>=0.0.15"],
        ),
        (
            MULTILINE_COMMENTED,
            ["setuptools>=70,<81", "wheel", "dimos>=0.0.15"],
            ["requests[socks]>=2", "numpy>=1.26", "dimos>=0.0.15"],
        ),
        (PROJECT_ONLY, ["setuptools>=70", "wheel", "dimos>=0.0.15"], ["dimos>=0.0.15"]),
    ],
    ids=["comma-in-requirement", "extras", "multiline-comments", "no-build-system"],
)
def test_supported_pyproject_inputs_keep_requirement_values(
    tmp_path: Path, text: str, requires: list[str], dependencies: list[str]
) -> None:
    """Ordinary PEP 508 strings, comments and missing [build-system] are supported."""
    project = write_project(tmp_path, text)
    before = tomllib.loads(text)

    apply(plan(project, dimos_requirement="dimos>=0.0.15"))

    after = tomllib.loads((project / "pyproject.toml").read_text())
    assert after["build-system"]["requires"] == requires
    assert after["build-system"]["build-backend"] == "dimos.message_codegen.backend"
    assert after["project"]["dependencies"] == dependencies
    assert after["project"]["name"] == before["project"]["name"]
    assert after["project"]["version"] == before["project"]["version"]
    assert after["tool"]["dimos"]["messages"] == {"source": "interfaces"}
    assert after["project"]["entry-points"]["dimos.messages"] == {
        "acme_telemetry": "acme_telemetry_schemas.provider"
    }
    assert (project / "pyproject.toml").read_text().count("[build-system]") == 1

    again = plan(project, dimos_requirement="dimos>=0.0.15")
    assert not again.changed
    assert apply(again) == []


def test_multiline_arrays_keep_their_comments(tmp_path: Path) -> None:
    project = write_project(tmp_path, MULTILINE_COMMENTED)

    apply(plan(project, dimos_requirement="dimos>=0.0.15"))

    text = (project / "pyproject.toml").read_text()
    assert "# Pinned build toolchain, kept as one specifier set." in text
    assert '"wheel",  # needed for bdist_wheel' in text
    assert '"requests[socks]>=2",  # proxy support' in text
    assert '    "dimos>=0.0.15",\n]' in text


def test_reviewed_mypy_overrides_fixture_is_preserved(tmp_path: Path) -> None:
    """The exact regression fixture: unrelated boolean inline table in [tool.mypy]."""
    project = write_project(tmp_path, MYPY_OVERRIDES, name="probe")

    addition = plan(project, dimos_requirement="dimos>=0.0.15")
    assert addition.changed
    assert 'overrides = [{module = "optional_sdk.*", ignore_missing_imports = true}]' in (
        addition.text
    )
    apply(addition)

    text = (project / "pyproject.toml").read_text()
    after = tomllib.loads(text)
    assert after["tool"]["mypy"] == tomllib.loads(MYPY_OVERRIDES)["tool"]["mypy"]
    assert after["build-system"]["requires"] == ["setuptools>=70", "wheel", "dimos>=0.0.15"]
    assert after["build-system"]["build-backend"] == "dimos.message_codegen.backend"
    assert after["project"]["dependencies"] == ["dimos>=0.0.15"]
    assert after["project"]["name"] == "probe"

    again = plan(project, dimos_requirement="dimos>=0.0.15")
    assert not again.changed
    assert apply(again) == []
    assert (project / "pyproject.toml").read_text() == text


def test_unrelated_boolean_and_numeric_configuration_is_preserved(tmp_path: Path) -> None:
    """Bare booleans, integers, floats and nested arrays outside our keys survive."""
    project = write_project(tmp_path, NESTED_SCALARS)
    before = tomllib.loads(NESTED_SCALARS)

    apply(plan(project, dimos_requirement="dimos>=0.0.15"))

    text = (project / "pyproject.toml").read_text()
    after = tomllib.loads(text)
    for table in UNRELATED_TOOL_TABLES:
        assert after["tool"][table] == before["tool"][table]
    assert 'overrides = [\n    {module = "optional_sdk.*", ignore_missing_imports = true},' in text
    assert "ports = [8080, 9090.5, -12, true, false]" in text
    assert "nested = [[1, 2], [true, false]]" in text
    assert after["project"]["dependencies"] == ["dimos>=0.0.15"]

    again = plan(project, dimos_requirement="dimos>=0.0.15")
    assert not again.changed
    assert apply(again) == []
    assert (project / "pyproject.toml").read_text() == text


def test_unrelated_configuration_survives_the_cli(tmp_path: Path) -> None:
    """The command users run accepts the reviewed fixture and is byte-idempotent."""
    from typer.testing import CliRunner

    from dimos.cli.dimos import main

    project = write_project(tmp_path, MYPY_OVERRIDES, name="probe")
    runner = CliRunner()

    first = runner.invoke(main, ["package", "add", "messages", "-C", str(project)])
    assert first.exit_code == 0, first.output
    text = (project / "pyproject.toml").read_text()
    assert 'overrides = [{module = "optional_sdk.*", ignore_missing_imports = true}]' in text
    assert tomllib.loads(text)["tool"]["mypy"] == {
        "overrides": [{"module": "optional_sdk.*", "ignore_missing_imports": True}]
    }

    second = runner.invoke(main, ["package", "add", "messages", "-C", str(project)])
    assert second.exit_code == 0, second.output
    assert "already set up" in second.output
    assert (project / "pyproject.toml").read_text() == text


@pytest.mark.parametrize(
    ("text", "message"),
    [
        (
            COMMA_REQUIREMENT.replace(
                'requires = ["setuptools>=70,<81", "wheel"]', 'requires = "setuptools>=70"'
            ),
            r"\[build-system\] requires must be an array",
        ),
        (
            COMMA_REQUIREMENT.replace(
                'version = "0.1.0"', 'version = "0.1.0"\ndependencies = "requests>=2"'
            ),
            r"\[project\] dependencies must be an array",
        ),
        (
            COMMA_REQUIREMENT.replace(
                'requires = ["setuptools>=70,<81", "wheel"]',
                'requires = ["setuptools>=70", {name = "wheel"}]',
            ),
            "non-string entry",
        ),
        (
            COMMA_REQUIREMENT.replace(
                'requires = ["setuptools>=70,<81", "wheel"]', 'requires = ["setuptools>=70"'
            ),
            "not valid TOML",
        ),
        (
            'project.name = "acme-telemetry"\nproject.version = "0.1.0"\n',
            "dotted keys",
        ),
    ],
    ids=[
        "requires-not-array",
        "dependencies-not-array",
        "non-string-entry",
        "unclosed-array",
        "dotted-project",
    ],
)
def test_unsupported_syntax_is_refused_before_writing(
    tmp_path: Path, text: str, message: str
) -> None:
    project = write_project(tmp_path, text)
    before = snapshot(project)

    with pytest.raises(AddError, match=message):
        plan(project, dimos_requirement="dimos>=0.0.15")

    assert snapshot(project) == before


def test_cli_add_messages_extends_reviewed_inputs(tmp_path: Path) -> None:
    """The command users run exercises the same edits end to end."""
    from typer.testing import CliRunner

    from dimos.cli.dimos import main

    project = write_project(tmp_path, COMMA_REQUIREMENT)
    runner = CliRunner()

    first = runner.invoke(main, ["package", "add", "messages", "-C", str(project)])
    assert first.exit_code == 0, first.output
    after = tomllib.loads((project / "pyproject.toml").read_text())
    requires = after["build-system"]["requires"]
    assert requires[:2] == ["setuptools>=70,<81", "wheel"]
    assert len(requires) == 3 and requires[2].startswith("dimos>=")

    written = (project / "pyproject.toml").read_text()
    second = runner.invoke(main, ["package", "add", "messages", "-C", str(project)])
    assert second.exit_code == 0, second.output
    assert "already set up" in second.output
    assert (project / "pyproject.toml").read_text() == written


def test_cli_refuses_unsupported_input_without_writing(tmp_path: Path) -> None:
    from typer.testing import CliRunner

    from dimos.cli.dimos import main

    text = COMMA_REQUIREMENT.replace(
        'version = "0.1.0"', 'version = "0.1.0"\ndependencies = "requests>=2"'
    )
    project = write_project(tmp_path, text)

    result = CliRunner().invoke(main, ["package", "add", "messages", "-C", str(project)])

    assert result.exit_code == 1
    assert "must be an array" in result.output
    assert (project / "pyproject.toml").read_text() == text
    assert not (project / "interfaces").exists()
