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
