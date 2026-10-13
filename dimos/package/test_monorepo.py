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

"""Generic Git/workspace/backend acceptance; no native SDK builds or CI matrix."""

from importlib.metadata import distribution as installed_distribution
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from urllib.parse import unquote
from zipfile import ZipFile

import pytest

from dimos.package.project import add, consumer_project, remove, sync, update
from dimos.package.sources import resolve_source, workspace_member


@pytest.fixture
def package_repository(tmp_path, monkeypatch):
    for tool in ("git", "uv"):
        if shutil.which(tool) is None:
            pytest.skip(f"consumer acceptance requires {tool}")
    for module in ("build", "scikit_build_core", "packaging", "pathspec"):
        if importlib.util.find_spec(module) is None:
            pytest.skip(f"consumer acceptance requires {module}")
    root = Path(__file__).resolve().parents[2]
    wheelhouse = tmp_path / "wheels"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "build",
            "--no-isolation",
            "--wheel",
            str(root / "packages/dimos-build-config"),
            "--outdir",
            str(wheelhouse),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    # Seed pure-Python build tools from the test environment into our own wheelhouse.
    for name in ("scikit-build-core", "packaging", "pathspec"):
        installed = installed_distribution(name)
        wheel = wheelhouse / f"{name.replace('-', '_')}-{installed.version}-py3-none-any.whl"
        with ZipFile(wheel, "w") as archive:
            for relative in installed.files or ():
                file = installed.locate_file(relative)
                if ".." not in relative.parts and file.is_file():
                    archive.write(file, str(relative))
    repository = tmp_path / "repository"
    repository.mkdir()
    (repository / "pyproject.toml").write_text('[tool.uv.workspace]\nmembers=["packages/*"]\n')
    declarations = (
        ("probe", "acme-probe", "acme_probe", '["acme-sibling", "dimos"]'),
        ("sibling", "acme-sibling", "acme_sibling", "[]"),
        ("host", "dimos", "dimos", "[]"),
    )
    for folder, distribution, package, dependencies in declarations:
        project = repository / "packages" / folder
        source = project / "src" / package
        source.mkdir(parents=True)
        (source / "__init__.py").write_text('VALUE = "first"\n')
        metadata = '\ndynamic=["entry-points"]\n' if folder == "probe" else "\n"
        suffix = (
            '\n[tool.dimos.blueprints]\nprobe="acme_probe:VALUE"\n[[tool.dynamic-metadata]]\nprovider="dimos"\n[tool.uv.sources]\nacme-sibling={workspace=true}\ndimos={workspace=true}\n'
            if folder == "probe"
            else "\n"
        )
        (project / "pyproject.toml").write_text(
            '[build-system]\nrequires=["scikit-build-core>=1.0,<2", "dimos-build-config"]\nbuild-backend="scikit_build_core.build"\n'
            f'[project]\nname="{distribution}"\nversion="0.1.0"\nrequires-python=">=3.10,<3.13"\ndependencies={dependencies}'
            + metadata
            + f'\n[tool.dimos]\npackage="src/{package}"\n'
            + suffix
        )
    host = repository / "packages/host/src/dimos/cli"
    host.mkdir()
    (host / "__init__.py").touch()
    (host / "entry.py").write_text("""import json, sys
from importlib.metadata import entry_points
from pathlib import Path
import shutil
def main():
    print(json.dumps({"python": sys.prefix, "host": __file__, "args": sys.argv[1:],
        "names": sorted(e.name for e in entry_points(group="dimos.blueprints"))}))
    if "fail" in sys.argv:
        raise SystemExit(17)
""")
    subprocess.run(
        ["git", "init", "-b", "fixture-main", str(repository)], check=True, capture_output=True
    )
    subprocess.run(["git", "-C", str(repository), "add", "."], check=True)
    subprocess.run(
        [
            "git",
            "-C",
            str(repository),
            "-c",
            "user.name=Fixture",
            "-c",
            "user.email=fixture@example.invalid",
            "commit",
            "-m",
            "fixture",
        ],
        check=True,
        capture_output=True,
    )
    monkeypatch.setenv("UV_FIND_LINKS", str(wheelhouse))
    monkeypatch.setenv("UV_CACHE_DIR", str(tmp_path / "uv-cache"))
    monkeypatch.setenv("UV_NO_INDEX", "true")
    monkeypatch.setenv("UV_OFFLINE", "false")
    monkeypatch.setenv("UV_PYTHON_DOWNLOADS", "never")
    monkeypatch.setenv("UV_PYTHON", sys.executable)
    return repository


def test_git_workspace_closure_lock_lifecycle_and_selected_dispatch(package_repository, tmp_path):
    repository = package_repository
    member = workspace_member(repository, "acme-probe")
    source = resolve_source(f"git+{repository.as_uri()}#subdirectory={member.as_posix()}")
    project = consumer_project(tmp_path / "consumer", create=True)
    add(project, source, branch="fixture-main")
    locked = (project / "uv.lock").read_bytes()
    assert "subdirectory=packages/sibling" in unquote(locked.decode())
    assert "subdirectory=packages/host" in unquote(locked.decode())
    sync(project)
    assert (project / "uv.lock").read_bytes() == locked
    root = Path(__file__).resolve().parents[2]
    env = {
        **os.environ,
        "PYTHONPATH": str(root),
        "UV_PROJECT_ENVIRONMENT": "/wrong/environment",
    }
    launcher = [sys.executable, "-m", "dimos.cli.entry", "--project", str(project)]
    result = subprocess.run(
        [*launcher, "list"], env=env, text=True, capture_output=True, check=True
    )
    report = json.loads(result.stdout)
    assert report["python"] == str(project / ".venv")
    assert str(project / ".venv") in report["host"]
    assert report["names"] == ["probe"]
    assert report["args"] == ["list"]
    result = subprocess.run([*launcher, "run", "fail"], env=env, capture_output=True)
    assert result.returncode == 17
    update(project, ("acme-probe",))
    assert (project / "uv.lock").read_bytes() == locked
    (repository / "packages/sibling/src/acme_sibling/__init__.py").write_text('VALUE="second"\n')
    subprocess.run(["git", "-C", str(repository), "add", "."], check=True)
    subprocess.run(
        [
            "git",
            "-C",
            str(repository),
            "-c",
            "user.name=Fixture",
            "-c",
            "user.email=fixture@example.invalid",
            "commit",
            "-m",
            "update",
        ],
        check=True,
        capture_output=True,
    )
    sync(project)
    assert (project / "uv.lock").read_bytes() == locked
    update(project, ("acme-probe",))
    assert (project / "uv.lock").read_bytes() == locked
    update(project, ())
    assert (project / "uv.lock").read_bytes() != locked
    (project / "pyproject.toml").write_text(
        (project / "pyproject.toml")
        .read_text()
        .replace('"acme-probe",', '"missing-package-for-lock-check",')
    )
    result = subprocess.run([*launcher, "list"], env=env, capture_output=True)
    assert result.returncode != 0
    assert b'"names"' not in result.stdout
    (project / "pyproject.toml").write_text(
        (project / "pyproject.toml")
        .read_text()
        .replace('"missing-package-for-lock-check",', '"acme-probe",')
    )
    remove(project, ("acme-probe",))
    assert "acme-probe" not in (project / "pyproject.toml").read_text()


def test_local_editable_does_not_join_author_workspace(package_repository, tmp_path):
    project = consumer_project(tmp_path / "editable-consumer", create=True)
    source = package_repository / "packages/sibling"
    add(project, str(source), editable=True)
    assert not (package_repository / "uv.lock").exists()
    assert str(source) in (project / "pyproject.toml").read_text()
    env_python = project / (".venv/Scripts/python.exe" if os.name == "nt" else ".venv/bin/python")
    result = subprocess.check_output(
        [str(env_python), "-I", "-c", "import acme_sibling; print(acme_sibling.VALUE)"], text=True
    )
    assert result.strip() == "first"
    (source / "src/acme_sibling/__init__.py").write_text('VALUE="editable-change"\n')
    result = subprocess.check_output(
        [str(env_python), "-I", "-c", "import acme_sibling; print(acme_sibling.VALUE)"], text=True
    )
    assert result.strip() == "editable-change"
