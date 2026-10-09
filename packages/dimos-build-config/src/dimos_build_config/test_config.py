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

"""Contract and real distribution tests for the independent build integration."""

import configparser
import os
import subprocess
import sys
import tarfile
import zipfile

import pytest

from dimos_build_config import config
from dimos_build_config.metadata import dynamic_metadata


@pytest.fixture
def project(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    package = tmp_path / "src/acme_probe"
    package.mkdir(parents=True)
    (package / "__init__.py").write_text("raise AssertionError('build imported package')")
    (package / "module.py").write_text("class Probe: pass")
    (package / "native").mkdir()
    (package / "native/Cargo.lock").write_text("# source lock")
    (package / "native/main.rs").write_text("fn main() {}")
    (package / "resources").mkdir()
    (package / "resources/message.txt").write_text("resource")
    (tmp_path / "pyproject.toml").write_text("""[build-system]
requires = ["scikit-build-core>=1.0,<2", "dimos-build-config"]
build-backend = "scikit_build_core.build"
[project]
name = "acme-probe"
version = "0.1.0"
dynamic = ["entry-points"]
[tool.dimos]
package = "src/acme_probe"
[tool.dimos.blueprints]
probe = "acme_probe.module:Probe"
[[tool.dynamic-metadata]]
provider = "dimos"
""")
    return tmp_path


@pytest.mark.parametrize(
    "suffix, message",
    [
        (
            '\n[project.entry-points."dimos.blueprints"]\nother = "acme_probe.module:Probe"',
            "only in tool.dimos",
        ),
        ("\n[tool.scikit-build]\nwheel.cmake = true", "cannot enable wheel.cmake"),
    ],
)
def test_rejects_conflicting_declarations(project, suffix, message):
    path = project / "pyproject.toml"
    path.write_text(path.read_text() + suffix)
    with pytest.raises(ValueError, match=message):
        config(env={})


@pytest.mark.parametrize(
    "old, new, message",
    [
        ('package = "src/acme_probe"', 'package = "../outside"', "non-escaping"),
        ('package = "src/acme_probe"', 'package = "missing"', "missing"),
        ('probe = "acme_probe.module:Probe"', 'Bad = "acme_probe.module:Probe"', "blueprint name"),
        ('probe = "acme_probe.module:Probe"', 'probe = "not an entry"', "blueprint target"),
        ('dynamic = ["entry-points"]', "dynamic = []", "Blueprints require"),
        (
            'package = "src/acme_probe"',
            'package = "src/acme_probe"\npackaging = "prebuilt"',
            "Unknown",
        ),
    ],
)
def test_rejects_invalid_schema(project, old, new, message):
    path = project / "pyproject.toml"
    path.write_text(path.read_text().replace(old, new))
    with pytest.raises(ValueError, match=message):
        config(env={})


def test_metadata_without_import_and_unrelated_project(project):
    assert dynamic_metadata({}, {}) == {
        "entry-points": {"dimos.blueprints": {"probe": "acme_probe.module:Probe"}}
    }
    (project / "pyproject.toml").write_text('[project]\nname = "unrelated"')
    assert config(env={}) == {}


def test_source_distributions_and_editable_never_compile(project):
    guards = project / "guards"
    guards.mkdir()
    calls = project / "compiler-calls"
    for name in ("cargo", "rustc", "cmake", "ninja", "cc", "c++"):
        guard = guards / name
        guard.write_text('#!/bin/sh\necho invoked >> "$COMPILER_CALLS"\nexit 97\n')
        guard.chmod(0o755)
    for name in ("target/debug/a", "build/a", "dist/a", ".env", "secret.pem"):
        path = project / "src/acme_probe" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("must not ship")
    env = {
        **os.environ,
        "PATH": str(guards) + os.pathsep + os.environ["PATH"],
        "COMPILER_CALLS": str(calls),
    }
    subprocess.run(
        [sys.executable, "-m", "build", "--no-isolation"],
        env=env,
        check=True,
        capture_output=True,
        text=True,
    )
    (sdist,) = (project / "dist").glob("*.tar.gz")
    (wheel,) = (project / "dist").glob("*.whl")
    assert wheel.name.endswith("py3-none-any.whl")
    with tarfile.open(sdist) as archive:
        sources = archive.getnames()
    with zipfile.ZipFile(wheel) as archive:
        names = archive.namelist()
        metadata = configparser.ConfigParser()
        metadata.read_string(archive.read("acme_probe-0.1.0.dist-info/entry_points.txt").decode())
        assert dict(metadata["dimos.blueprints"]) == {"probe": "acme_probe.module:Probe"}
        assert archive.read("acme_probe/resources/message.txt") == b"resource"
        assert archive.read("acme_probe/native/Cargo.lock") == b"# source lock"
    assert not any(
        any(part in name for part in ("/target/", "/build/", "/dist/", "/.env", "secret.pem"))
        for name in names + sources
    )
    # Repeat installation regenerates metadata; editable source changes remain visible.
    prefix = project / "installed"
    command = [
        "uv",
        "pip",
        "install",
        "--python",
        sys.executable,
        "--target",
        str(prefix),
        "--no-deps",
        "--no-build-isolation",
        "--reinstall",
        "-e",
        str(project),
    ]
    subprocess.run(command, env=env, check=True, capture_output=True, text=True)
    path = project / "pyproject.toml"
    path.write_text(
        path.read_text().replace(
            'probe = "acme_probe.module:Probe"', 'updated = "acme_probe.module:Probe"'
        )
    )
    subprocess.run(command, env=env, check=True, capture_output=True, text=True)
    metadata = configparser.ConfigParser()
    metadata.read(prefix / "acme_probe-0.1.0.dist-info/entry_points.txt")
    assert dict(metadata["dimos.blueprints"]) == {"updated": "acme_probe.module:Probe"}
    (project / "src/acme_probe/__init__.py").write_text("")
    resource = project / "src/acme_probe/resources/message.txt"
    resource.write_text("edited resource")
    script = (
        "import site; site.addsitedir(" + repr(str(prefix)) + "); "
        "from importlib.resources import files; "
        "assert files('acme_probe').joinpath('resources/message.txt').read_text() == 'edited resource'"
    )
    subprocess.run(
        [sys.executable, "-c", script],
        cwd=project,
        env=env,
        check=True,
        capture_output=True,
        text=True,
    )
    assert not calls.exists()
