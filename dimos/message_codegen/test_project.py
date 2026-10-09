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

import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
from types import SimpleNamespace

import pytest

from dimos.message_codegen.backend import build_sdist, get_requires_for_build_wheel
from dimos.message_codegen.build import build_project
from dimos.message_codegen.generate import generate
import dimos.message_codegen.project as project_module
from dimos.message_codegen.project import Project, prepare


def project(tmp_path, definition="float64 value\n"):
    (tmp_path / "pyproject.toml").write_text("""[project]
name = "example-messages"
version = "1.2.3"
[tool.dimos.messages]
dependencies = {}
""")
    source = tmp_path / "interfaces/example_msgs/msg/Reading.msg"
    source.parent.mkdir(parents=True)
    source.write_text(definition)
    return Project.load(tmp_path), source


def test_defaults_and_cache_reuse_without_touching_generated_files(tmp_path):
    config, _ = project(tmp_path)
    output = prepare(config)
    header = output / "cpp/messages.hpp"
    stamp = header.stat().st_mtime_ns
    assert config.module == "example_messages"
    assert config.languages == ("python",)
    assert prepare(config) == output
    assert header.stat().st_mtime_ns == stamp


def test_schema_edits_deletion_and_rename_remove_stale_outputs(tmp_path):
    config, source = project(tmp_path)
    old = source.with_name("Old.msg")
    old.write_text("uint32 number\n")
    output = prepare(config)
    assert (output / "schemas/example_msgs/msg/Old.msg").is_file()
    old.unlink()
    source.rename(source.with_name("Renamed.msg"))
    prepare(config)
    assert not (output / "schemas/example_msgs/msg/Old.msg").exists()
    assert not (output / "schemas/example_msgs/msg/Reading.msg").exists()
    assert (output / "schemas/example_msgs/msg/Renamed.msg").is_file()
    source.with_name("Renamed.msg").write_text("int32 replacement\n")
    prepare(config)
    assert "replacement" in (output / "cpp/messages.hpp").read_text()


def test_missing_or_modified_generated_file_is_repaired(tmp_path):
    config, _ = project(tmp_path)
    output = prepare(config)
    header = output / "cpp/messages.hpp"
    original = header.read_bytes()
    header.write_text("corrupt")
    prepare(config)
    assert header.read_bytes() == original
    header.unlink()
    prepare(config)
    assert header.read_bytes() == original


def test_missing_standard_type_owner_is_not_silently_regenerated(tmp_path):
    config, _ = project(tmp_path, "std_msgs/Header header\n")
    with pytest.raises(ValueError, match="Missing dependency owners"):
        prepare(config)
    assert not config.output.exists()


def test_invalid_input_keeps_last_successful_generation(tmp_path):
    config, source = project(tmp_path)
    output = prepare(config)
    state = (output / "generation.json").read_bytes()
    source.write_text("unknown_msgs/Absent value\n")
    with pytest.raises(ValueError, match="unresolved"):
        prepare(config)
    assert (output / "generation.json").read_bytes() == state


def test_config_rejects_typo_instead_of_ignoring_it(tmp_path):
    config, _ = project(tmp_path)
    with (tmp_path / "pyproject.toml").open("a") as stream:
        stream.write('langauges = ["rust"]\n')
    with pytest.raises(ValueError, match="Unknown message configuration"):
        Project.load(tmp_path)


def test_python_backend_never_requests_native_sdk_or_rust(tmp_path, monkeypatch):
    project(tmp_path)
    monkeypatch.chdir(tmp_path)
    assert get_requires_for_build_wheel() == [
        "setuptools>=70",
        "wheel",
    ]
    name = build_sdist(str(tmp_path / "dist"))
    with tarfile.open(tmp_path / "dist" / name) as archive:
        assert sorted(archive.getnames()) == [
            "example_messages-1.2.3/interfaces/example_msgs/msg/Reading.msg",
            "example_messages-1.2.3/pyproject.toml",
        ]
    assert not (tmp_path / "build").exists()


def test_version_change_invalidates_generated_metadata(tmp_path):
    config, _ = project(tmp_path)
    output = prepare(config)
    path = tmp_path / "pyproject.toml"
    path.write_text(path.read_text().replace('"1.2.3"', '"1.2.4"'))
    prepare(Project.load(tmp_path))
    assert json.loads((output / "message-package.json").read_text())["version"] == "1.2.4"


def test_source_archive_is_reproducible(tmp_path, monkeypatch):
    project(tmp_path)
    monkeypatch.chdir(tmp_path)
    first = build_sdist(str(tmp_path / "first"))
    second = build_sdist(str(tmp_path / "second"))
    assert (tmp_path / "first" / first).read_bytes() == (tmp_path / "second" / second).read_bytes()


def test_module_override_cannot_change_distribution_identity(tmp_path):
    project(tmp_path)
    with (tmp_path / "pyproject.toml").open("a") as stream:
        stream.write('python-module = "unrelated"\n')
    with pytest.raises(ValueError, match="normalized project name"):
        Project.load(tmp_path)


def test_transitive_dependencies_are_discovered_and_versions_checked(tmp_path, monkeypatch):
    config, _ = project(tmp_path)
    base = tmp_path / "base_schemas/package"
    generate([], base, ["std_msgs/msg/Header"], "base", version="1.0.0", shared=True)
    middle = tmp_path / "middle_schemas/package"
    middle.mkdir(parents=True)
    metadata = json.loads((base / "message-package.json").read_text())
    metadata.update(module="middle", owned=[], dependencies={"base": "1.0.0"}, codec_owner="base")
    (middle / "message-package.json").write_text(json.dumps(metadata))
    monkeypatch.setattr(
        project_module,
        "find_spec",
        lambda name: SimpleNamespace(origin=str(tmp_path / name / "__init__.py")),
    )
    config.dependency_specs["middle"] = {"version": "1.0.0"}
    assert [dep.module for dep in config.dependencies()] == ["base", "middle"]
    config.dependency_specs["base"] = {"version": "2.0.0"}
    with pytest.raises(ValueError, match="mismatch|Conflicting package versions"):
        config.dependencies()


def test_python_only_build_needs_no_native_build_tool(tmp_path, monkeypatch):
    config, _ = project(tmp_path)
    monkeypatch.setattr("dimos.message_codegen.build.shutil.which", lambda tool: None)
    artifacts = build_project(config)
    assert artifacts["python_wheel"].endswith("-py3-none-any.whl")
    assert (tmp_path / "dist/example_messages-1.2.3.tar.gz").is_file()
    assert "cmake_prefix" not in artifacts
    assert "cargo_manifest" not in artifacts


@pytest.mark.parametrize(
    "resource, artifact",
    [
        ("_vendor/rosidl/serialization/msg__cdr.hpp.em", "cpp/messages.hpp"),
        ("templates/message_build.rs", "rust/build.rs"),
    ],
)
def test_upstream_template_and_cargo_adapter_changes_invalidate_cache(tmp_path, resource, artifact):
    project(tmp_path)
    toolkit = tmp_path / "toolkit"
    shutil.copytree(
        Path(project_module.__file__).parent,
        toolkit,
        ignore=shutil.ignore_patterns("__pycache__", "build", "*.egg-info"),
    )
    program = rf"""
import sys
from pathlib import Path
sys.path.insert(0, {str(tmp_path)!r})
from toolkit.project import Project, prepare
project = Project.load(Path({str(tmp_path)!r}))
output = prepare(project)
artifact = output / {artifact!r}
assert 'UPSTREAM_CACHE_PROBE' not in artifact.read_text()
resource = Path({str(toolkit / resource)!r})
resource.write_text(resource.read_text() + '\n// UPSTREAM_CACHE_PROBE\n')
prepare(project)
assert 'UPSTREAM_CACHE_PROBE' in artifact.read_text()
"""
    result = subprocess.run([sys.executable, "-I", "-c", program], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
