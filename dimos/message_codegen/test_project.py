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

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
from types import SimpleNamespace
import zipfile

from dimos_cdrgen.generate import generate
from dimos_cdrgen.registry import schema_roots
import pytest

from dimos.message_codegen.backend import (
    build_sdist,
    build_wheel,
    get_requires_for_build_wheel,
    prepare_metadata_for_build_wheel,
)
from dimos.message_codegen.native import configure_cargo, prepare_rust
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
    header = output / "python/example_messages/_types.pyi"
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
    assert "replacement" in (output / "python/example_messages/_types.pyi").read_text()


def test_missing_or_modified_generated_file_is_repaired(tmp_path):
    config, _ = project(tmp_path)
    output = prepare(config)
    header = output / "python/example_messages/_types.pyi"
    original = header.read_bytes()
    header.write_text("corrupt")
    prepare(config)
    assert header.read_bytes() == original
    header.unlink()
    prepare(config)
    assert header.read_bytes() == original


def test_missing_standard_type_owner_is_not_silently_regenerated(tmp_path):
    config, _ = project(tmp_path, "std_msgs/Header header\n")
    with pytest.raises(ValueError, match="unresolved message dependency"):
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
        "setuptools>=77",
        "wheel",
    ]
    name = build_sdist(str(tmp_path / "dist"))
    with tarfile.open(tmp_path / "dist" / name) as archive:
        assert sorted(archive.getnames()) == [
            "example_messages-1.2.3/PKG-INFO",
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


def test_python_namespace_is_independent_of_native_and_distribution_identity(tmp_path):
    project(tmp_path)
    with (tmp_path / "pyproject.toml").open("a") as stream:
        stream.write('python-module = "robot.messages"\n')
    config = Project.load(tmp_path)
    output = prepare(config)
    metadata = json.loads((output / "message-package.json").read_text())
    assert config.python_module == "robot.messages"
    assert metadata["module"] == "example_messages"
    assert metadata["distribution"] == "example-messages"
    assert (output / "python/robot/messages/example_msgs/msg/__init__.py").is_file()


def test_transitive_dependencies_are_discovered_and_versions_checked(tmp_path, monkeypatch):
    config, _ = project(tmp_path)
    base = tmp_path / "base_schemas/package"
    generate(
        list(schema_roots()),
        base,
        ["std_msgs/msg/Header"],
        "base",
        version="1.0.0",
        shared=True,
        installed=True,
    )
    middle = tmp_path / "middle_schemas/package"
    middle.mkdir(parents=True)
    metadata = json.loads((base / "message-package.json").read_text())
    metadata.update(
        module="middle",
        python_module="middle",
        distribution="middle",
        owned=[],
        dependencies={"base": "1.0.0"},
        codec_owner="base",
    )
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


def test_python_package_generation_invokes_no_native_build_tool(tmp_path, monkeypatch):
    config, _ = project(tmp_path)

    def no_tool(*args, **kwargs):
        raise AssertionError("Python preparation invoked a process")

    monkeypatch.setattr(subprocess, "run", no_tool)
    output = prepare(config, ("python",))
    assert (output / "python/example_messages/__init__.py").is_file()
    assert not (output / "cpp").exists()
    assert not (output / "rust").exists()


def test_generator_version_invalidates_project_cache(tmp_path, monkeypatch):
    config, _ = project(tmp_path)
    output = prepare(config)
    state = (output / "generation.json").read_bytes()
    monkeypatch.setattr(project_module, "version", lambda _: "0.1.2")
    prepare(config)
    assert (output / "generation.json").read_bytes() != state


def test_selected_language_cache_invalidates_on_selection_edit_delete_and_rename(
    tmp_path, monkeypatch
):
    (tmp_path / "pyproject.toml").write_text(
        '[project]\nname="accept-messages"\nversion="0.1.0"\n[tool.dimos.messages]\ndependencies={}\n'
    )
    source = tmp_path / "interfaces/accept_msgs/msg/Reading.msg"
    source.parent.mkdir(parents=True)
    source.write_text("float64 value\n")
    config = Project.load(tmp_path)

    def no_tools(*args, **kwargs):
        raise AssertionError("Source packaging invoked an external build")

    monkeypatch.setattr(subprocess, "run", no_tools)
    output = prepare(config)
    stamp = (output / "generation.json").stat().st_mtime_ns
    assert prepare(config) == output
    assert (output / "generation.json").stat().st_mtime_ns == stamp
    assert not (output / "cpp").exists()
    assert not (output / "rust").exists()
    rust = prepare(config, ("rust",))
    assert (rust / "rust/build.rs").is_file()
    assert (output / "python").is_dir()
    assert rust != output
    source.rename(source.with_name("Renamed.msg"))
    source.with_name("Renamed.msg").write_text("int32 other\n")
    prepare(config)
    assert not (output / "schemas/accept_msgs/msg/Reading.msg").exists()
    assert (output / "schemas/accept_msgs/msg/Renamed.msg").read_text() == "int32 other\n"
    source.with_name("Renamed.msg").unlink()
    with pytest.raises(ValueError, match="No messages"):
        prepare(config)
    assert (output / "schemas/accept_msgs/msg/Renamed.msg").is_file()


def test_source_symlink_cannot_escape_project(tmp_path):
    config, source = project(tmp_path)
    outside = tmp_path.parent / (tmp_path.name + "-outside.msg")
    outside.write_text("uint32 value\n")
    source.unlink()
    source.symlink_to(outside)
    with pytest.raises(ValueError, match="inside the project"):
        prepare(config)


def test_extra_generated_file_invalidates_and_reconciles_cache(tmp_path):
    config, _ = project(tmp_path)
    output = prepare(config)
    extra = output / "python/unowned.py"
    extra.write_text("unexpected")
    prepare(config)
    assert not extra.exists()


def test_parallel_language_preparation_uses_independent_outputs(tmp_path):
    config, _ = project(tmp_path)
    with ThreadPoolExecutor(max_workers=2) as pool:
        python = pool.submit(prepare, config, ("python",))
        rust = pool.submit(prepare, config, ("rust",))
        python_output, rust_output = python.result(), rust.result()
    assert (python_output / "python/example_messages/_types.pyi").is_file()
    assert (rust_output / "rust/interfaces/example_msgs/msg/Reading.msg").is_file()
    assert python_output != rust_output


def test_wheel_preserves_metadata_provider_and_cache_across_pep517_hooks(tmp_path, monkeypatch):
    config, _ = project(tmp_path)
    (tmp_path / "README.md").write_text("Read this message package.\n")
    (tmp_path / "LICENSE").write_text("An example license fixture.\n")
    path = tmp_path / "pyproject.toml"
    path.write_text(
        path.read_text().replace(
            'version = "1.2.3"',
            'version = "1.2.3"\ndescription = "Test message metadata"\nreadme = "README.md"\nlicense = "MIT"\nlicense-files = ["LICENSE"]',
        )
    )
    output = prepare(config)
    stamp = (output / "generation.json").stat().st_mtime_ns
    monkeypatch.chdir(tmp_path)
    metadata = tmp_path / "metadata"
    metadata.mkdir()
    prepare_metadata_for_build_wheel(str(metadata))
    wheel = build_wheel(str(tmp_path / "dist"))
    assert (output / "generation.json").stat().st_mtime_ns == stamp
    assert wheel.endswith("-py3-none-any.whl")
    with zipfile.ZipFile(tmp_path / "dist" / wheel) as archive:
        names = archive.namelist()
        contents = archive.read(next(name for name in names if name.endswith("/METADATA"))).decode()
        entries = archive.read(
            next(name for name in names if name.endswith("/entry_points.txt"))
        ).decode()
        assert "Summary: Test message metadata" in contents
        assert "Read this message package." in contents
        assert "Requires-Dist: dimos-cdrgen==0.1.1" in contents
        assert "example_messages = example_messages_schemas.provider" in entries
        assert any(name.endswith("/licenses/LICENSE") for name in names)
        assert not any(name.endswith((".so", ".pyd", ".rlib")) for name in names)


def test_builtin_owner_is_reused_and_incompatible_schema_keeps_previous_output(tmp_path):
    config, source = project(tmp_path, "std_msgs/Header header\nfloat64 value\n")
    config.dependency_specs["dimos.messages"] = {"version": "0.0.14"}
    (dependency,) = config.dependencies()
    assert dependency.module == "dimos_generated"
    assert dependency.python_module == "dimos.messages"
    output = prepare(config, ("python", "cpp", "rust"))
    assert not (output / "cpp/std_msgs").exists()
    assert "pub use dimos_generated_messages::std_msgs;" in (output / "rust/src/lib.rs").read_text()
    code = """import sys
sys.path.insert(0, sys.argv[1])
from example_messages.example_msgs.msg import Reading
from dimos.messages.std_msgs.msg import Header
from dimos.messages.builtin_interfaces.msg import Time
from dimos_cdrgen.registry import encode, decode, schema
value = Reading(Header(Time(17, 23), "map"), 2.5)
assert type(value.header) is Header
assert type(decode(encode(value), Reading).header) is Header
assert decode(encode(value), Reading) == value
assert "MSG: std_msgs/Header" in schema(Reading.__msgtype__)
"""
    subprocess.run(
        [sys.executable, "-I", "-c", code, str(output / "python")], cwd=tmp_path, check=True
    )
    previous = (output / "generation.json").read_bytes()
    bad = tmp_path / "bad-dependency"
    shutil.copytree(dependency.root, bad)
    metadata = json.loads((bad / "message-package.json").read_text())
    metadata["schemas"]["std_msgs/msg/Header"] = "0" * 64
    (bad / "message-package.json").write_text(json.dumps(metadata))
    config.dependency_specs["dimos.messages"]["path"] = str(bad)
    with pytest.raises(ValueError, match="Dependency schema mismatch"):
        prepare(config, ("python", "cpp", "rust"))
    assert (output / "generation.json").read_bytes() == previous
    config.dependency_specs["dimos.messages"] = {"version": "0.0.15"}
    with pytest.raises(ValueError, match="identity/version mismatch"):
        prepare(config)


def test_cargo_sdk_export_generates_no_types_and_preserves_existing_user_settings(tmp_path):
    config, _ = project(tmp_path)
    folder = tmp_path / ".cargo"
    folder.mkdir()
    path = folder / "config.toml"
    path.write_text(
        '[net]\noffline = true\n[patch.crates-io]\nunrelated = { path = "/custom/source" }\n'
    )
    configure_cargo(config)
    data = project_module.tomllib.loads(path.read_text())
    assert data["net"]["offline"] is True
    assert data["patch"]["crates-io"]["unrelated"] == {"path": "/custom/source"}
    assert Path(data["patch"]["crates-io"]["dimos-message-build"]["path"]).is_dir()
    assert not config.output.exists()
    first = path.read_bytes()
    configure_cargo(config)
    assert path.read_bytes() == first
    path.write_text('[patch.crates-io]\ndimos-message-build = { path = "/different/sdk" }\n')
    changed = path.read_bytes()
    with pytest.raises(ValueError, match="Existing Cargo patch"):
        configure_cargo(config)
    assert path.read_bytes() == changed


def test_cargo_sdk_export_reuses_builtin_source_artifact_and_checks_schema(tmp_path):
    config, _ = project(tmp_path)
    config.dependency_specs["dimos.messages"] = {"version": "0.0.14"}
    (dependency,) = config.dependencies()
    path = configure_cargo(config)
    data = project_module.tomllib.loads(path.read_text())
    assert (
        Path(data["patch"]["crates-io"]["dimos-generated-messages"]["path"])
        == dependency.root / "rust"
    )
    assert not config.output.exists()


def test_cargo_hook_rejects_dependency_version_mismatch_before_generating(tmp_path):
    config, _ = project(tmp_path)
    config = Project(
        config.root,
        config.name,
        config.version,
        config.module,
        config.source,
        config.python_module,
        ("python", "rust"),
        {"dimos.messages": {"version": "0.0.14"}},
    )
    cargo = tmp_path / "Cargo.toml"
    cargo.write_text(
        '[package]\nname="example-messages-messages"\nversion="1.2.3"\n[dependencies]\ndimos-generated-messages="=0.0.15"\n'
    )
    with pytest.raises(ValueError, match="exact version =0.0.14"):
        prepare_rust(config, cargo)
    assert not config.output_for(("rust",)).exists()


def test_explicit_native_identity_does_not_rename_python_distribution(tmp_path):
    project(tmp_path)
    with (tmp_path / "pyproject.toml").open("a") as stream:
        stream.write('module = "native_example"\npython-module = "robot.messages"\n')
    config = Project.load(tmp_path)
    output = prepare(config)
    metadata = json.loads((output / "message-package.json").read_text())
    assert metadata["module"] == "native_example"
    assert metadata["python_module"] == "robot.messages"
    assert metadata["distribution"] == "example-messages"


def test_native_plan_can_cache_outside_immutable_source_project(tmp_path):
    config, source = project(tmp_path)
    cache = tmp_path / "cargo-output"
    output = prepare(config, ("rust",), cache=cache)
    assert output == cache / "rust"
    assert (
        output / "rust/interfaces/example_msgs/msg/Reading.msg"
    ).read_text() == source.read_text()
    assert not (config.root / "build").exists()
