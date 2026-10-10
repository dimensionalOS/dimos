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

"""Acceptance of native library identities and explicit unsupported semantics."""

import json
from pathlib import Path
import pickle
import sys

import numpy as np
import pytest

from dimos.message_codegen import native_build, registry
from dimos.message_codegen.generate import generate
from dimos.message_codegen.native_build import complete, save_complete, source_digest, stage_module


@pytest.fixture
def frozen_registry(monkeypatch):
    previous = sys.modules.get("rosbags.usertypes")
    monkeypatch.setattr(registry, "_store", None)
    monkeypatch.setattr(registry, "_packages", {})
    monkeypatch.setattr(registry, "_unsupported", set())
    monkeypatch.setattr(registry, "_schemas", {})
    monkeypatch.setattr(registry, "providers", lambda: ())
    try:
        yield
    finally:
        if previous is None:
            sys.modules.pop("rosbags.usertypes", None)
        else:
            sys.modules["rosbags.usertypes"] = previous


def package(tmp_path, name, definition):
    source = tmp_path / "interfaces" / "accept_msgs" / "msg" / f"{name}.msg"
    source.parent.mkdir(parents=True, exist_ok=True)
    source.write_text(definition)
    output = tmp_path / "output"
    generate(
        [tmp_path / "interfaces"],
        output,
        [f"accept_msgs/msg/{name}"],
        module="accept_messages",
        shared=True,
        languages=("python",),
    )
    return output


def test_frozen_native_classes_roundtrip_and_pickle(tmp_path, frozen_registry):
    root = package(tmp_path, "Reading", "std_msgs/Header header\nfloat64 value\n")
    store = registry.initialize((root,))
    header = store.types["std_msgs/msg/Header"](
        store.types["builtin_interfaces/msg/Time"](1, 2), "map"
    )
    cls = store.types["accept_msgs/msg/Reading"]
    value = cls(header, 1.5)
    assert not hasattr(value, "encode")
    assert type(pickle.loads(pickle.dumps(value)).header) is type(header)
    assert type(registry.decode(registry.encode(value), value.__msgtype__).header) is type(header)
    assert registry.initialize((root,)).types[header.__msgtype__] is type(header)


def test_late_registration_fails_without_replacing_existing_classes(tmp_path, frozen_registry):
    root = package(tmp_path / "one", "Reading", "float64 value\n")
    store = registry.initialize((root,))
    value = store.types["accept_msgs/msg/Reading"](1.5)
    newer = package(tmp_path / "two", "Other", "int32 value\n")
    with pytest.raises(RuntimeError, match="frozen"):
        registry.initialize((newer,))
    assert type(pickle.loads(pickle.dumps(value))) is type(value)


def test_schema_tampering_fails_before_registration(tmp_path, frozen_registry):
    root = package(tmp_path, "Reading", "float64 value\n")
    (root / "schemas.json").write_text(json.dumps({"accept_msgs/msg/Reading": "int32 value\n"}))
    with pytest.raises(ValueError, match="digest"):
        registry.initialize((root,))
    assert registry._store is None


def test_unsupported_native_python_bounds_are_not_silently_accepted(tmp_path, frozen_registry):
    root = package(tmp_path, "Limited", "float64[<=3] values\n")
    store = registry.initialize((root,))
    value = store.types["accept_msgs/msg/Limited"](np.array([1.0, 2.0, 3.0, 4.0]))
    with pytest.raises(NotImplementedError, match="bounds"):
        registry.encode(value)
    wire = bytes(store.serialize_cdr(value, value.__msgtype__))
    with pytest.raises(NotImplementedError, match="bounds"):
        registry.decode(wire, value.__msgtype__)


def test_native_python_defaults_are_explicit_not_rewritten(tmp_path, frozen_registry):
    root = package(tmp_path, "Reading", "float64 value 1.5\n")
    cls = registry.initialize((root,)).types["accept_msgs/msg/Reading"]
    with pytest.raises(TypeError):
        cls()
    assert cls(1.5).value == 1.5
    assert json.loads((root / "message-package.json").read_text())[
        "types_with_declared_defaults"
    ] == ["accept_msgs/msg/Reading"]


def test_cpp_and_rust_bounded_strings_fail_explicitly(tmp_path):
    source = tmp_path / "interfaces/accept_msgs/msg/Limited.msg"
    source.parent.mkdir(parents=True)
    source.write_text("string<=3 value\n")
    for language in ["cpp", "rust"]:
        with pytest.raises(NotImplementedError, match="Unsupported native"):
            generate(
                [tmp_path / "interfaces"],
                tmp_path / language,
                ["accept_msgs/msg/Limited"],
                languages=(language,),
            )


def test_source_cache_tracks_edits_renames_and_deletions_without_build_outputs(tmp_path):
    source = tmp_path / "module"
    source.mkdir()
    file = source / "main.cpp"
    file.write_text("int main() { return 0; }\n")
    digest = source_digest(source)
    staged, executable = stage_module(source, source / "build/app", tmp_path / "cache")
    assert (staged / "main.cpp").read_bytes() == file.read_bytes()
    assert executable == staged / "build/app"
    (source / "build").mkdir()
    (source / "build/app").write_text("built executable")
    assert source_digest(source) == digest
    assert stage_module(source, source / "build/app", tmp_path / "cache")[0] == staged
    file.rename(file.with_name("renamed.cpp"))
    renamed = source_digest(source)
    assert renamed != digest
    file.with_name("renamed.cpp").write_text("int main() { return 1; }\n")
    assert source_digest(source) != renamed
    file.with_name("renamed.cpp").unlink()
    assert source_digest(source) != digest


def test_native_cache_rejects_deleted_or_modified_artifacts(tmp_path):
    prefix = tmp_path / "install"
    prefix.mkdir()
    library = prefix / "library.so"
    library.write_bytes(b"original library")
    marker = tmp_path / "complete.json"
    save_complete(prefix, marker)
    assert complete(prefix, marker)
    library.write_bytes(b"changed library")
    assert not complete(prefix, marker)
    library.unlink()
    assert not complete(prefix, marker)


def test_native_build_rejects_changed_schema_before_preparing_support(tmp_path, monkeypatch):
    root = package(tmp_path, "Reading", "float64 value\n")
    (root / "schemas/accept_msgs/msg/Reading.msg").write_text("int32 value\n")

    def no_build(*args, **kwargs):
        raise AssertionError("Invalid schema started native preparation")

    monkeypatch.setattr(native_build, "prepare_support", no_build)
    with pytest.raises(ValueError, match="schema mismatch"):
        native_build.prepare_cpp(root, cache=tmp_path / "cache")
    assert not (tmp_path / "cache").exists()


@pytest.mark.parametrize("platform", ["linux", "darwin"])
def test_cache_identity_tracks_selected_compilers_and_cmake(monkeypatch, platform):
    monkeypatch.setattr(native_build.sys, "platform", platform)
    monkeypatch.setenv("SDKROOT", "/test-sdk")
    monkeypatch.setenv("CXX", "selected-cxx --target=test")
    monkeypatch.setenv("CC", "selected-cc")
    monkeypatch.setattr(native_build.shutil, "which", lambda name: "/tools/" + name)
    monkeypatch.setattr(native_build, "find_spec", lambda name: True)
    monkeypatch.setattr(native_build.metadata, "version", lambda name: "1.0")
    versions = {"selected-cxx": "CXX 1", "selected-cc": "CC 1", "cmake": "CMake 1"}
    commands = []

    def compiler_output(command, **kwargs):
        commands.append(command)
        return "test-target" if command[-1] == "-dumpmachine" else versions[command[0]]

    monkeypatch.setattr(native_build.subprocess, "check_output", compiler_output)
    first = native_build._toolchain_key()
    assert ["selected-cxx", "--target=test", "--version"] in commands
    assert ["selected-cc", "--version"] in commands
    versions["selected-cxx"] = "CXX 2"
    second = native_build._toolchain_key()
    assert second != first
    versions["selected-cc"] = "CC 2"
    third = native_build._toolchain_key()
    assert third != second
    versions["cmake"] = "CMake 2"
    assert native_build._toolchain_key() != third


@pytest.mark.parametrize("language", ["cpp", "rust"])
@pytest.mark.xfail(
    strict=True,
    raises=NotImplementedError,
    reason="CDR-L09: docs/development/message-limitations.md#cdr-l09",
)
def test_original_bounded_telemetry_fixture_generates_native_sources(tmp_path, language):
    root = Path(__file__).resolve().parents[2]
    names = generate(
        [root / "examples/message-codegen"],
        tmp_path / language,
        ["demo_msgs/msg/Telemetry"],
        "legacy_telemetry",
        languages=(language,),
    )
    assert "demo_msgs/msg/Telemetry" in names
