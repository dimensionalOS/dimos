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

import subprocess
import sys
from types import ModuleType

import pytest

from dimos.message_codegen import providers, registry
from dimos.message_codegen.definitions import Definitions
from dimos.message_codegen.distribution import write_distribution
from dimos.message_codegen.generate import generate


def test_generation_is_reproducible_and_removes_stale_schemas(tmp_path):
    first, second = tmp_path / "one", tmp_path / "two"
    generate([], first, ["sensor_msgs/msg/Image"])
    generate([], second, ["sensor_msgs/msg/Image"])

    def contents(root):
        return {str(p.relative_to(root)): p.read_bytes() for p in root.rglob("*") if p.is_file()}

    assert contents(first) == contents(second)

    generate([], first, ["geometry_msgs/msg/Point"])

    assert not (first / "schemas/sensor_msgs/msg/Image.msg").exists()
    assert not (first / "rust/schemas/sensor_msgs/msg/Image.msg").exists()
    assert (first / "schemas/geometry_msgs/msg/Point.msg").exists()


def test_invalid_definition_does_not_create_output(tmp_path):
    source = tmp_path / "input/custom_msgs/msg/Value.msg"
    source.parent.mkdir(parents=True)
    source.write_text("missing_msgs/Absent value\n")
    output = tmp_path / "output"

    with pytest.raises(ValueError, match="unresolved"):
        generate([tmp_path / "input"], output, ["custom_msgs/msg/Value"])

    assert not output.exists()


def test_distribution_carries_source_closure_and_pinned_generator(tmp_path):
    names = generate([], tmp_path, ["sensor_msgs/msg/Image"])
    write_distribution(tmp_path, "example_messages", names)
    project = tmp_path / "python"

    assert (project / "example_messages_schemas/schemas/std_msgs/msg/Header.msg").is_file()
    assert (project / "_codegen/dimos/message_codegen/_vendor/rosidl_parser.py").is_file()
    assert "dimos.messages" in (project / "setup.py").read_text()
    assert "pybind11==3.0.1" in (project / "pyproject.toml").read_text()


def test_bundled_generator_wins_over_installed_dimos(tmp_path):
    names = generate([], tmp_path / "messages", ["geometry_msgs/msg/Point"])
    write_distribution(tmp_path / "messages", "example_messages", names)
    installed = tmp_path / "installed" / "dimos"
    installed.mkdir(parents=True)
    (installed / "__init__.py").write_text("raise RuntimeError('wrong installed generator')\n")
    bundled = tmp_path / "messages/python/_codegen"
    subprocess.run(
        [
            sys.executable,
            "-I",
            "-c",
            "import sys; from pathlib import Path; sys.path[:0] = sys.argv[1:]; "
            "from dimos.message_codegen import native_build; "
            "assert Path(native_build.__file__).is_relative_to(sys.argv[1])",
            str(bundled),
            str(installed.parent),
        ],
        check=True,
        cwd=tmp_path,
    )


def test_installed_schema_discovery_does_not_import_native_types(tmp_path, monkeypatch):
    source = tmp_path / "custom_msgs/msg/Value.msg"
    source.parent.mkdir(parents=True)
    source.write_text("geometry_msgs/Point position\n")
    provider = ModuleType("external_provider")
    monkeypatch.setattr(provider, "schema_root", lambda: tmp_path, raising=False)
    monkeypatch.setattr(providers, "providers", lambda: (provider,))

    names = [
        message.name
        for message in Definitions([], installed=True).resolve(["custom_msgs/msg/Value"])
    ]

    assert names == ["geometry_msgs/msg/Point", "custom_msgs/msg/Value"]


def test_registry_rejects_conflicting_installed_definitions(tmp_path, monkeypatch):
    source = tmp_path / "geometry_msgs/msg/Point.msg"
    source.parent.mkdir(parents=True)
    source.write_text("int32 x\n")
    monkeypatch.setattr(registry, "schema_roots", lambda: (tmp_path,))

    with pytest.raises(ValueError, match="Conflicting definition geometry_msgs/msg/Point"):
        registry.message_types()
