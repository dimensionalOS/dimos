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

from dataclasses import replace
import json

import pytest

from dimos.message_codegen.generate import generate
from dimos.message_codegen.ownership import Dependency


def packages(tmp_path):
    base = tmp_path / "base"
    generate([], base, ["std_msgs/msg/Header"], "base_messages", shared=True)
    dependency = Dependency.load(base)
    source = tmp_path / "interfaces/custom_msgs/msg/Reading.msg"
    source.parent.mkdir(parents=True)
    source.write_text("std_msgs/Header header\nfloat64 value\n")
    return dependency, source.parent.parent.parent


def test_external_package_owns_only_custom_type_and_preserves_schema_closure(tmp_path):
    dependency, interfaces = packages(tmp_path)
    output = tmp_path / "custom"
    generate(
        [interfaces],
        output,
        ["custom_msgs/msg/Reading"],
        "custom_messages",
        dependencies=(dependency,),
        shared=True,
    )
    manifest = json.loads((output / "message-package.json").read_text())
    assert manifest["owned"] == ["custom_msgs/msg/Reading"]
    assert set(manifest["schemas"]) == {
        "custom_msgs/msg/Reading",
        "std_msgs/msg/Header",
        "builtin_interfaces/msg/Time",
    }
    assert (
        "MSG: std_msgs/Header"
        in json.loads((output / "schemas.json").read_text())["custom_msgs/msg/Reading"]
    )
    assert "struct Header" not in (output / "cpp/messages.hpp").read_text()
    assert "pub struct Header" not in (output / "rust/src/lib.rs").read_text()


def test_dependency_schema_mismatch_fails_before_creating_output(tmp_path):
    dependency, interfaces = packages(tmp_path)
    changed = replace(dependency, schemas={})
    output = tmp_path / "custom"
    with pytest.raises(ValueError, match="Dependency schema mismatch"):
        generate([interfaces], output, ["custom_msgs/msg/Reading"], dependencies=(changed,))
    assert not output.exists()


def test_duplicate_owner_and_conflicting_package_versions_are_rejected(tmp_path):
    dependency, interfaces = packages(tmp_path)
    with pytest.raises(ValueError, match="Multiple owners"):
        generate(
            [interfaces],
            tmp_path / "out",
            ["custom_msgs/msg/Reading"],
            dependencies=(dependency, dependency),
        )
    with pytest.raises(ValueError, match="Conflicting package versions"):
        generate(
            [interfaces],
            tmp_path / "out",
            ["custom_msgs/msg/Reading"],
            dependencies=(dependency, replace(dependency, version="2.0.0")),
        )


def test_incompatible_dependency_abi_is_rejected(tmp_path):
    dependency, _ = packages(tmp_path)
    manifest = dependency.root / "message-package.json"
    data = json.loads(manifest.read_text())
    data["abi"] = "unknown"
    manifest.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="Incompatible message package ABI"):
        Dependency.load(dependency.root)
