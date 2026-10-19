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

"""Explicit product inputs do not expand when unrelated schemas appear."""

import json
from pathlib import Path
import shutil

from dimos_cdrgen.source_package import prepare
import pytest


def test_unlisted_sources_do_not_expand_the_public_catalog(tmp_path):
    project = tmp_path / "project"
    shutil.copytree(
        Path(__file__).resolve().parents[2] / "packages/dimos-generated",
        project,
        ignore=shutil.ignore_patterns("build", "dist", "*.egg-info", "__pycache__"),
    )
    extra = project / "interfaces/extra_msgs/msg/NotExported.msg"
    extra.parent.mkdir(parents=True)
    extra.write_text("std_msgs/Header header\n")
    source = prepare(project)
    manifest = json.loads(
        (source / "dimos_generated_schemas/package/message-package.json").read_text()
    )
    assert len(manifest["owned"]) == 78
    assert "extra_msgs/msg/NotExported" not in manifest["owned"]
    assert "builtin_interfaces/msg/Time" in manifest["owned"]


def test_declared_root_requires_a_reason_and_a_resolvable_closure(tmp_path):
    project = tmp_path / "project"
    shutil.copytree(
        Path(__file__).resolve().parents[2] / "packages/dimos-generated",
        project,
        ignore=shutil.ignore_patterns("build", "dist", "*.egg-info", "__pycache__"),
    )
    catalog = project / "catalog.json"
    catalog.write_text(json.dumps({"format": 1, "roots": {"std_msgs/msg/Header": ""}}))
    with pytest.raises(ValueError, match="requires a reason"):
        prepare(project)
    catalog.write_text(json.dumps({"format": 1, "roots": {"std_msgs/msg/Header": "Public API"}}))
    source = prepare(project)
    manifest = json.loads(
        (source / "dimos_generated_schemas/package/message-package.json").read_text()
    )
    assert set(manifest["owned"]) == {"std_msgs/msg/Header", "builtin_interfaces/msg/Time"}
    (project / "interfaces/builtin_interfaces/msg/Time.msg").unlink()
    with pytest.raises(ValueError, match="unresolved message dependency"):
        prepare(project)
