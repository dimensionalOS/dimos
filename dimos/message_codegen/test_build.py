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

"""Source-build preparation is verified, reusable and preserves previous outputs."""

from hashlib import sha256
from io import BytesIO
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

import pytest

from dimos.message_codegen.build import copy_native_sdk, prepare_builtin_sources, prepare_web_dist


@pytest.fixture
def source_project(tmp_path):
    project = tmp_path / "project"
    source = project / "custom/dimos_msgs/msg/Reading.msg"
    source.parent.mkdir(parents=True)
    source.write_text("float64 value\n")
    (source.parents[1] / "LICENSE").write_text("Project license\n")
    payload = BytesIO()
    contents = b"uint32 sec\n"
    with tarfile.open(fileobj=payload, mode="w:gz") as archive:
        info = tarfile.TarInfo("upstream/stamp_msgs/msg/Stamp.msg")
        info.size = len(contents)
        archive.addfile(info, BytesIO(contents))
    data = payload.getvalue()
    url = "https://example.test/messages.tar.gz"
    lock = {
        "format": 1,
        "sources": [
            {
                "url": url,
                "sha256": sha256(data).hexdigest(),
                "prefix": "upstream/",
                "files": {
                    "stamp_msgs/msg/Stamp.msg": {
                        "path": "stamp_msgs/msg/Stamp.msg",
                        "sha256": sha256(contents).hexdigest(),
                    }
                },
            }
        ],
    }
    (project / "schema-sources.json").write_text(json.dumps(lock))
    cache = tmp_path / "cache"
    archives = cache / "archives"
    archives.mkdir(parents=True)
    (archives / (sha256(url.encode()).hexdigest() + ".tar.gz")).write_bytes(data)
    return project, cache, data


def test_cold_preparation_downloads_verified_inputs(source_project, mocker):
    project, cache, data = source_project
    shutil.rmtree(cache)
    response = mocker.patch("requests.get").return_value.__enter__.return_value
    response.content = data

    target = prepare_builtin_sources(cache, project=project)

    assert (target / "stamp_msgs/msg/Stamp.msg").read_text() == "uint32 sec\n"
    assert (target / "dimos_msgs/msg/Reading.msg").read_text() == "float64 value\n"
    assert (target / "licenses/dimos_msgs-LICENSE").read_text() == "Project license\n"


def test_prepared_inputs_and_local_rename_work_offline_without_archives(source_project):
    project, cache, _ = source_project
    target = prepare_builtin_sources(cache, offline=True, project=project)
    shutil.rmtree(cache)
    assert prepare_builtin_sources(cache, offline=True, project=project) == target
    source = project / "custom/dimos_msgs/msg/Reading.msg"
    source.unlink()
    source.with_name("Changed.msg").write_text("float64 value\nstring label\n")

    prepare_builtin_sources(cache, offline=True, project=project)

    assert not (target / "dimos_msgs/msg/Reading.msg").exists()
    assert (target / "dimos_msgs/msg/Changed.msg").read_text() == "float64 value\nstring label\n"
    assert (target / "stamp_msgs/msg/Stamp.msg").read_text() == "uint32 sec\n"


def test_bad_source_hash_preserves_prepared_inputs(source_project):
    project, cache, _ = source_project
    target = prepare_builtin_sources(cache, offline=True, project=project)
    path = project / "schema-sources.json"
    lock = json.loads(path.read_text())
    lock["sources"][0]["files"]["stamp_msgs/msg/Stamp.msg"]["sha256"] = "0" * 64
    path.write_text(json.dumps(lock))

    with pytest.raises(ValueError, match="SHA256 mismatch"):
        prepare_builtin_sources(cache, offline=True, project=project)

    assert (target / "stamp_msgs/msg/Stamp.msg").read_text() == "uint32 sec\n"
    assert (target / "dimos_msgs/msg/Reading.msg").read_text() == "float64 value\n"


def test_offline_cold_inputs_fail_without_creating_output(source_project):
    project, cache, _ = source_project
    shutil.rmtree(cache)
    with pytest.raises(FileNotFoundError, match="not cached"):
        prepare_builtin_sources(cache, offline=True, project=project)
    assert not (project / "interfaces").exists()


@pytest.fixture
def web_tree(tmp_path):
    web = tmp_path / "web"
    for name in ("sdk", "cockpit"):
        (web / name).mkdir(parents=True)
    return web


def test_web_build_publishes_both_bundles(web_tree, monkeypatch):
    monkeypatch.setattr(shutil, "which", lambda _: "/tool/deno")

    def build(command, *, cwd, check, timeout):
        assert "--frozen" in command and "--cached-only" in command
        if "--outDir" in command:
            output = Path(command[command.index("--outDir") + 1])
            output.mkdir()
            (output / ("sdk.js" if cwd.name == "sdk" else "index.html")).write_text(cwd.name)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(subprocess, "run", build)
    prepare_web_dist(web_tree, offline=True)
    assert (web_tree / "sdk/dist/sdk.js").read_text() == "sdk"
    assert (web_tree / "cockpit/dist/index.html").read_text() == "cockpit"
    monkeypatch.setattr(shutil, "which", lambda _: None)
    prepare_web_dist(web_tree, offline=True)
    assert (web_tree / "sdk/dist/sdk.js").read_text() == "sdk"


def test_failed_web_build_keeps_previous_bundle(web_tree, monkeypatch):
    previous = web_tree / "sdk/dist/sdk.js"
    previous.parent.mkdir()
    previous.write_text("previous")
    monkeypatch.setattr(shutil, "which", lambda _: "/tool/deno")

    def build(command, *, cwd, check, timeout):
        if cwd.name == "cockpit":
            raise subprocess.CalledProcessError(1, command)
        if "--outDir" in command:
            output = Path(command[command.index("--outDir") + 1])
            output.mkdir()
            (output / "sdk.js").write_text("replacement")
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(subprocess, "run", build)
    with pytest.raises(subprocess.CalledProcessError):
        prepare_web_dist(web_tree)
    assert previous.read_text() == "previous"
    assert not (web_tree / "cockpit/dist").exists()


def test_missing_web_tool_has_actionable_error(web_tree, monkeypatch):
    monkeypatch.setattr(shutil, "which", lambda _: None)
    with pytest.raises(RuntimeError, match="isolated pip install"):
        prepare_web_dist(web_tree)


def test_sdk_source_packaging_reconciles_deletions_and_excludes_build_outputs(tmp_path):
    root = tmp_path / "source"
    files = {
        "native/cpp/CMakeLists.txt": "project(sdk CXX)",
        "native/cpp/include/dimos/native/module.hpp": "#pragma once",
        "native/cpp/include/dimos/native/old.hpp": "old header",
        "native/cpp/tests/test.cpp": "not a shipped library header",
        "native/rust/dimos-module/Cargo.toml": '[package]\nname="dimos-module"',
        "native/rust/dimos-module/src/lib.rs": "pub mod cdr;",
        "native/rust/dimos-module/target/output": "not a shipped source",
        "native/rust/dimos-module-macros/Cargo.toml": '[package]\nname="dimos-module-macros"',
        "native/rust/dimos-module-macros/src/lib.rs": "// macros",
    }
    for name, text in files.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    target = tmp_path / "sdk"
    copy_native_sdk(root, target)
    assert (target / "cpp/include/dimos/native/module.hpp").read_text() == "#pragma once"
    assert (target / "rust/dimos-module/src/lib.rs").read_text() == "pub mod cdr;"
    assert not (target / "cpp/tests").exists()
    assert not (target / "rust/dimos-module/target").exists()
    (root / "native/cpp/include/dimos/native/old.hpp").unlink()
    copy_native_sdk(root, target)
    assert not (target / "cpp/include/dimos/native/old.hpp").exists()
    (root / "native/cpp/include/dimos/native/module.hpp").unlink()
    with pytest.raises(RuntimeError, match="SDK sources missing"):
        copy_native_sdk(root, target)
    assert (target / "cpp/include/dimos/native/module.hpp").read_text() == "#pragma once"
