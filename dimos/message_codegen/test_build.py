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
import tarfile

from pydantic import ValidationError
import pytest

from dimos.message_codegen.build import prepare_builtin_sources


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


@pytest.mark.parametrize("invalid", ["format", "path", "hash", "missing", "unknown"])
def test_invalid_source_lock_preserves_inputs(source_project, invalid):
    project, cache, _ = source_project
    target = prepare_builtin_sources(cache, offline=True, project=project)
    previous = (target / "stamp_msgs/msg/Stamp.msg").read_bytes()
    lock_path = project / "schema-sources.json"
    lock = json.loads(lock_path.read_text())
    if invalid == "format":
        lock["format"] = 2
    elif invalid == "hash":
        lock["sources"][0]["sha256"] = "invalid"
    elif invalid == "missing":
        del lock["sources"][0]["url"]
    elif invalid == "unknown":
        lock["sources"][0]["sha265"] = "0" * 64
    else:
        files = lock["sources"][0]["files"]
        files["../outside.msg"] = files.pop("stamp_msgs/msg/Stamp.msg")
    lock_path.write_text(json.dumps(lock))
    with pytest.raises(ValidationError):
        prepare_builtin_sources(cache, offline=True, project=project)
    assert (target / "stamp_msgs/msg/Stamp.msg").read_bytes() == previous


def test_failed_publish_restores_previous_inputs(source_project, monkeypatch):
    project, cache, _ = source_project
    target = prepare_builtin_sources(cache, offline=True, project=project)
    source = project / "custom/dimos_msgs/msg/Reading.msg"
    source.write_text("string replacement\n")
    rename = Path.rename

    def fail_publish(path, destination):
        if path.name == "inputs" and destination == target:
            raise OSError("publish failed")
        return rename(path, destination)

    monkeypatch.setattr(Path, "rename", fail_publish)
    with pytest.raises(OSError, match="publish failed"):
        prepare_builtin_sources(cache, offline=True, project=project)
    assert (target / "dimos_msgs/msg/Reading.msg").read_text() == "float64 value\n"
