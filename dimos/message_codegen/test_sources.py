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

"""Committed message inputs match their provenance record and refresh explicitly."""

from hashlib import sha256
from io import BytesIO
import json
from pathlib import Path
import tarfile

import pytest

from dimos.message_codegen import sources


def test_committed_inputs_match_the_provenance_record():
    assert sources.verify() == []


def _project(tmp_path: Path) -> Path:
    project = tmp_path / "project"

    def write(relative: str, contents: str) -> None:
        path = project / "interfaces" / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)

    write("dimos_msgs/msg/Reading.msg", "float64 value\n")
    write("licenses/dimos_msgs-LICENSE", "Project license\n")
    return project


def _lock(project: Path, archive: bytes) -> None:
    contents = b"uint32 sec\n"
    (project / "schema-sources.json").write_text(
        json.dumps(
            {
                "format": 1,
                "sources": [
                    {
                        "repository": "example/stamp_msgs",
                        "revision": "a" * 40,
                        "url": "https://example.test/stamp_msgs.tar.gz",
                        "sha256": sha256(archive).hexdigest(),
                        "prefix": "upstream/",
                        "files": {
                            "stamp_msgs/msg/Stamp.msg": {
                                "path": "stamp_msgs/msg/Stamp.msg",
                                "sha256": sha256(contents).hexdigest(),
                            },
                            "licenses/stamp_msgs-LICENSE": {
                                "path": "stamp_msgs/LICENSE",
                                "sha256": sha256(b"Upstream license\n").hexdigest(),
                            },
                        },
                    }
                ],
            }
        )
    )


def _archive() -> bytes:
    payload = BytesIO()
    with tarfile.open(fileobj=payload, mode="w:gz") as archive:
        for name, contents in (
            ("upstream/stamp_msgs/msg/Stamp.msg", b"uint32 sec\n"),
            ("upstream/stamp_msgs/LICENSE", b"Upstream license\n"),
        ):
            info = tarfile.TarInfo(name)
            info.size = len(contents)
            archive.addfile(info, BytesIO(contents))
    return payload.getvalue()


def test_vendor_rewrites_pinned_files_from_a_verified_archive(tmp_path):
    project = _project(tmp_path)
    data = _archive()
    _lock(project, data)
    cache = tmp_path / "cache"
    cache.mkdir()
    (cache / (sha256(data).hexdigest() + ".tar.gz")).write_bytes(data)

    sources.vendor(project=project, cache=cache)

    assert sources.verify(project) == []
    assert (project / "interfaces/stamp_msgs/msg/Stamp.msg").read_text() == "uint32 sec\n"
    assert (project / "interfaces/dimos_msgs/msg/Reading.msg").is_file()


def test_vendor_drops_files_no_longer_pinned_without_touching_local_definitions(tmp_path):
    project = _project(tmp_path)
    data = _archive()
    _lock(project, data)
    cache = tmp_path / "cache"
    cache.mkdir()
    (cache / (sha256(data).hexdigest() + ".tar.gz")).write_bytes(data)
    sources.vendor(project=project, cache=cache)
    retired = project / "interfaces/old_msgs/msg/Old.msg"
    retired.parent.mkdir(parents=True)
    retired.write_text("int32 value\n")

    sources.vendor(project=project, cache=cache)

    assert not retired.exists()
    assert (project / "interfaces/dimos_msgs/msg/Reading.msg").is_file()


def test_edited_or_missing_definition_is_reported(tmp_path):
    project = _project(tmp_path)
    data = _archive()
    _lock(project, data)
    cache = tmp_path / "cache"
    cache.mkdir()
    (cache / (sha256(data).hexdigest() + ".tar.gz")).write_bytes(data)
    sources.vendor(project=project, cache=cache)
    stamp = project / "interfaces/stamp_msgs/msg/Stamp.msg"

    stamp.write_text("uint32 sec\nuint32 nsec\n")
    assert any("does not match its pin" in problem for problem in sources.verify(project))

    stamp.unlink()
    assert any("missing vendored definition" in problem for problem in sources.verify(project))


def test_unexpected_inputs_and_bad_hashes_are_rejected(tmp_path):
    project = _project(tmp_path)
    data = _archive()
    _lock(project, data)
    cache = tmp_path / "cache"
    cache.mkdir()
    (cache / (sha256(data).hexdigest() + ".tar.gz")).write_bytes(data)
    sources.vendor(project=project, cache=cache)
    stray = project / "interfaces/stamp_msgs/msg/Stray.msg"
    stray.write_text("int32 value\n")
    assert any("unexpected message input" in problem for problem in sources.verify(project))

    lock = json.loads((project / "schema-sources.json").read_text())
    lock["sources"][0]["sha256"] = "invalid"
    (project / "schema-sources.json").write_text(json.dumps(lock))
    with pytest.raises(ValueError, match="Invalid archive hash"):
        sources.load(project)


def test_bad_archive_hash_leaves_committed_inputs_untouched(tmp_path):
    project = _project(tmp_path)
    data = _archive()
    _lock(project, data)
    cache = tmp_path / "cache"
    cache.mkdir()
    corrupted = b"not the archive"
    lock = json.loads((project / "schema-sources.json").read_text())
    (cache / (lock["sources"][0]["sha256"] + ".tar.gz")).write_bytes(corrupted)
    previous = (project / "interfaces/dimos_msgs/msg/Reading.msg").read_bytes()

    with pytest.raises(ValueError, match="Archive hash mismatch"):
        sources.vendor(project=project, cache=cache)

    assert (project / "interfaces/dimos_msgs/msg/Reading.msg").read_bytes() == previous
    assert not (project / "interfaces/stamp_msgs").exists()
