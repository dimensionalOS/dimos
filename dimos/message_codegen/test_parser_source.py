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

from hashlib import sha256
from io import BytesIO
import json
import tarfile

import pytest

from scripts import prepare_message_parser as preparation


@pytest.fixture
def archive(tmp_path, monkeypatch):
    toolkit = tmp_path / "toolkit"
    toolkit.mkdir()
    path = tmp_path / "source.tar.gz"
    revision = "a" * 40
    parser = b"# unchanged upstream parser\n"
    with tarfile.open(path, "w:gz") as bundle:
        for name, content in {
            "LICENSE": b"Apache-2.0\n",
            "rosidl_adapter/package.xml": b"<package/>\n",
            "rosidl_adapter/rosidl_adapter/__init__.py": b"",
            "rosidl_adapter/rosidl_adapter/parser.py": parser,
        }.items():
            entry = tarfile.TarInfo(f"rosidl-{revision}/{name}")
            entry.size = len(content)
            bundle.addfile(entry, BytesIO(content))
    lock = {
        "revision": revision,
        "url": "https://example.invalid/pinned.tar.gz",
        "sha256": sha256(path.read_bytes()).hexdigest(),
        "parser_sha256": sha256(parser).hexdigest(),
    }
    (toolkit / "parser-source.json").write_text(json.dumps(lock))
    monkeypatch.setattr(preparation, "TOOLKIT", toolkit)
    return path, parser


def test_offline_cache_reuses_unchanged_source_and_license(tmp_path, archive, monkeypatch):
    path, original = archive
    monkeypatch.setattr(preparation.requests, "get", lambda *a, **kw: pytest.fail("offline fetch"))
    preparation.prepare(tmp_path / "cache", tmp_path / "first", archive=path, offline=True)
    preparation.prepare(tmp_path / "cache", tmp_path / "second", offline=True)
    assert (tmp_path / "second/rosidl_adapter/parser.py").read_bytes() == original
    assert (tmp_path / "second/rosidl_adapter/LICENSE").read_text() == "Apache-2.0\n"


def test_missing_offline_source_does_not_fetch(tmp_path, archive, monkeypatch):
    monkeypatch.setattr(preparation.requests, "get", lambda *a, **kw: pytest.fail("offline fetch"))
    with pytest.raises(FileNotFoundError, match="Offline parser source missing"):
        preparation.prepare(tmp_path / "empty", tmp_path / "output", offline=True)
    assert not (tmp_path / "output").exists()


def test_corrupt_archive_does_not_replace_prepared_source(tmp_path, archive):
    path, original = archive
    output = tmp_path / "output"
    preparation.prepare(tmp_path / "cache", output, archive=path, offline=True)
    path.write_bytes(b"corrupt")
    with pytest.raises(ValueError, match="SHA256 mismatch"):
        preparation.prepare(tmp_path / "cache", output, archive=path, offline=True)
    assert (output / "rosidl_adapter/parser.py").read_bytes() == original


def test_parser_hash_mismatch_is_rejected_before_install(tmp_path, archive):
    path, _ = archive
    manifest = preparation.TOOLKIT / "parser-source.json"
    lock = json.loads(manifest.read_text())
    lock["parser_sha256"] = "0" * 64
    manifest.write_text(json.dumps(lock))
    with pytest.raises(ValueError, match="parser SHA256 mismatch"):
        preparation.prepare(tmp_path / "cache", tmp_path / "output", archive=path, offline=True)
    assert not (tmp_path / "output").exists()


def test_unsafe_archive_path_does_not_escape_cache(tmp_path, archive):
    path, _ = archive
    manifest = preparation.TOOLKIT / "parser-source.json"
    lock = json.loads(manifest.read_text())
    with tarfile.open(path, "w:gz") as bundle:
        member = tarfile.TarInfo(f"rosidl-{lock['revision']}/rosidl_adapter/../../escape")
        member.size = 1
        bundle.addfile(member, BytesIO(b"x"))
    lock["sha256"] = sha256(path.read_bytes()).hexdigest()
    manifest.write_text(json.dumps(lock))
    with pytest.raises(ValueError, match="Unsafe upstream archive path"):
        preparation.prepare(tmp_path / "cache", tmp_path / "output", archive=path, offline=True)
    assert not (tmp_path / "escape").exists()
    assert not (tmp_path / "output").exists()
