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

"""Exercise the public data CLI with synthetic files and a local fake transport."""

import hashlib
import io
import os
from pathlib import Path
import tarfile
from typing import Any

import pytest
from typer.testing import CliRunner

from dimos.cloud import data, directory_archive
from dimos.cloud.test_data import FakeTransport


@pytest.fixture
def cli_transport(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> tuple[CliRunner, Any, FakeTransport]:
    from dimos.cli.dimos import main

    transport = FakeTransport()
    monkeypatch.setattr(data, "DOWNLOADS_DIR", tmp_path / "downloads")
    monkeypatch.setenv("DIMOS_CLOUD_STAGING_DIR", str(tmp_path / "wire-staging"))
    monkeypatch.setenv("DIMOS_CLOUD_UPLOAD_CODEC", "lz4")
    monkeypatch.setenv("DIMOS_CLOUD_URL", "http://127.0.0.1:1")
    monkeypatch.setenv("DIMOS_CLOUD_API_KEY", "fixture-only")
    monkeypatch.setenv("DIMOS_CLOUD_UPLOAD_RETRIES", "0")

    def request(base_url: str, key: str, timeout: float) -> FakeTransport:
        assert base_url == "http://127.0.0.1:1" and key == "fixture-only"
        return transport

    monkeypatch.setattr(data, "HttpCloudRequest", request)
    return CliRunner(), main, transport


@pytest.mark.parametrize("empty", [False, True])
def test_cli_directory_roundtrip(
    cli_transport: tuple[CliRunner, Any, FakeTransport],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    empty: bool,
) -> None:
    runner, app, transport = cli_transport
    source = tmp_path / "latest"
    source.mkdir()
    monkeypatch.chdir(tmp_path)
    # Include the case where the archive's staging directory is inside the upload.
    monkeypatch.setenv("DIMOS_CLOUD_STAGING_DIR", str(source))
    files = (
        {}
        if empty
        else {
            ".private": bytes(range(256)) * 5200,
            "nested/" + "長" * 42: b"\x00\xff\n",
            "nested/empty-file": b"",
        }
    )
    folders = set() if empty else {"nested", "nested/empty-dir"}
    for name in folders:
        (source / name).mkdir(parents=True, exist_ok=True)
    for name, content in files.items():
        (source / name).write_bytes(content)

    result = runner.invoke(app, ["data", "upload", "latest"])
    assert result.exit_code == 0, (result.output, result.exception)
    row = transport.uploads["u0"]
    assert row["manifest"] == directory_archive.MANIFEST
    assert (row["filename"], row["kind"], row["content_encoding"]) == ("latest.tar", "blob", None)
    assert row["sha256"] == hashlib.sha256(row["blob"]).hexdigest()
    assert row["size"] >= directory_archive.MIN_WIRE
    with tarfile.open(fileobj=io.BytesIO(row["blob"]), mode="r:") as archive:
        assert {m.name for m in archive if m.isdir()} == folders
        assert {m.name for m in archive if m.isfile()} == files.keys()

    out = tmp_path / "downloads" / "20260830-120000-u0-latest"
    args = ["data", "pull", "u0"]
    if empty:
        out = tmp_path / "restored-empty"
        args += ["--dest", str(out)]
    result = runner.invoke(app, args)
    assert result.exit_code == 0, (result.output, result.exception)
    assert out.is_dir()
    assert {str(p.relative_to(out)) for p in out.rglob("*") if p.is_dir()} == folders
    assert {
        str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).digest()
        for p in out.rglob("*")
        if p.is_file()
    } == {name: hashlib.sha256(content).digest() for name, content in files.items()}
    result = runner.invoke(app, ["data", "pull", "u0", "--dest", str(out)])
    assert result.exit_code == 1 and "destination already exists" in result.output


@pytest.mark.parametrize("kind", ["root link", "parent link", "member link", "fifo"])
def test_cli_rejects_unsupported_source(
    cli_transport: tuple[CliRunner, Any, FakeTransport], tmp_path: Path, kind: str
) -> None:
    runner, app, transport = cli_transport
    root = tmp_path / "source"
    root.mkdir()
    (root / "nested").mkdir()
    if kind in ("root link", "parent link"):
        link = tmp_path / "link"
        link.symlink_to(root, target_is_directory=True)
        root = link if kind == "root link" else link / "nested"
    elif kind == "member link":
        (root / "link").symlink_to(tmp_path)
    else:
        os.mkfifo(root / "fifo")
    result = runner.invoke(app, ["data", "upload", str(root)])
    assert result.exit_code == 1 and not transport.uploads


def _tar(name: str, kind: bytes = tarfile.REGTYPE, *, fmt: int = tarfile.GNU_FORMAT) -> bytes:
    stream = io.BytesIO()
    with tarfile.open(fileobj=stream, mode="w", format=fmt) as archive:
        member = tarfile.TarInfo(name)
        member.type = kind
        member.size = 3 if member.isfile() else 0
        archive.addfile(member, io.BytesIO(b"abc"))
    return stream.getvalue()


@pytest.mark.parametrize(
    "bad", ["escape", "symlink", "hardlink", "fifo", "pax", "checksum", "truncated", "occupied"]
)
def test_cli_never_publishes_bad_directory_tar(
    cli_transport: tuple[CliRunner, Any, FakeTransport], tmp_path: Path, bad: str
) -> None:
    runner, app, transport = cli_transport
    blob = {
        "escape": _tar("../escaped"),
        "symlink": _tar("link", tarfile.SYMTYPE),
        "hardlink": _tar("link", tarfile.LNKTYPE),
        "fifo": _tar("fifo", tarfile.FIFOTYPE),
        "pax": _tar("a" * 120, fmt=tarfile.PAX_FORMAT),
        "truncated": _tar("truncated")[:512],
    }.get(bad, _tar("file"))
    transport.uploads["u0"] = dict(
        filename="tree.tar",
        kind="blob",
        manifest=directory_archive.MANIFEST,
        content_encoding=None,
        state="complete",
        size=len(blob),
        blob=blob,
        sha256="0" * 64 if bad == "checksum" else hashlib.sha256(blob).hexdigest(),
    )
    out = tmp_path / "out"
    if bad == "occupied":
        out.symlink_to(tmp_path / "missing")
    result = runner.invoke(app, ["data", "pull", "u0", "--dest", str(out)])
    assert result.exit_code == 1 and result.output
    assert not out.exists() and out.is_symlink() == (bad == "occupied")
    assert not (tmp_path / "escaped").exists()
    assert not list(tmp_path.rglob(".dimos-*-*"))


def test_atomic_publish_cannot_replace_empty_directory(tmp_path: Path) -> None:
    stage, dest = tmp_path / "stage", tmp_path / "dest"
    stage.mkdir()
    dest.mkdir()
    (stage / "file").touch()
    with pytest.raises(FileExistsError):
        directory_archive.publish_directory(stage, dest)
    assert not list(dest.iterdir()) and (stage / "file").is_file()


def test_cli_user_tar_stays_a_file(
    cli_transport: tuple[CliRunner, Any, FakeTransport], tmp_path: Path
) -> None:
    runner, app, transport = cli_transport
    src, dst = tmp_path / "ordinary.tar", tmp_path / "restored.tar"
    raw = _tar("a-file")
    src.write_bytes(raw)
    for args in (["data", "upload", str(src)], ["data", "pull", "u0", "--dest", str(dst)]):
        result = runner.invoke(app, args)
        assert result.exit_code == 0, (result.output, result.exception)
    assert not directory_archive.marked(transport.uploads["u0"]["manifest"])
    assert dst.read_bytes() == raw


def test_cli_raw_tar_dedup_does_not_reuse_directory_upload(
    cli_transport: tuple[CliRunner, Any, FakeTransport],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner, app, transport = cli_transport
    root = tmp_path / "tree"
    root.mkdir()
    result = runner.invoke(app, ["data", "upload", str(root)])
    assert result.exit_code == 0, (result.output, result.exception)
    raw_tar = tmp_path / "ordinary.tar"
    raw_tar.write_bytes(transport.uploads["u0"]["blob"])
    monkeypatch.setenv("DIMOS_CLOUD_UPLOAD_CODEC", "")
    result = runner.invoke(app, ["data", "upload", str(raw_tar)])
    assert result.exit_code == 1 and "different or unknown upload type" in result.output
