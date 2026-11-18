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

from collections.abc import Callable
import io
from pathlib import Path
import tarfile

from click.testing import Result
import pytest
from typer.testing import CliRunner

from dimos.cli.dimos import main
from dimos.cloud import data
from dimos.cloud.test_data import FakeTransport


@pytest.fixture
def cli(monkeypatch: pytest.MonkeyPatch) -> tuple[Callable[..., Result], FakeTransport]:
    transport = FakeTransport()
    monkeypatch.setenv("DIMOS_CLOUD_URL", "http://127.0.0.1:1")
    monkeypatch.setenv("DIMOS_CLOUD_API_KEY", "fixture-only")
    monkeypatch.setenv("DIMOS_CLOUD_UPLOAD_CODEC", "")
    monkeypatch.setattr(data, "HttpCloudRequest", lambda *_: transport)
    return lambda *args: CliRunner().invoke(main, ["data", *args]), transport


def test_directory_roundtrip(
    cli: tuple[Callable[..., Result], FakeTransport],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    run, transport = cli
    source, out = tmp_path / "latest", tmp_path / "restored"
    (source / "nested/empty").mkdir(parents=True)
    files = {".private": b"\0\xff", "nested/" + "長" * 42: b"data"}
    for name, content in files.items():
        (source / name).write_bytes(content)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("DIMOS_CLOUD_STAGING_DIR", str(source))
    for args in (("upload", "latest"), ("pull", "u0", "--dest", str(out))):
        result = run(*args)
        assert result.exit_code == 0, (result.output, result.exception)
    assert {str(p.relative_to(out)) for p in out.rglob("*")} == {*files, "nested", "nested/empty"}
    assert {name: (out / name).read_bytes() for name in files} == files

    (tmp_path / "occupied").mkdir()
    assert run("pull", "u0", "--dest", "occupied").exit_code == 1
    assert not list((tmp_path / "occupied").iterdir())
    raw = tmp_path / "same.tar"
    raw.write_bytes(transport.uploads["u0"]["blob"])
    result = run("upload", str(raw))
    assert result.exit_code == 1 and "different upload type" in result.output


def test_source_link_is_not_uploaded(
    cli: tuple[Callable[..., Result], FakeTransport], tmp_path: Path
) -> None:
    run, transport = cli
    tree = tmp_path / "tree"
    tree.mkdir()
    (tree / "link").symlink_to(tmp_path)
    assert run("upload", str(tree)).exit_code == 1
    assert not transport.uploads


@pytest.mark.parametrize("case", ["plain tar", "traversal", "link", "checksum", "member limit"])
def test_pull_checks_tar_before_publish(
    cli: tuple[Callable[..., Result], FakeTransport],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    case: str,
) -> None:
    run, transport = cli
    if case == "member limit":
        monkeypatch.setattr(data, "MAX_TAR_MEMBERS", 1)
    stream = io.BytesIO()
    with tarfile.open(fileobj=stream, mode="w") as archive:
        archive.addfile(tarfile.TarInfo("safe"))
        member = tarfile.TarInfo("../escaped" if case == "traversal" else "second")
        if case == "link":
            member.type, member.linkname = tarfile.LNKTYPE, "safe"
        archive.addfile(member)
    blob = stream.getvalue()
    src, out = tmp_path / "input.tar", tmp_path / "restored"
    src.write_bytes(blob)
    assert run("upload", str(src)).exit_code == 0
    if case != "plain tar":
        transport.uploads["u0"]["manifest"] = {**data.DIRECTORY_MANIFEST, "streams": []}
    if case == "checksum":
        transport.uploads["u0"]["sha256"] = "0" * 64
    result = run("pull", "u0", "--dest", str(out))
    if case == "plain tar":
        assert result.exit_code == 0 and out.read_bytes() == blob
    else:
        assert result.exit_code == 1 and not out.exists()
        assert not list(tmp_path.rglob("escaped"))
