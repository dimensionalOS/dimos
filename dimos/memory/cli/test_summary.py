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

from contextlib import closing
import os
from pathlib import Path
import re
import sqlite3

import numpy as np
import pytest
from pytest_mock import MockerFixture
from typer.testing import CliRunner

from dimos.cli.dimos import main as cli_app
from dimos.memory.blobstore.file import FileBlobStore
from dimos.memory.blobstore.sqlite import SqliteBlobStore
from dimos.memory.cli.app import mem_app
from dimos.memory.cli.dataset import open_dataset
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.vectorstore.sqlite import SqliteVectorStore
from dimos.models.embedding.base import Embedding


def test_summary_counts_attached_poses_per_stream(sqlite_store: SqliteStore) -> None:
    lidar = sqlite_store.stream("lidar", float)
    lidar.append(1.0, ts=1.0, pose=(1, 2, 3))
    lidar.append(2.0, ts=2.0, pose=(1, 2, 3))
    lidar.append(3.0, ts=3.0)
    sqlite_store.stream("cmd_vel", float).append(1.0, ts=1.0)

    result = CliRunner().invoke(mem_app, ["summary", sqlite_store.config.path])

    assert result.exit_code == 0, result.output
    # Rich colours the output when CI forces a terminal.
    lines = re.sub(r"\x1b\[[0-9;]*m", "", result.output).splitlines()
    header = next(line for line in lines if "Stream" in line)
    column = [cell.strip() for cell in header.split("┃")].index("Poses")
    poses = {}
    for name in ("lidar", "cmd_vel"):
        row = next(line for line in lines if f" {name} " in line)
        poses[name] = row.split("│")[column].strip()
    assert poses == {"lidar": "2", "cmd_vel": "—"}


@pytest.mark.parametrize(
    "command",
    [
        "mem summary",
        "mem rerun --no-gui",
        "map pose-fill",
        "map rename",
        "map replay --no-gui",
        "map replay-marker --no-gui",
        "map global --no-gui",
    ],
)
def test_inspection_preserves_unrelated_database(tmp_path: Path, command: str) -> None:
    path = tmp_path / "other #1?.db"
    with closing(sqlite3.connect(path)) as conn:
        conn.execute("CREATE TABLE measurements (value REAL)")
    original = path.read_bytes()

    args = [*command.split(), str(path)]
    if command == "map rename":
        args.extend(["--out", str(tmp_path / "dest.db")])
    result = CliRunner().invoke(cli_app, args)

    assert list(tmp_path.iterdir()) == [path]
    assert path.read_bytes() == original
    assert result.exit_code == 2
    assert "recording" in result.output


def test_dataset_reads_wal_and_rejects_writes(tmp_path: Path) -> None:
    path = tmp_path / "live #1?.db"
    with SqliteStore(path=path) as writer:
        values = writer.stream("values1", float)
        values.append(1.25, ts=1.0)
        with open_dataset(path) as reader:
            stream = reader.stream("values1")
            assert stream.first().data == 1.25
            values.append(2.5, ts=2.0)
            assert stream.last().data == 2.5
            with pytest.raises(sqlite3.OperationalError, match="readonly"):
                stream.append(5.0, ts=3.0)
            with pytest.raises(sqlite3.OperationalError, match="No stream"):
                reader.stream("missing")
            with pytest.raises(sqlite3.OperationalError, match="No stream"):
                reader.stream("missing", float)
        assert writer.list_streams() == ["values1"]
        assert values.count() == 2


def test_read_only_dataset_follows_a_writer_that_starts_later(tmp_path: Path) -> None:
    path = tmp_path / "reopened.db"
    with SqliteStore(path=path) as writer:
        writer.stream("measure", float).append(1.0, ts=1.0)
    with closing(sqlite3.connect(path)) as conn:
        conn.execute("PRAGMA journal_mode=DELETE")
    original = path.read_bytes()

    with open_dataset(path) as reader:
        stream = reader.stream("measure")
        assert stream.last().data == 1.0
        assert path.read_bytes() == original
        with SqliteStore(path=path, must_exist=True) as writer:
            writer.stream("measure").append(2.5, ts=2.0)
            assert stream.last().data == 2.5


def test_read_external_payloads_without_changing_journal(tmp_path: Path) -> None:
    path, payloads, vectors = (
        tmp_path / f"{name}.db" for name in ("metadata", "payloads", "vectors")
    )
    embedding = Embedding(vector=np.array([1.0, 0.0], dtype=np.float32))
    with SqliteStore(path=path) as writer:
        writer.stream(
            "text",
            str,
            blob_store=SqliteBlobStore(path=str(payloads)),
            vector_store=SqliteVectorStore(path=str(vectors)),
        ).append("sample", embedding=embedding)
    for db in (path, payloads, vectors):
        with closing(sqlite3.connect(db)) as conn:
            conn.execute("PRAGMA journal_mode=DELETE")
    original = [db.read_bytes() for db in (path, payloads, vectors)]
    with open_dataset(path) as reader:
        assert reader.stream("text").first().data == "sample"
        assert [hit.data for hit in reader.stream("text").search(embedding, k=1)] == ["sample"]
    assert [db.read_bytes() for db in (path, payloads, vectors)] == original


def test_inspecting_missing_external_blob_root_does_not_create_it(tmp_path: Path) -> None:
    path, blobs = tmp_path / "metadata.db", tmp_path / "blobs"
    with SqliteStore(path=path) as writer:
        writer.stream("text", str, blob_store=FileBlobStore(root=str(blobs))).append("sample")
    blobs.rename(tmp_path / "offline")
    with open_dataset(path) as reader:
        assert reader.stream("text").count() == 1
        with pytest.raises(KeyError, match="No blob"):
            reader.stream("text").first().load()
    assert not blobs.exists()


def test_empty_file_is_distinct_from_empty_recording(tmp_path: Path) -> None:
    path = tmp_path / "empty.db"
    path.touch()
    result = CliRunner().invoke(mem_app, ["summary", str(path)])
    assert result.exit_code == 2
    assert "registry" in result.output
    assert path.read_bytes() == b""
    with SqliteStore(path=path):
        pass
    result = CliRunner().invoke(mem_app, ["summary", str(path)])
    assert result.exit_code == 0, result.output
    assert "total" in result.output


def test_rename_missing_source_creates_neither_file(tmp_path: Path) -> None:
    source, dest = tmp_path / "missing #1.db", tmp_path / "out.db"
    result = CliRunner().invoke(cli_app, ["map", "rename", str(source), "--out", str(dest)])
    assert result.exit_code == 2
    assert "found" in result.output
    assert not list(tmp_path.iterdir())


@pytest.mark.skipif(os.name != "posix", reason="uses POSIX file and directory permissions")
def test_closed_wal_archive_on_read_only_media(tmp_path: Path) -> None:
    archive = tmp_path / "archive"
    archive.mkdir()
    path = archive / "measurement.db"
    with SqliteStore(path=path) as writer:
        writer.stream("measure", float).append(2.5, ts=1.0)
    assert list(archive.iterdir()) == [path]
    original = path.read_bytes()
    path.chmod(0o444)
    archive.chmod(0o555)
    try:
        result = CliRunner().invoke(mem_app, ["summary", str(path)])
        assert result.exit_code == 0, result.output
        with open_dataset(path) as reader:
            assert reader.stream("measure").first().data == 2.5
        assert list(archive.iterdir()) == [path]
        assert path.read_bytes() == original
    finally:
        archive.chmod(0o755)
        path.chmod(0o644)


@pytest.mark.skipif(os.name != "posix", reason="uses POSIX file and directory permissions")
def test_sealed_recording_still_reads_committed_wal(tmp_path: Path) -> None:
    path = tmp_path / "measure.db"
    with SqliteStore(path=path) as writer:
        writer.stream("measure", float).append(1.0, ts=1.0)
    with SqliteStore(path=path, must_exist=True) as writer:
        writer.stream("measure").append(2.5, ts=2.0)
        path.chmod(0o444)
        tmp_path.chmod(0o555)
        try:
            with open_dataset(path) as reader:
                assert reader.stream("measure").last().data == 2.5
                with pytest.raises(sqlite3.OperationalError, match="No stream"):
                    reader.stream("missing", float)
        finally:
            tmp_path.chmod(0o755)
            path.chmod(0o644)


@pytest.mark.parametrize(
    ("command", "alias", "option"),
    [
        ("map replay --no-gui", "source.db", "--out"),
        ("map replay-marker --no-gui", "symlink.rrd", "--out"),
        ("map global --no-gui", "source.rrd", ""),
        ("map global --no-gui", "source.map.db", "--export"),
        ("map global --no-gui", "source_map.db", "--export-cloud"),
        ("map view --stream lidar", "hardlink.rrd", "--out"),
        ("mem rerun --no-gui", "source.rrd", ""),
    ],
)
def test_recording_output_cannot_alias_input(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, command: str, alias: str, option: str
) -> None:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    monkeypatch.chdir(tmp_path)
    source = tmp_path / "source.db"
    with SqliteStore(path=source) as store:
        cloud = PointCloud2.from_numpy(np.ones((1, 3)), frame_id="world", timestamp=1.0)
        store.stream("lidar", PointCloud2).append(cloud, ts=1.0)
        if "replay-marker" in command:
            from dimos.msgs.sensor_msgs.Image import Image

            store.stream("color_image", Image)
    output = tmp_path / alias
    if alias == "symlink.rrd":
        output.symlink_to(source)
    elif output != source:
        output.hardlink_to(source)
    args = [*command.split(), str(source)]
    if command.startswith("map global"):
        args.extend(["--device", "CPU:0", "--block-count", "16"])
    if option:
        args.append(option)
    if option == "--out":
        args.append(str(output))
    before = source.read_bytes()
    result = CliRunner().invoke(cli_app, args)
    assert source.read_bytes() == before
    assert result.exit_code == 2, result.output
    assert "input recording" in result.output
    with open_dataset(source) as reader:
        assert reader.stream("lidar").count() == 1


def test_rerun_preserves_recording_components(tmp_path: Path, mocker: MockerFixture) -> None:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    source, blobs, vectors = (tmp_path / f"{name}.db" for name in ("scan", "blobs", "vectors"))
    files = tmp_path / "payloads"
    embedding = Embedding(vector=np.array([1.0, 0.0], dtype=np.float32))
    with SqliteStore(path=source) as writer:
        cloud = PointCloud2.from_numpy(np.ones((1, 3)), frame_id="world", timestamp=1.0)
        writer.stream(
            "lidar",
            PointCloud2,
            blob_store=SqliteBlobStore(path=str(blobs)),
            vector_store=SqliteVectorStore(path=str(vectors)),
        ).append(cloud, ts=1.0, embedding=embedding)
        writer.stream("notes", str, blob_store=FileBlobStore(root=str(files))).append("sample")

    for db in (source, blobs, vectors):
        with closing(sqlite3.connect(db)) as conn:
            conn.execute("PRAGMA journal_mode=DELETE")
    (blob_file,) = (files / "notes").glob("*.bin")
    originals = {p: p.read_bytes() for p in (source, blobs, vectors, blob_file)}
    out = tmp_path / "preview.rrd"
    sink = mocker.patch("rerun.save", side_effect=AssertionError("sink opened"))
    for target in (blobs, vectors, blob_file):
        out.hardlink_to(target)
        try:
            result = CliRunner().invoke(
                mem_app, ["rerun", str(source), "--out", str(out), "--no-gui"]
            )
            assert {p: p.read_bytes() for p in originals} == originals
            sink.assert_not_called()
            assert result.exit_code == 2 and "input recording" in result.output
        finally:
            out.unlink()
    with open_dataset(source) as reader:
        assert len(reader.stream("lidar").search(embedding, k=1).first().data) == 1
        assert reader.stream("notes").first().data == "sample"
