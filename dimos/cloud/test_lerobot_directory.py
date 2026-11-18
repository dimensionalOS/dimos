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

import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
from typing import Any

import pytest
from typer.testing import CliRunner

from dimos.cloud import data, lerobot_directory as bundle
from dimos.cloud.test_data import FakeTransport

DATA = "data/chunk-000/file-000.parquet"
EP = "meta/episodes/chunk-000/file-000.parquet"
CAM = "observation.images.camera"
VIDEO = f"videos/{CAM}/chunk-000/file-000.mp4"
PRIVATE = b"PRIVATE_SENTINEL"


@pytest.fixture
def dataset(tmp_path: Path) -> Path:
    pa = pytest.importorskip("pyarrow")
    pq = pytest.importorskip("pyarrow.parquet")
    pd = pytest.importorskip("pandas")
    if shutil.which("ffmpeg") is None:
        pytest.skip("video fixture requires ffmpeg")
    root = tmp_path / "lerobot_data1"
    for name in [DATA, EP, VIDEO]:
        (root / name).parent.mkdir(parents=True, exist_ok=True)
    info = dict(
        codebase_version="v3.0",
        total_episodes=2,
        total_frames=6,
        fps=10,
        chunks_size=1000,
        data_files_size_in_mb=100,
        video_files_size_in_mb=200,
        total_tasks=1,
        robot_type="fixture",
        splits={"train": "0:2"},
        data_path=bundle.DATA,
        video_path=bundle.VIDEO,
        features={
            **{
                k: dict(dtype="float32" if k == "timestamp" else "int64", shape=[1], names=None)
                for k in bundle.BASE
            },
            "action": dict(
                dtype="float32", shape=[2], names=["joint", "second"], private=PRIVATE.decode()
            ),
            "private_notes": dict(dtype="string", shape=[1]),
            CAM: dict(
                dtype="video",
                shape=[16, 16, 3],
                names=["height", "width", "channels"],
                info={"video.fps": 10, "video.codec": "h264", "notes": PRIVATE.decode()},
            ),
        },
        annotations=PRIVATE.decode(),
    )
    (root / bundle.INFO).write_text(json.dumps(info))
    (root / bundle.STATS).write_text(
        json.dumps({"action": {"min": [0], "notes": PRIVATE.decode()}})
    )
    pd.DataFrame(
        {"task_index": [0], "notes": [PRIVATE.decode()]},
        index=pd.Index(["synthetic task"], name="task"),
    ).to_parquet(root / bundle.TASKS)
    table = pa.table(
        dict(
            index=list(range(6)),
            episode_index=[0] * 3 + [1] * 3,
            frame_index=[0, 1, 2] * 2,
            task_index=[0] * 6,
            timestamp=pa.array([0, 0.1, 0.2] * 2, type=pa.float32()),
            action=pa.array(
                [[float(i), -float(i)] for i in range(6)],
                type=pa.list_(pa.field("element", pa.float32(), metadata={b"private": PRIVATE}), 2),
            ),
            private_notes=[PRIVATE.decode()] * 6,
        )
    ).replace_schema_metadata({b"secret": PRIVATE})
    pq.write_table(table, root / DATA)
    pq.write_table(
        pa.table(
            {
                "episode_index": [0, 1],
                "length": [3, 3],
                "tasks": [["synthetic task"]] * 2,
                "dataset_from_index": [0, 3],
                "dataset_to_index": [3, 6],
                "data/chunk_index": [0, 0],
                "data/file_index": [0, 0],
                f"videos/{CAM}/chunk_index": [0, 0],
                f"videos/{CAM}/file_index": [0, 0],
                f"videos/{CAM}/from_timestamp": [0.0, 0.3],
                f"videos/{CAM}/to_timestamp": [0.3, 0.6],
                "private_notes": [PRIVATE.decode()] * 2,
            }
        ),
        root / EP,
    )
    subprocess.run(
        [
            "ffmpeg",
            "-nostdin",
            "-v",
            "error",
            "-f",
            "lavfi",
            "-i",
            "color=red:size=16x16:rate=10",
            "-frames:v",
            "6",
            "-c:v",
            "libx264",
            "-metadata",
            f"comment={PRIVATE.decode()}",
            str(root / VIDEO),
        ],
        check=True,
        timeout=30,
    )
    for name in (
        VIDEO.replace("file-000", "file-001"),
        VIDEO.replace(".camera/", ".other/"),
        DATA.replace("file-000", "file-001"),
        "annotations.json",
        ".cache/private",
    ):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(PRIVATE)
    return root


def cli_transport(monkeypatch: pytest.MonkeyPatch) -> tuple[CliRunner, Any, FakeTransport]:
    from dimos.cli.dimos import main

    transport = FakeTransport()
    monkeypatch.setenv("DIMOS_CLOUD_URL", "http://127.0.0.1:1")
    monkeypatch.setenv("DIMOS_CLOUD_API_KEY", "fixture-only")
    monkeypatch.setenv("DIMOS_CLOUD_UPLOAD_RETRIES", "0")

    def request(base_url: str, key: str, timeout: float) -> FakeTransport:
        assert base_url == "http://127.0.0.1:1" and key == "fixture-only"
        return transport

    monkeypatch.setattr(data, "HttpCloudRequest", request)
    return CliRunner(), main, transport


def test_public_cli_roundtrip(
    dataset: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner, app, transport = cli_transport(monkeypatch)
    result = runner.invoke(app, ["data", "upload", str(dataset)])
    assert result.exit_code == 0, (result.output, result.exception)
    row = transport.uploads["u0"]
    assert row["manifest"] == bundle.MANIFEST
    assert (row["kind"], row["filename"], row["content_encoding"]) == (
        "blob",
        "lerobot_data1.tar",
        None,
    )
    with tarfile.open(fileobj=io.BytesIO(row["blob"]), mode="r:") as archive:
        assert set(archive.getnames()) == {DATA, EP, VIDEO, bundle.INFO, bundle.STATS, bundle.TASKS}
        for member in archive:
            f = archive.extractfile(member)
            assert f is not None and PRIVATE not in f.read()
    out = tmp_path / "restored"
    result = runner.invoke(app, ["data", "pull", "u0", "--dest", str(out)])
    assert result.exit_code == 0, (result.output, result.exception)
    assert (out / VIDEO).is_file() and PRIVATE in (dataset / VIDEO).read_bytes()
    import pyarrow.parquet as pq

    schema = pq.read_schema(out / DATA)
    assert schema.metadata is None and "private_notes" not in schema.names
    assert schema.field("action").type.value_field.metadata is None
    assert runner.invoke(app, ["data", "pull", "u0", "--dest", str(out)]).exit_code == 1
    assert (out / VIDEO).is_file()
    result = runner.invoke(app, ["data", "upload", str(dataset)])
    assert result.exit_code == 0 and "already uploaded" in result.output
    # Optional separate CPU runtime (keeps torch/LeRobot out of the CLI env).
    if python := os.environ.get("DIMOS_TEST_LEROBOT_PYTHON"):
        subprocess.run(
            [
                python,
                "-c",
                """
import sys
from pathlib import Path
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
root = Path(sys.argv[1])
d = LeRobotDataset('local/fixture', root=root, video_backend='pyav')
assert len(d) == 6 and d.meta.total_episodes == 2
for i in range(len(d)):
    x = d[i]
    video = x['observation.images.camera']
    assert tuple(video.shape) == (3, 16, 16) and video.device.type == 'cpu'
    assert video[0].mean() > 0.9 and video[1].mean() < 0.1
    assert x['action'].tolist() == [i, -i] and x['episode_index'].item() == i // 3
    assert x['task'] == 'synthetic task'
assert not torch.cuda.is_initialized()
print('LeRobot CPU: 6 frames, 2 episodes, all 6 RGB samples decoded')
""",
                str(out),
            ],
            check=True,
            timeout=60,
            env={
                **os.environ,
                "HF_HOME": str(tmp_path / "hf"),
                "HF_HUB_OFFLINE": "1",
                "HF_DATASETS_OFFLINE": "1",
                "CUDA_VISIBLE_DEVICES": "",
            },
        )


@pytest.mark.parametrize("missing", [False, True])
def test_invalid_source_does_not_contact_transfer(
    dataset: Path, monkeypatch: pytest.MonkeyPatch, missing: bool
) -> None:
    runner, app, transport = cli_transport(monkeypatch)
    path = dataset / VIDEO
    original = path.with_suffix(".bak")
    path.rename(original)
    if not missing:
        path.symlink_to(original)
    result = runner.invoke(app, ["data", "upload", str(dataset)])
    assert result.exit_code == 1 and not transport.uploads


def test_restore_commit_cannot_replace_directory_created_after_preflight(tmp_path: Path) -> None:
    source, target = tmp_path / "stage", tmp_path / "target"
    source.mkdir()
    target.mkdir()
    (source / "content").touch()
    with pytest.raises(FileExistsError):
        bundle.publish_directory(source, target)
    assert not list(target.iterdir()) and (source / "content").is_file()


def wire_archive(entries: list[tuple[str, bytes, bytes]], fmt: int = tarfile.USTAR_FORMAT) -> bytes:
    output = io.BytesIO()
    with tarfile.open(fileobj=output, mode="w", format=fmt) as archive:
        for name, kind, content in entries:
            member = tarfile.TarInfo(name)
            member.type, member.size = kind, len(content)
            archive.addfile(member, io.BytesIO(content))
    return output.getvalue()


@pytest.mark.parametrize(
    "problem",
    [
        "traversal",
        "link",
        "hardlink",
        "special",
        "duplicate",
        "size",
        "count",
        "pax",
        "truncated",
        "digest",
        "occupied",
        "desktop",
    ],
)
def test_bad_bundle_is_not_published(
    problem: str, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner, app, transport = cli_transport(monkeypatch)
    entries = [(bundle.INFO, tarfile.REGTYPE, b"{}")]
    marker = bundle.MANIFEST
    if problem == "traversal":
        entries.append(("../escaped", tarfile.REGTYPE, b"bad"))
    elif problem in {"link", "hardlink", "special"}:
        kind = {"link": tarfile.SYMTYPE, "hardlink": tarfile.LNKTYPE, "special": tarfile.FIFOTYPE}[
            problem
        ]
        entries.append((DATA, kind, b""))
    elif problem == "duplicate":
        entries += entries
    elif problem == "size":
        monkeypatch.setattr(bundle, "MAX_BYTES", 1)
    elif problem == "count":
        monkeypatch.setattr(bundle, "MAX_MEMBERS", 1)
        entries.append((DATA, tarfile.REGTYPE, b"a"))
    elif problem == "pax":
        entries.append(("a" * 130, tarfile.REGTYPE, b"a"))
    elif problem == "desktop":
        marker = {"format": bundle.DESKTOP, "version": 1}
    blob = wire_archive(entries, tarfile.PAX_FORMAT if problem == "pax" else tarfile.USTAR_FORMAT)
    if problem == "truncated":
        blob = blob[:513]
    digest = "0" * 64 if problem == "digest" else hashlib.sha256(blob).hexdigest()
    transport.uploads["u0"] = dict(
        filename="dataset.tar",
        kind="blob",
        manifest=marker,
        content_encoding=None,
        sha256=digest,
        state="complete",
        size=len(blob),
        blob=blob,
    )
    out = tmp_path / "out"
    if problem == "occupied":
        out.symlink_to(tmp_path / "missing")
    result = runner.invoke(app, ["data", "pull", "u0", "--dest", str(out)])
    assert result.exit_code == 1, result.output
    assert not out.exists() and out.is_symlink() == (problem == "occupied")
    assert not (tmp_path / "escaped").exists()
    assert not list(tmp_path.glob(".dimos-*"))


def test_ordinary_tar_is_still_a_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    runner, app, _ = cli_transport(monkeypatch)
    raw = wire_archive([(bundle.INFO, tarfile.REGTYPE, b"{}")])
    src, dst = tmp_path / "ordinary.tar", tmp_path / "restored.tar"
    src.write_bytes(raw)
    for args in (["data", "upload", str(src)], ["data", "pull", "u0", "--dest", str(dst)]):
        result = runner.invoke(app, args)
        assert result.exit_code == 0, (result.output, result.exception)
    assert dst.read_bytes() == raw
