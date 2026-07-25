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

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest
from typer.testing import CliRunner

from dimos.protocol.pubsub.impl.webrtc import replay_repository_cli
from dimos.protocol.pubsub.impl.webrtc.replay_nodes import NodeRecommendation
from dimos.protocol.pubsub.impl.webrtc.replay_repository import (
    ReplayManifest,
    ReplayObject,
    write_manifest,
)

runner = CliRunner()


def _object(
    *,
    filename: str = "capture.mp4",
    object_id: str = "a" * 64,
) -> ReplayObject:
    return ReplayObject(
        owner="alice",
        repository="go2",
        object_id=object_id,
        filename=filename,
        size_bytes=123,
        sha256=object_id,
        content_type="video/mp4",
        created_at="2026-07-25T00:00:00+00:00",
    )


def test_serve_requires_a_token_for_public_interfaces(tmp_path: Path) -> None:
    result = runner.invoke(
        replay_repository_cli.replay_repository_app,
        ["serve", "--root", str(tmp_path), "--host", "0.0.0.0"],
    )

    assert result.exit_code == 2
    assert "DIMOS_REPLAY_REPOSITORY_TOKEN" in result.output


def test_serve_uses_environment_token(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    received: dict[str, Any] = {}

    def fake_serve_repository(**kwargs: Any) -> None:
        received.update(kwargs)

    monkeypatch.setattr(
        replay_repository_cli,
        "serve_repository",
        fake_serve_repository,
    )
    result = runner.invoke(
        replay_repository_cli.replay_repository_app,
        [
            "serve",
            "--root",
            str(tmp_path),
            "--host",
            "0.0.0.0",
            "--port",
            "9876",
            "--public-read",
        ],
        env={"DIMOS_REPLAY_REPOSITORY_TOKEN": "secret"},
    )

    assert result.exit_code == 0
    assert json.loads(result.output)["event"] == "ready"
    assert received == {
        "root": tmp_path,
        "host": "0.0.0.0",
        "port": 9876,
        "token": "secret",
        "public_read": True,
        "repository": None,
        "node_name": "local",
        "region": "other",
    }


def test_serve_builds_an_s3_compatible_backend(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    received: dict[str, Any] = {}
    storage = object()

    def fake_s3_repository(**kwargs: Any) -> object:
        received["s3"] = kwargs
        return storage

    monkeypatch.setattr(
        replay_repository_cli,
        "S3ReplayRepository",
        fake_s3_repository,
    )
    monkeypatch.setattr(
        replay_repository_cli,
        "serve_repository",
        lambda **kwargs: received.setdefault("server", kwargs),
    )
    result = runner.invoke(
        replay_repository_cli.replay_repository_app,
        [
            "serve",
            "--root",
            str(tmp_path),
            "--backend",
            "s3",
            "--s3-bucket",
            "replays",
            "--s3-endpoint-url",
            "https://minio.example",
            "--s3-addressing-style",
            "path",
        ],
    )

    assert result.exit_code == 0
    assert received["s3"]["bucket"] == "replays"
    assert received["s3"]["endpoint_url"] == "https://minio.example"
    assert received["s3"]["addressing_style"] == "path"
    assert received["server"]["repository"] is storage


def test_serve_rejects_missing_s3_bucket(tmp_path: Path) -> None:
    result = runner.invoke(
        replay_repository_cli.replay_repository_app,
        ["serve", "--root", str(tmp_path), "--backend", "s3"],
    )

    assert result.exit_code == 2
    assert "--s3-bucket is required" in result.output


def test_serve_rejects_invalid_region(tmp_path: Path) -> None:
    result = runner.invoke(
        replay_repository_cli.replay_repository_app,
        ["serve", "--root", str(tmp_path), "--region", "europe"],
    )

    assert result.exit_code == 2
    assert "--region must be china, us, or other" in result.output


def test_recommend_node_command(monkeypatch: pytest.MonkeyPatch) -> None:
    node = replay_repository_cli.load_replay_nodes(
        '[{"name":"cn","url":"https://cn.example","region":"china"}]'
    )[0]
    recommendation = replay_repository_cli.recommend_node
    monkeypatch.setattr(replay_repository_cli, "load_replay_nodes", lambda: (node,))
    monkeypatch.setattr(
        replay_repository_cli,
        "recommend_node",
        lambda *_, **__: NodeRecommendation(
            selected=node,
            detected_region="china",
            measured_at=1.0,
            probes=(),
        ),
    )

    result = runner.invoke(
        replay_repository_cli.replay_repository_app,
        ["recommend-node", "--force"],
    )

    assert result.exit_code == 0
    assert json.loads(result.output)["selected"]["url"] == "https://cn.example"
    assert recommendation is not replay_repository_cli.recommend_node


def test_upload_list_and_download_commands(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "capture.mp4"
    source.write_bytes(b"video")
    destination = tmp_path / "downloaded.mp4"
    item = _object()

    monkeypatch.setattr(replay_repository_cli, "upload_file", lambda **_: item)
    upload = runner.invoke(
        replay_repository_cli.replay_repository_app,
        [
            "upload",
            str(source),
            "--owner",
            "alice",
            "--repo",
            "go2",
            "--server-url",
            "https://replay.example",
        ],
        env={"DIMOS_REPLAY_REPOSITORY_TOKEN": "secret"},
    )
    assert upload.exit_code == 0
    assert json.loads(upload.output)["object_id"] == item.object_id

    monkeypatch.setattr(replay_repository_cli, "list_objects", lambda **_: [item])
    listing = runner.invoke(
        replay_repository_cli.replay_repository_app,
        ["list", "--owner", "alice", "--repo", "go2"],
    )
    assert listing.exit_code == 0
    assert json.loads(listing.output) == [item.to_dict()]

    monkeypatch.setattr(
        replay_repository_cli,
        "download_object",
        lambda **_: destination,
    )
    download = runner.invoke(
        replay_repository_cli.replay_repository_app,
        [
            "download",
            item.object_id,
            "--owner",
            "alice",
            "--repo",
            "go2",
            "--output",
            str(destination),
        ],
    )
    assert download.exit_code == 0
    assert json.loads(download.output) == {
        "downloaded": str(destination),
        "object_id": item.object_id,
    }


def test_batch_commands_write_and_consume_manifest(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source_dir = tmp_path / "capture"
    source_dir.mkdir()
    (source_dir / "capture.mp4").write_bytes(b"video")
    manifest_path = tmp_path / "manifest.json"
    output_dir = tmp_path / "restored"
    item = _object()
    manifest = ReplayManifest(owner="alice", repository="go2", objects=(item,))

    monkeypatch.setattr(replay_repository_cli, "upload_files", lambda **_: manifest)
    upload = runner.invoke(
        replay_repository_cli.replay_repository_app,
        [
            "batch-upload",
            str(source_dir),
            "--owner",
            "alice",
            "--repo",
            "go2",
            "--manifest",
            str(manifest_path),
        ],
    )
    assert upload.exit_code == 0
    assert manifest_path.is_file()
    assert json.loads(upload.output)["objects"] == 1

    write_manifest(manifest_path, manifest)
    restored = output_dir / item.filename
    monkeypatch.setattr(
        replay_repository_cli,
        "download_objects",
        lambda **_: (restored,),
    )
    download = runner.invoke(
        replay_repository_cli.replay_repository_app,
        [
            "batch-download",
            str(manifest_path),
            "--output-dir",
            str(output_dir),
        ],
    )
    assert download.exit_code == 0
    assert json.loads(download.output) == {"downloaded": [str(restored)]}


def test_batch_upload_rejects_an_empty_directory(tmp_path: Path) -> None:
    result = runner.invoke(
        replay_repository_cli.replay_repository_app,
        [
            "batch-upload",
            str(tmp_path),
            "--owner",
            "alice",
            "--repo",
            "go2",
        ],
    )

    assert result.exit_code == 2
    assert "no files matched" in result.output
