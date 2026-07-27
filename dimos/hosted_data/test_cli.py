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

"""Tests for the unified hosted-data CLI."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from click import unstyle
import pytest
from typer.testing import CliRunner

from dimos.hosted_data import cli as hosted_data_cli
from dimos.hosted_data.nodes import NodeRecommendation
from dimos.hosted_data.repository import (
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
        hosted_data_cli.hosted_data_app,
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
        hosted_data_cli,
        "serve_repository",
        fake_serve_repository,
    )
    result = runner.invoke(
        hosted_data_cli.hosted_data_app,
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
        "max_object_bytes": None,
        "max_repository_bytes": None,
        "cdn_base_url": None,
        "tls_certfile": None,
        "tls_keyfile": None,
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
        hosted_data_cli,
        "S3ReplayRepository",
        fake_s3_repository,
    )
    monkeypatch.setattr(
        hosted_data_cli,
        "serve_repository",
        lambda **kwargs: received.setdefault("server", kwargs),
    )
    result = runner.invoke(
        hosted_data_cli.hosted_data_app,
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
        hosted_data_cli.hosted_data_app,
        ["serve", "--root", str(tmp_path), "--backend", "s3"],
    )

    assert result.exit_code == 2
    assert "--s3-bucket is required" in unstyle(result.output)


def test_serve_rejects_invalid_region(tmp_path: Path) -> None:
    result = runner.invoke(
        hosted_data_cli.hosted_data_app,
        ["serve", "--root", str(tmp_path), "--region", "europe"],
    )

    assert result.exit_code == 2
    assert "--region must be china, us, or other" in unstyle(result.output)


def test_serve_passes_tls_and_limit_configuration(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    cert = tmp_path / "server.crt"
    key = tmp_path / "server.key"
    cert.write_text("certificate")
    key.write_text("key")
    received: dict[str, Any] = {}
    monkeypatch.setattr(
        hosted_data_cli,
        "serve_repository",
        lambda **kwargs: received.update(kwargs),
    )

    result = runner.invoke(
        hosted_data_cli.hosted_data_app,
        [
            "serve",
            "--root",
            str(tmp_path / "objects"),
            "--tls-certfile",
            str(cert),
            "--tls-keyfile",
            str(key),
            "--max-object-bytes",
            "1000",
            "--max-repository-bytes",
            "5000",
            "--cdn-base-url",
            "https://cdn.example",
        ],
    )

    assert result.exit_code == 0
    assert json.loads(result.output)["tls"] is True
    assert received["tls_certfile"] == cert
    assert received["tls_keyfile"] == key
    assert received["max_object_bytes"] == 1000
    assert received["max_repository_bytes"] == 5000
    assert received["cdn_base_url"] == "https://cdn.example"


def test_serve_requires_tls_certificate_and_key_together(tmp_path: Path) -> None:
    cert = tmp_path / "server.crt"
    cert.write_text("certificate")

    result = runner.invoke(
        hosted_data_cli.hosted_data_app,
        ["serve", "--root", str(tmp_path), "--tls-certfile", str(cert)],
    )

    assert result.exit_code == 2
    assert "must be provided together" in result.output


def test_recommend_node_command(monkeypatch: pytest.MonkeyPatch) -> None:
    node = hosted_data_cli.load_replay_nodes(
        '[{"name":"cn","url":"https://cn.example","region":"china"}]'
    )[0]
    recommendation = hosted_data_cli.recommend_node
    monkeypatch.setattr(hosted_data_cli, "load_replay_nodes", lambda: (node,))
    monkeypatch.setattr(
        hosted_data_cli,
        "recommend_node",
        lambda *_, **__: NodeRecommendation(
            selected=node,
            detected_region="china",
            measured_at=1.0,
            probes=(),
        ),
    )

    result = runner.invoke(
        hosted_data_cli.hosted_data_app,
        ["recommend", "--force"],
    )

    assert result.exit_code == 0
    assert json.loads(result.output)["selected"]["url"] == "https://cn.example"
    assert recommendation is not hosted_data_cli.recommend_node


def test_upload_list_and_download_commands(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "capture.mp4"
    source.write_bytes(b"video")
    destination = tmp_path / "downloaded.mp4"
    item = _object()

    monkeypatch.setattr(hosted_data_cli, "upload_file", lambda **_: item)
    upload = runner.invoke(
        hosted_data_cli.hosted_data_app,
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

    monkeypatch.setattr(hosted_data_cli, "list_objects", lambda **_: [item])
    listing = runner.invoke(
        hosted_data_cli.hosted_data_app,
        ["list", "--owner", "alice", "--repo", "go2"],
    )
    assert listing.exit_code == 0
    assert json.loads(listing.output) == [item.to_dict()]

    monkeypatch.setattr(
        hosted_data_cli,
        "download_object",
        lambda **_: destination,
    )
    download = runner.invoke(
        hosted_data_cli.hosted_data_app,
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


def test_upload_auto_detects_a_memory2_database(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from dimos.hosted_data.integrations import memory2_upload

    source = tmp_path / "go2.db"
    source.write_bytes(b"sqlite")
    received: dict[str, Any] = {}

    class _Result:
        dataset_object = _object()

        def to_dict(self) -> dict[str, str]:
            return {"kind": "memory2"}

    def fake_upload_memory2_dataset(**kwargs: Any) -> _Result:
        received.update(kwargs)
        return _Result()

    monkeypatch.setattr(memory2_upload, "upload_memory2_dataset", fake_upload_memory2_dataset)
    monkeypatch.setattr(
        hosted_data_cli,
        "upload_file",
        lambda **_: pytest.fail("memory2 auto-detection must not use blob upload"),
    )

    result = runner.invoke(
        hosted_data_cli.hosted_data_app,
        [
            "upload",
            str(source),
            "--owner",
            "alice",
            "--repo",
            "go2",
            "--server-url",
            "https://replay.example",
            "--name",
            "field-test",
        ],
    )

    assert result.exit_code == 0
    payload = json.loads(result.output)
    assert payload["kind"] == "memory2"
    assert payload["replay_uri"] == (
        f"dimos-replay://alice/go2/{_Result.dataset_object.object_id}"
        "?server=https%3A%2F%2Freplay.example"
    )
    assert received["path"] == source
    assert received["server_url"] == "https://replay.example"
    assert received["dataset"] == "field-test"


def test_blob_upload_rejects_a_memory2_name(tmp_path: Path) -> None:
    source = tmp_path / "capture.db"
    source.write_bytes(b"raw")

    result = runner.invoke(
        hosted_data_cli.hosted_data_app,
        [
            "upload",
            str(source),
            "--owner",
            "alice",
            "--repo",
            "go2",
            "--kind",
            "blob",
            "--name",
            "invalid",
        ],
    )

    assert result.exit_code == 2
    assert "--name is only valid for memory2 uploads" in unstyle(result.output)


def test_live_transport_is_nested_below_data() -> None:
    result = runner.invoke(hosted_data_cli.hosted_data_app, ["live", "--help"])

    assert result.exit_code == 0
    assert "local" in result.output
    assert "publish" in result.output
    assert "subscribe" in result.output


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

    monkeypatch.setattr(hosted_data_cli, "upload_files", lambda **_: manifest)
    upload = runner.invoke(
        hosted_data_cli.hosted_data_app,
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
        hosted_data_cli,
        "download_objects",
        lambda **_: (restored,),
    )
    download = runner.invoke(
        hosted_data_cli.hosted_data_app,
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
        hosted_data_cli.hosted_data_app,
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


def test_serve_rejects_unknown_backend() -> None:
    result = runner.invoke(hosted_data_cli.hosted_data_app, ["serve", "--backend", "unknown"])

    assert result.exit_code == 2
    assert "--backend must be filesystem or s3" in unstyle(result.output)


def test_serve_handles_operator_interrupt(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        hosted_data_cli,
        "serve_repository",
        lambda **_: (_ for _ in ()).throw(KeyboardInterrupt),
    )

    result = runner.invoke(hosted_data_cli.hosted_data_app, ["serve"])

    assert result.exit_code == 0
    assert json.loads(result.output)["event"] == "ready"


def test_recommend_requires_configured_nodes(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("DIMOS_REPLAY_NODES", raising=False)

    result = runner.invoke(hosted_data_cli.hosted_data_app, ["recommend"])

    assert result.exit_code == 2
    assert "DIMOS_REPLAY_NODES is not configured" in unstyle(result.output)


def test_main_invokes_the_hosted_data_app(monkeypatch: pytest.MonkeyPatch) -> None:
    called = False

    def fake_app() -> None:
        nonlocal called
        called = True

    monkeypatch.setattr(hosted_data_cli, "hosted_data_app", fake_app)
    hosted_data_cli.main()

    assert called
