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

"""Unified CLI for hosted replay data, node selection, and live transport."""

from __future__ import annotations

from enum import Enum
import json
from pathlib import Path

import typer

from dimos.constants import STATE_DIR
from dimos.hosted_data.nodes import (
    choose_server_url,
    load_replay_nodes,
    recommend_node,
)
from dimos.hosted_data.repository import (
    download_object,
    download_objects,
    list_objects,
    read_manifest,
    serve_repository,
    upload_file,
    upload_files,
    write_manifest,
)
from dimos.hosted_data.storage.s3 import S3ReplayRepository
from dimos.protocol.pubsub.impl.webrtc.mvp_cli import webrtc_mvp_app


class UploadKind(str, Enum):
    """Supported upload preparation modes."""

    AUTO = "auto"
    BLOB = "blob"
    MEMORY2 = "memory2"


hosted_data_app = typer.Typer(
    help="Publish, discover, download, and replay hosted DimOS data",
    no_args_is_help=True,
)
hosted_data_app.add_typer(
    webrtc_mvp_app,
    name="live",
    help="Measure the experimental WebRTC live-video path",
)


@hosted_data_app.command("serve")
def serve(
    root: Path = typer.Option(
        STATE_DIR / "replay-repository",
        "--root",
        help="Filesystem root for immutable objects",
    ),
    host: str = typer.Option("127.0.0.1", "--host"),
    port: int = typer.Option(8765, "--port", min=0, max=65535),
    token: str | None = typer.Option(
        None,
        "--token",
        envvar="DIMOS_REPLAY_REPOSITORY_TOKEN",
        hide_input=True,
    ),
    public_read: bool = typer.Option(
        False,
        "--public-read",
        help="Let anyone list, play, and download objects; uploads still require the token",
    ),
    backend: str = typer.Option(
        "filesystem",
        "--backend",
        help="Storage backend: filesystem or s3 (also supports R2 and MinIO)",
    ),
    s3_bucket: str | None = typer.Option(
        None,
        "--s3-bucket",
        envvar="DIMOS_REPLAY_S3_BUCKET",
    ),
    s3_prefix: str = typer.Option(
        "dimos-replays",
        "--s3-prefix",
        envvar="DIMOS_REPLAY_S3_PREFIX",
    ),
    s3_endpoint_url: str | None = typer.Option(
        None,
        "--s3-endpoint-url",
        envvar="DIMOS_REPLAY_S3_ENDPOINT_URL",
    ),
    s3_region: str | None = typer.Option(
        None,
        "--s3-region",
        envvar="AWS_DEFAULT_REGION",
    ),
    s3_access_key_id: str | None = typer.Option(
        None,
        "--s3-access-key-id",
        envvar="AWS_ACCESS_KEY_ID",
    ),
    s3_secret_access_key: str | None = typer.Option(
        None,
        "--s3-secret-access-key",
        envvar="AWS_SECRET_ACCESS_KEY",
        hide_input=True,
    ),
    s3_session_token: str | None = typer.Option(
        None,
        "--s3-session-token",
        envvar="AWS_SESSION_TOKEN",
        hide_input=True,
    ),
    s3_addressing_style: str = typer.Option(
        "auto",
        "--s3-addressing-style",
        envvar="DIMOS_REPLAY_S3_ADDRESSING_STYLE",
        help="S3 addressing style: auto, path, or virtual",
    ),
    node_name: str = typer.Option("local", "--node-name"),
    region: str = typer.Option(
        "other",
        "--region",
        help="Node region advertised by /healthz: china, us, or other",
    ),
    max_object_bytes: int | None = typer.Option(
        None,
        "--max-object-bytes",
        min=1,
        envvar="DIMOS_REPLAY_MAX_OBJECT_BYTES",
    ),
    max_repository_bytes: int | None = typer.Option(
        None,
        "--max-repository-bytes",
        min=1,
        envvar="DIMOS_REPLAY_MAX_REPOSITORY_BYTES",
    ),
    cdn_base_url: str | None = typer.Option(
        None,
        "--cdn-base-url",
        envvar="DIMOS_REPLAY_CDN_BASE_URL",
    ),
    tls_certfile: Path | None = typer.Option(
        None,
        "--tls-certfile",
        exists=True,
        dir_okay=False,
        envvar="DIMOS_REPLAY_TLS_CERTFILE",
    ),
    tls_keyfile: Path | None = typer.Option(
        None,
        "--tls-keyfile",
        exists=True,
        dir_okay=False,
        envvar="DIMOS_REPLAY_TLS_KEYFILE",
    ),
) -> None:
    """Run the MVP repository server."""
    if region not in {"china", "us", "other"}:
        raise typer.BadParameter("--region must be china, us, or other")
    if (tls_certfile is None) != (tls_keyfile is None):
        raise typer.BadParameter("--tls-certfile and --tls-keyfile must be provided together")
    if host not in {"127.0.0.1", "::1", "localhost"} and not token:
        raise typer.BadParameter(
            "DIMOS_REPLAY_REPOSITORY_TOKEN or --token is required for a non-loopback server"
        )
    storage = None
    if backend == "s3":
        if not s3_bucket:
            raise typer.BadParameter(
                "DIMOS_REPLAY_S3_BUCKET or --s3-bucket is required for the s3 backend"
            )
        storage = S3ReplayRepository(
            bucket=s3_bucket,
            prefix=s3_prefix,
            endpoint_url=s3_endpoint_url,
            region_name=s3_region,
            access_key_id=s3_access_key_id,
            secret_access_key=s3_secret_access_key,
            session_token=s3_session_token,
            addressing_style=s3_addressing_style,
        )
    elif backend != "filesystem":
        raise typer.BadParameter("--backend must be filesystem or s3")
    typer.echo(
        json.dumps(
            {
                "event": "ready",
                "host": host,
                "port": port,
                "root": str(root),
                "backend": backend,
                "auth": token is not None,
                "public_read": public_read,
                "node": node_name,
                "region": region,
                "tls": tls_certfile is not None,
            },
            sort_keys=True,
        )
    )
    try:
        serve_repository(
            root=root,
            host=host,
            port=port,
            token=token,
            public_read=public_read,
            repository=storage,
            node_name=node_name,
            region=region,
            max_object_bytes=max_object_bytes,
            max_repository_bytes=max_repository_bytes,
            cdn_base_url=cdn_base_url,
            tls_certfile=tls_certfile,
            tls_keyfile=tls_keyfile,
        )
    except KeyboardInterrupt:
        pass


@hosted_data_app.command("upload")
def upload(
    path: Path = typer.Argument(..., exists=True, dir_okay=False, readable=True),
    owner: str = typer.Option(..., "--owner", help="Developer or organization name"),
    repository: str = typer.Option(..., "--repository", "--repo"),
    server_url: str | None = typer.Option(None, "--server-url"),
    token: str | None = typer.Option(
        None,
        "--token",
        envvar="DIMOS_REPLAY_REPOSITORY_TOKEN",
        hide_input=True,
    ),
    kind: UploadKind = typer.Option(
        UploadKind.AUTO,
        "--kind",
        case_sensitive=False,
        help="Upload kind: auto, blob, or memory2 (.db snapshot plus index)",
    ),
    name: str | None = typer.Option(None, "--name", help="Portable memory2 dataset name"),
    retries: int = typer.Option(3, "--retries", min=0, max=10),
    backoff_seconds: float = typer.Option(0.5, "--backoff-seconds", min=0.0),
) -> None:
    """Upload a raw file or a consistent memory2 snapshot without ZIP packaging."""
    selected_server_url = choose_server_url(server_url)
    if kind is UploadKind.MEMORY2 or (kind is UploadKind.AUTO and path.suffix.lower() == ".db"):
        from dimos.hosted_data.integrations.memory2_replay import RemoteReplayReference
        from dimos.hosted_data.integrations.memory2_upload import upload_memory2_dataset

        upload_result = upload_memory2_dataset(
            path=path,
            server_url=selected_server_url,
            owner=owner,
            repository=repository,
            token=token,
            dataset=name,
            retries=retries,
            backoff_seconds=backoff_seconds,
        )
        payload = upload_result.to_dict()
        payload["replay_uri"] = RemoteReplayReference(
            server_url=selected_server_url,
            owner=owner,
            repository=repository,
            object_id=upload_result.dataset_object.object_id,
        ).to_uri()
    else:
        if name is not None:
            raise typer.BadParameter("--name is only valid for memory2 uploads")
        payload = upload_file(
            server_url=selected_server_url,
            owner=owner,
            repository=repository,
            path=path,
            token=token,
            retries=retries,
            backoff_seconds=backoff_seconds,
        ).to_dict()
    typer.echo(json.dumps(payload, sort_keys=True))


@hosted_data_app.command("list")
def list_repository(
    owner: str = typer.Option(..., "--owner", help="Developer or organization name"),
    repository: str = typer.Option(..., "--repository", "--repo"),
    server_url: str | None = typer.Option(None, "--server-url"),
    token: str | None = typer.Option(
        None,
        "--token",
        envvar="DIMOS_REPLAY_REPOSITORY_TOKEN",
        hide_input=True,
    ),
) -> None:
    """List downloadable objects owned by one developer repository."""
    objects = list_objects(
        server_url=choose_server_url(server_url),
        owner=owner,
        repository=repository,
        token=token,
    )
    typer.echo(json.dumps([item.to_dict() for item in objects], sort_keys=True))


@hosted_data_app.command("download")
def download(
    object_id: str = typer.Argument(..., help="SHA-256 object id returned by upload"),
    owner: str = typer.Option(..., "--owner", help="Developer or organization name"),
    repository: str = typer.Option(..., "--repository", "--repo"),
    output: Path | None = typer.Option(None, "--output", "-o"),
    server_url: str | None = typer.Option(None, "--server-url"),
    token: str | None = typer.Option(
        None,
        "--token",
        envvar="DIMOS_REPLAY_REPOSITORY_TOKEN",
        hide_input=True,
    ),
    retries: int = typer.Option(3, "--retries", min=0, max=10),
    backoff_seconds: float = typer.Option(0.5, "--backoff-seconds", min=0.0),
    overwrite: bool = typer.Option(False, "--overwrite"),
) -> None:
    """Download one object and verify its SHA-256 digest."""
    destination = download_object(
        server_url=choose_server_url(server_url),
        owner=owner,
        repository=repository,
        object_id=object_id,
        output=output,
        token=token,
        overwrite=overwrite,
        retries=retries,
        backoff_seconds=backoff_seconds,
    )
    typer.echo(json.dumps({"downloaded": str(destination), "object_id": object_id}, sort_keys=True))


@hosted_data_app.command("batch-upload")
def batch_upload(
    directory: Path = typer.Argument(..., exists=True, file_okay=False, readable=True),
    owner: str = typer.Option(..., "--owner", help="Developer or organization name"),
    repository: str = typer.Option(..., "--repository", "--repo"),
    pattern: str = typer.Option("*", "--pattern", help="Recursive filename glob"),
    manifest: Path | None = typer.Option(None, "--manifest"),
    server_url: str | None = typer.Option(None, "--server-url"),
    token: str | None = typer.Option(
        None,
        "--token",
        envvar="DIMOS_REPLAY_REPOSITORY_TOKEN",
        hide_input=True,
    ),
    workers: int = typer.Option(4, "--workers", min=1, max=32),
    retries: int = typer.Option(3, "--retries", min=0, max=10),
    backoff_seconds: float = typer.Option(0.5, "--backoff-seconds", min=0.0),
) -> None:
    """Upload a directory concurrently and write a completed manifest."""
    paths = tuple(sorted(path for path in directory.rglob(pattern) if path.is_file()))
    if not paths:
        raise typer.BadParameter(f"no files matched {pattern!r} below {directory}")
    result = upload_files(
        server_url=choose_server_url(server_url),
        owner=owner,
        repository=repository,
        paths=paths,
        token=token,
        workers=workers,
        retries=retries,
        backoff_seconds=backoff_seconds,
    )
    manifest_path = manifest or directory / "replay-manifest.json"
    write_manifest(manifest_path, result)
    typer.echo(
        json.dumps(
            {
                "manifest": str(manifest_path),
                "owner": result.owner,
                "repository": result.repository,
                "objects": len(result.objects),
            },
            sort_keys=True,
        )
    )


@hosted_data_app.command("batch-download")
def batch_download(
    manifest: Path = typer.Argument(..., exists=True, dir_okay=False, readable=True),
    output_dir: Path = typer.Option(..., "--output-dir"),
    server_url: str | None = typer.Option(None, "--server-url"),
    token: str | None = typer.Option(
        None,
        "--token",
        envvar="DIMOS_REPLAY_REPOSITORY_TOKEN",
        hide_input=True,
    ),
    workers: int = typer.Option(4, "--workers", min=1, max=32),
    retries: int = typer.Option(3, "--retries", min=0, max=10),
    backoff_seconds: float = typer.Option(0.5, "--backoff-seconds", min=0.0),
    overwrite: bool = typer.Option(False, "--overwrite"),
) -> None:
    """Download every object in a manifest concurrently and verify each hash."""
    result = read_manifest(manifest)
    paths = download_objects(
        server_url=choose_server_url(server_url),
        manifest=result,
        output_dir=output_dir,
        token=token,
        workers=workers,
        retries=retries,
        backoff_seconds=backoff_seconds,
        overwrite=overwrite,
    )
    typer.echo(json.dumps({"downloaded": [str(path) for path in paths]}, sort_keys=True))


@hosted_data_app.command("recommend")
def recommend_repository_node(
    force: bool = typer.Option(False, "--force", help="Ignore the hourly cached result"),
    timeout_seconds: float = typer.Option(3.0, "--timeout-seconds", min=0.1, max=30.0),
) -> None:
    """Probe configured China/US nodes and print the fastest healthy route."""
    nodes = load_replay_nodes()
    if not nodes:
        raise typer.BadParameter("DIMOS_REPLAY_NODES is not configured")
    result = recommend_node(nodes, force=force, timeout_seconds=timeout_seconds)
    typer.echo(json.dumps(result.to_dict(), sort_keys=True))


def main() -> None:
    hosted_data_app()


if __name__ == "__main__":
    main()
