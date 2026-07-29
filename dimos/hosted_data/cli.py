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

"""Commands for serving and transferring hosted DimOS replay data."""

from __future__ import annotations

import json
import mimetypes
import os
from pathlib import Path
from typing import Any, NoReturn

import requests
import typer

from dimos.hosted_data.repository import ReplayRepository, sha256_file

_DEFAULT_SERVER_URL = "http://127.0.0.1:8765"
_DEFAULT_ROOT = Path.home() / ".local" / "share" / "dimos" / "replay-repository"
_CONNECT_TIMEOUT_SECONDS = 10
_TRANSFER_TIMEOUT_SECONDS = 3600


data_app = typer.Typer(help="Upload, download, and serve replay data", no_args_is_help=True)


def _objects_url(server_url: str, owner: str, repository: str) -> str:
    return f"{server_url.rstrip('/')}/v1/repositories/{owner}/{repository}/objects"


def _fail(message: str) -> NoReturn:
    typer.echo(f"Error: {message}", err=True)
    raise typer.Exit(code=1)


def _check_response(response: requests.Response) -> None:
    try:
        response.raise_for_status()
    except requests.RequestException as exc:
        detail = response.text.strip()
        _fail(detail or str(exc))


@data_app.command("serve")
def serve(
    root: Path = typer.Option(_DEFAULT_ROOT, "--root", help="Repository storage directory"),
    host: str = typer.Option("127.0.0.1", "--host", help="Address to listen on"),
    port: int = typer.Option(8765, "--port", help="TCP port"),
) -> None:
    """Start the hosted replay upload and download service."""
    try:
        import uvicorn

        from dimos.hosted_data.api import create_app
    except ImportError:
        _fail("server dependencies are missing; install DimOS with the 'web' extra")
    uvicorn.run(create_app(ReplayRepository(root)), host=host, port=port)


@data_app.command("upload")
def upload(
    path: Path = typer.Argument(..., exists=True, dir_okay=False, readable=True),
    owner: str = typer.Option("default", "--owner", envvar="DIMOS_DATA_OWNER"),
    repository: str = typer.Option("replays", "--repo", envvar="DIMOS_DATA_REPOSITORY"),
    server_url: str = typer.Option(
        _DEFAULT_SERVER_URL,
        "--server-url",
        envvar="DIMOS_DATA_SERVER_URL",
    ),
) -> None:
    """Stream one replay or video file to a repository server."""
    digest = sha256_file(path)
    headers = {
        "Content-Length": str(path.stat().st_size),
        "Content-Type": mimetypes.guess_type(path.name)[0] or "application/octet-stream",
        "X-Content-SHA256": digest,
    }
    try:
        with path.open("rb") as source:
            response = requests.put(
                _objects_url(server_url, owner, repository),
                params={"filename": path.name},
                data=source,
                headers=headers,
                timeout=(_CONNECT_TIMEOUT_SECONDS, _TRANSFER_TIMEOUT_SECONDS),
            )
    except requests.RequestException as exc:
        _fail(str(exc))
    _check_response(response)
    item = response.json()
    typer.echo(f"Uploaded {item['filename']} ({item['size_bytes']} bytes)")
    typer.echo(item["object_id"])


@data_app.command("list")
def list_objects(
    owner: str = typer.Option("default", "--owner", envvar="DIMOS_DATA_OWNER"),
    repository: str = typer.Option("replays", "--repo", envvar="DIMOS_DATA_REPOSITORY"),
    server_url: str = typer.Option(
        _DEFAULT_SERVER_URL,
        "--server-url",
        envvar="DIMOS_DATA_SERVER_URL",
    ),
    json_output: bool = typer.Option(False, "--json", help="Print machine-readable JSON"),
) -> None:
    """List objects in a remote repository."""
    try:
        response = requests.get(
            _objects_url(server_url, owner, repository),
            timeout=_CONNECT_TIMEOUT_SECONDS,
        )
    except requests.RequestException as exc:
        _fail(str(exc))
    _check_response(response)
    items: list[dict[str, Any]] = response.json()
    if json_output:
        typer.echo(json.dumps(items, indent=2))
        return
    if not items:
        typer.echo("No objects found.")
        return
    for item in items:
        typer.echo(f"{item['object_id']}  {item['size_bytes']:>12}  {item['filename']}")


@data_app.command("download")
def download(
    object_id: str = typer.Argument(..., help="SHA-256 object ID"),
    output: Path | None = typer.Option(None, "--output", "-o"),
    owner: str = typer.Option("default", "--owner", envvar="DIMOS_DATA_OWNER"),
    repository: str = typer.Option("replays", "--repo", envvar="DIMOS_DATA_REPOSITORY"),
    server_url: str = typer.Option(
        _DEFAULT_SERVER_URL,
        "--server-url",
        envvar="DIMOS_DATA_SERVER_URL",
    ),
    force: bool = typer.Option(False, "--force", help="Replace an existing output file"),
) -> None:
    """Stream one repository object to a local file and verify its SHA-256."""
    base_url = _objects_url(server_url, owner, repository)
    try:
        listing = requests.get(base_url, timeout=_CONNECT_TIMEOUT_SECONDS)
        _check_response(listing)
        metadata = next((item for item in listing.json() if item["object_id"] == object_id), None)
        if metadata is None:
            _fail("object not found")
        target = output or Path(metadata["filename"])
        if target.exists() and not force:
            _fail(f"output already exists: {target}; pass --force to replace it")
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_name(f".{target.name}.part")
        response = requests.get(
            f"{base_url}/{object_id}",
            stream=True,
            timeout=(_CONNECT_TIMEOUT_SECONDS, _TRANSFER_TIMEOUT_SECONDS),
        )
        _check_response(response)
        try:
            with temporary.open("wb") as destination:
                for chunk in response.iter_content(chunk_size=1024 * 1024):
                    if chunk:
                        destination.write(chunk)
            if sha256_file(temporary) != object_id:
                _fail("downloaded file failed SHA-256 verification")
            os.replace(temporary, target)
        finally:
            temporary.unlink(missing_ok=True)
            response.close()
    except requests.RequestException as exc:
        _fail(str(exc))
    typer.echo(f"Downloaded {target}")
