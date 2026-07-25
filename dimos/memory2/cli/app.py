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

"""``dimos mem`` — memory2 store commands."""

from __future__ import annotations

import json

import typer

from dimos.memory2.cli.summary import main as _summary_main

mem_app = typer.Typer(help="memory2 store commands", no_args_is_help=True)
mem_app.command("summary")(_summary_main)


@mem_app.command()
def upload(
    dataset: str = typer.Argument(..., help="memory2 .db: bare name (cwd or data/) or path"),
    owner: str = typer.Option(..., "--owner", help="Developer or organization name"),
    repository: str = typer.Option(..., "--repository", "--repo"),
    server_url: str = typer.Option("http://127.0.0.1:8765", "--server-url"),
    token: str | None = typer.Option(
        None,
        "--token",
        envvar="DIMOS_REPLAY_REPOSITORY_TOKEN",
        hide_input=True,
    ),
    name: str | None = typer.Option(None, "--name", help="Portable dataset name"),
    retries: int = typer.Option(3, "--retries", min=0, max=10),
    backoff_seconds: float = typer.Option(0.5, "--backoff-seconds", min=0.0),
) -> None:
    """Upload a consistent, uncompressed memory2 snapshot and searchable index."""
    from dimos.memory2.cli.dataset import resolve_dataset
    from dimos.memory2.remote_dataset import upload_memory2_dataset

    path = resolve_dataset(dataset)
    result = upload_memory2_dataset(
        path=path,
        server_url=server_url,
        owner=owner,
        repository=repository,
        token=token,
        dataset=name,
        retries=retries,
        backoff_seconds=backoff_seconds,
    )
    typer.echo(json.dumps(result.to_dict(), sort_keys=True))


@mem_app.command()
def rerun(
    path: str = typer.Argument(..., help="Store: bare name (cwd, data/, LFS), .db or .mcap path"),
    out: str = typer.Option(None, "--out", help="Output .rrd (default: alongside the source)"),
    seconds: float = typer.Option(None, "--seconds", help="Only the first N seconds"),
    no_gui: bool = typer.Option(False, "--no-gui", help="Write the .rrd but don't open the viewer"),
    root: str = typer.Option(
        None, "--root", help="Nest every stream under this entity path (<root>/<name>)"
    ),
) -> None:
    """Render a memory2 store into rerun (writes a .rrd, then opens the viewer)."""
    from dimos.memory2.cli.dataset import open_dataset
    from dimos.memory2.cli.render import render_store

    render_store(open_dataset(path), out=out, seconds=seconds, no_gui=no_gui, root=root)
