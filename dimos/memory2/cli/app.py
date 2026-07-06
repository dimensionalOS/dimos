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

import typer

mem_app = typer.Typer(help="memory2 store commands", no_args_is_help=True)


@mem_app.command()
def rerun(
    path: str = typer.Argument(..., help="Store to render (.mcap path/name or .db)"),
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


@mem_app.command()
def summary(
    dataset: str = typer.Argument(..., help="Dataset .db/.mcap: bare name (cwd or data/) or path"),
) -> None:
    """Print per-stream counts and time ranges for a recorded dataset."""
    from dimos.memory2.cli.dataset import open_dataset

    store = open_dataset(dataset)
    with store:
        for name in store.list_streams():
            store.streams[name]  # register so summary() includes it
        typer.echo(store.summary())

        # mcap files may carry channels we have no codec for (e.g. h264 video);
        # list them so the inventory is complete rather than silently filtered.
        uncodec = getattr(store, "uncodec_channels", None)
        for topic, (count, schema) in sorted(uncodec().items()) if uncodec else []:
            typer.echo(f"  (no codec) {topic}: {count} msgs [{schema or '?'}]")
