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

from dimos.memory2.cli.summary import main as _summary_main

mem_app = typer.Typer(help="memory2 store commands", no_args_is_help=True)
mem_app.command("summary")(_summary_main)


@mem_app.command()
def rerun(
    path: str = typer.Argument(..., help="Store: bare name (cwd, data/, LFS), .db or .mcap path"),
    out: str = typer.Option(None, "--out", help="Output .rrd (default: alongside the source)"),
    seconds: float = typer.Option(None, "--seconds", help="Only the first N seconds"),
    no_gui: bool = typer.Option(False, "--no-gui", help="Write the .rrd but don't open the viewer"),
) -> None:
    """Render a memory2 store into rerun (writes a .rrd, then opens the viewer)."""
    from dimos.memory2.cli.render import open_store, render_store

    render_store(open_store(path), out=out, seconds=seconds, no_gui=no_gui)
