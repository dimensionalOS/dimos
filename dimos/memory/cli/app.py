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

"""``dimos mem`` — memory store commands."""

from __future__ import annotations

import typer

from dimos.memory.cli.summary import main as _summary_main

mem_app = typer.Typer(help="memory store commands", no_args_is_help=True)
mem_app.command("summary")(_summary_main)


@mem_app.command()
def rerun(
    path: str = typer.Argument(..., help="Store: bare name (cwd, data/, LFS), .db or .mcap path"),
    out: str = typer.Option(None, "--out", help="Output .rrd (default: alongside the source)"),
    seconds: float = typer.Option(None, "--seconds", help="Only the first N seconds"),
    no_gui: bool = typer.Option(False, "--no-gui", help="Write the .rrd but don't open the viewer"),
    root: str = typer.Option(
        None, "--root", help="Nest every stream under this entity path (<root>/<name>)"
    ),
    view: str = typer.Option(
        None,
        "--view",
        help="module:function returning a rerun_config, e.g. "
        "dimos.robot.unitree.go2.belief_rerun:go2_belief_rerun_config",
    ),
) -> None:
    """Render a memory store into rerun (writes a .rrd, then opens the viewer)."""
    from dimos.memory.cli.dataset import open_dataset
    from dimos.memory.cli.render import render_store

    # `module:function`, the same form `all_blueprints` uses, so a robot's view
    # lives with the robot instead of needing a registry here or a script of its
    # own. Called rather than passed as a value: a config carries closures and a
    # shared dict would leak one robot's statics into the next one's view.
    rerun_config = None
    if view:
        import importlib

        module_path, _, attr = view.partition(":")
        if not attr:
            raise typer.BadParameter("expected module:function", param_hint="--view")
        rerun_config = getattr(importlib.import_module(module_path), attr)()

    render_store(
        open_dataset(path),
        out=out,
        seconds=seconds,
        no_gui=no_gui,
        root=root,
        rerun_config=rerun_config,
    )
