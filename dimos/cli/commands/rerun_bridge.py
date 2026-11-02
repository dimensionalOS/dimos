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

"""The rerun-bridge command; the implementation lives in dimos.visualization.rerun."""

from __future__ import annotations

import os

import typer

from dimos.core.global_config import global_config


def rerun_bridge_cmd(
    ctx: typer.Context,
    options: list[str] | None = typer.Argument(
        None, help="Bridge settings, e.g. --rerun-open web --memory-limit 4GB"
    ),
    show_help: bool = typer.Option(False, "--help"),
) -> None:
    """Launch the Rerun visualization bridge on its own."""
    from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.visualization.rerun.bridge import RerunBridgeModule, run_bridge

    # Unlike a bridge inside a stack, the standalone one serves the web viewer.
    parser = BlueprintConfigParser(RerunBridgeModule.blueprint(rerun_web=True))
    if show_help:
        typer.echo(ctx.get_help())
        typer.echo()
        typer.echo(parser.format_help())
        return
    try:
        parsed = parser.parse(options or (), environ=os.environ, global_overrides=ctx.obj)
    except BlueprintConfigError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error
    # The bridge runs in this process and reads the global config singleton,
    # which so far only has the global options given before the command name.
    global_config.update(**parsed.explicit_global_config_values())
    run_bridge(**parsed.module_kwargs("rerunbridgemodule"))
