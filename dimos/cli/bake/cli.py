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

"""dimos bake: link native modules into one host binary."""

from __future__ import annotations

from collections.abc import Sequence
import importlib
import json
from pathlib import Path
from typing import get_type_hints

import typer

from dimos.cli.bake.build import BUILDERS, build_host, install
from dimos.cli.bake.codegen import check_host_name, generate_crate
from dimos.cli.bake.discovery import (
    RegisteredModule,
    discover_modules,
    render_registry,
    select_modules,
)
from dimos.cli.bake.errors import BakeError
from dimos.cli.bake.graph import Graph, build_graph, parse_remap, render


def default_config(module: RegisteredModule) -> dict[str, object]:
    """The module's python wrapper config at its defaults, as the native struct sees it."""
    module_name, _, class_name = module.python_ref.partition(":")
    try:
        wrapper = getattr(importlib.import_module(module_name), class_name)
    except (ImportError, AttributeError) as exc:
        raise BakeError(
            f"module `{module.id}`: cannot import `{module.python_ref}`: {exc}"
        ) from exc
    config_type = get_type_hints(wrapper)["config"]
    return dict(config_type().to_config_dict())


def emit_config(graph: Graph, modules: Sequence[RegisteredModule]) -> dict[str, object]:
    """The stdin blob for the host, bar the `session` block only the deployment knows."""
    topics = graph.topics()
    return {
        "modules": {
            module.id: {
                "topics": topics[module.id],
                "config": default_config(module) or None,
            }
            for module in modules
        },
        "graph": graph.fingerprint(),
        "qos": graph.qos(),
        "suppress": list(graph.suppressed_topics()),
    }


def bake(
    modules: list[str] = typer.Argument(
        None, help="Module ids to bake, e.g. `ray-tracing mls-planner`."
    ),
    out: Path = typer.Option(
        None, "-o", "--out", help="Where to write the host binary; its name names the host."
    ),
    target: str = typer.Option(None, "--target", help="Rust target triple. Default: host-native."),
    suppress: list[str] = typer.Option(
        None,
        "--suppress",
        help="Keep this topic inside the host (name or full topic). Repeatable.",
    ),
    remap: list[str] = typer.Option(
        None, "--remap", help="Rename a port: <module>.<port>=<name>. Repeatable."
    ),
    builder: str = typer.Option("cargo", "--builder", help=f"Build driver: {', '.join(BUILDERS)}."),
    debug: bool = typer.Option(False, "--debug", help="Build the dev profile instead of release."),
    dry_run: bool = typer.Option(False, "--dry-run", help="Print the graph and stop."),
    emit_config_to: Path = typer.Option(
        None, "--emit-config", help="Also write a ready-to-pipe stdin JSON config here."
    ),
    list_modules: bool = typer.Option(
        False, "--list", help="List registered native modules and exit."
    ),
) -> None:
    """Compose rust native modules into a single host binary."""
    try:
        registry = discover_modules()
        if list_modules:
            typer.echo(render_registry(registry))
            return

        if out is None:
            raise BakeError("-o/--out is required: its filename names the host binary")
        host = out.name
        check_host_name(host)
        selected = select_modules(registry, modules or [])
        graph = build_graph(
            host,
            selected,
            remaps=dict(parse_remap(r) for r in remap or []),
            suppress=suppress or [],
        )

        typer.echo(render(graph))
        typer.echo("")

        if emit_config_to is not None:
            emit_config_to.parent.mkdir(parents=True, exist_ok=True)
            # The host reads its config with a single read_line, so the blob must
            # stay on one line.
            emit_config_to.write_text(json.dumps(emit_config(graph, selected)) + "\n")
            typer.echo(f"Wrote {emit_config_to}")

        if dry_run:
            return

        crate = generate_crate(host, selected, graph)
        typer.echo(f"Generated {crate}")
        artifact = build_host(crate, host, builder=builder, target=target, debug=debug)
        size = install(artifact, out)
        typer.echo(f"Wrote {out} ({size / 1e6:.1f} MB)")
    except BakeError as exc:
        typer.echo(typer.style(f"bake: {exc}", fg=typer.colors.RED), err=True)
        raise typer.Exit(1) from exc


def main(argv: list[str] | None = None) -> None:
    """Run the bake CLI on argv, or sys.argv when argv is None."""
    app = typer.Typer(add_completion=False)
    app.command()(bake)
    typer.main.get_command(app)(args=argv, prog_name="dimos bake")
