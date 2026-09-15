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

from collections.abc import Mapping, Sequence
import importlib
from pathlib import Path
from typing import get_type_hints

import typer

from dimos.cli.bake.build import BUILDERS, build_host, install
from dimos.cli.bake.codegen import check_host_name, generate_crate
from dimos.cli.bake.deployment import Deployment, load_deployment
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


def emit_config(
    graph: Graph,
    modules: Sequence[RegisteredModule],
    configs: Mapping[str, Mapping[str, object]] | None = None,
    session: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """The launch config the host embeds; `configs` replace class-default blocks."""
    topics = graph.topics()
    configs = configs or {}
    if unknown := set(configs) - {module.id for module in modules}:
        raise BakeError(f"deployment configures modules this bake does not have: {sorted(unknown)}")
    blob: dict[str, object] = {
        "modules": {
            module.id: {
                "topics": topics[module.id],
                "config": configs.get(module.id, default_config(module)) or None,
            }
            for module in modules
        },
        "graph": graph.fingerprint(),
        "qos": graph.qos(),
        "suppress": list(graph.suppressed_topics()),
    }
    if session is not None:
        blob["session"] = session
    return blob


def deployment_modules(deployment: Deployment, modules: Sequence[str]) -> list[str]:
    """The deployment's module list, refusing a positional list that disagrees."""
    if modules and set(modules) != set(deployment.modules):
        raise BakeError(f"the deployment bakes {sorted(deployment.modules)}, not {sorted(modules)}")
    return list(deployment.modules)


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
    deployment_ref: str = typer.Option(
        None,
        "--deployment",
        help="`pkg.mod:NAME` of a Deployment: its module list, config overrides and "
        "session are embedded in the binary.",
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
        deployment = Deployment(tuple(modules or []))
        if deployment_ref is not None:
            deployment = load_deployment(deployment_ref)
        selected = select_modules(registry, deployment_modules(deployment, modules or []))
        graph = build_graph(
            host,
            selected,
            remaps=dict(parse_remap(r) for r in remap or []),
            suppress=suppress or [],
        )

        typer.echo(render(graph))
        typer.echo("")
        config = emit_config(graph, selected, deployment.configs, deployment.session)

        if dry_run:
            return

        crate = generate_crate(host, selected, graph, config)
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
