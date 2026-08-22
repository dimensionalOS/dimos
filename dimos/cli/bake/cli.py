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

import click
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


def session_settings(
    robot_ip: str | None = None,
    mode: str | None = None,
    connect: list[str] | None = None,
    listen: list[str] | None = None,
    interface: str | None = None,
) -> dict[str, object]:
    """The `session` block: these flags over the settings `dimos run` would use."""
    from pydantic import ValidationError

    from dimos.core.global_config import global_config
    from dimos.protocol.service.zenohservice import ZenohConfig, connect_endpoints

    # The session block is zenoh's, so resolve the endpoints as zenoh regardless
    # of the transport this machine defaults to.
    settings: dict[str, object] = {
        "connect": connect_endpoints(
            global_config.robot_ip if robot_ip is None else robot_ip,
            global_config.robot_ips,
            global_config.zenoh_connect if connect is None else ",".join(connect),
        ),
        "listen": listen or [],
    }
    if mode is not None:
        settings["mode"] = mode
    if interface is not None:
        settings["scouting_interface"] = interface
    try:
        return ZenohConfig(**settings).to_wire()
    except ValidationError as exc:
        reasons = ". ".join(e["msg"].removeprefix("Value error, ") for e in exc.errors())
        raise BakeError(f"session: {reasons}") from exc


def emit_config(
    graph: Graph, modules: Sequence[RegisteredModule], session: dict[str, object]
) -> dict[str, object]:
    """The stdin blob the host runs on, session included."""
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
        "session": session,
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
    emit_config_to: str = typer.Option(
        None,
        "--emit-config",
        help="Also write a ready-to-pipe stdin JSON config here. Default: <out>.json.",
    ),
    robot_ip: str = typer.Option(
        None, "--robot-ip", help="Robot the emitted session dials, as `dimos run` takes it."
    ),
    zenoh_connect: list[str] = typer.Option(
        None, "--zenoh-connect", help="Extra locator the session dials. Repeatable."
    ),
    zenoh_listen: list[str] = typer.Option(
        None, "--zenoh-listen", help="Locator the session listens on. Repeatable."
    ),
    zenoh_mode: str = typer.Option(None, "--zenoh-mode", help="peer, client or router."),
    zenoh_interface: str = typer.Option(
        None, "--zenoh-interface", help="Interface multicast scouting binds to, e.g. wlan0."
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
        config_path = None
        if emit_config_to is not None:
            config_path = Path(emit_config_to) if emit_config_to else Path(str(out) + ".json")
        selected = select_modules(registry, modules or [])
        graph = build_graph(
            host,
            selected,
            remaps=dict(parse_remap(r) for r in remap or []),
            suppress=suppress or [],
        )

        typer.echo(render(graph))
        typer.echo("")

        if config_path is not None:
            config_path.parent.mkdir(parents=True, exist_ok=True)
            session = session_settings(
                robot_ip=robot_ip,
                mode=zenoh_mode,
                connect=zenoh_connect or None,
                listen=zenoh_listen,
                interface=zenoh_interface,
            )
            # The host reads its config with a single read_line, so the blob must
            # stay on one line.
            config_path.write_text(json.dumps(emit_config(graph, selected, session)) + "\n")
            typer.echo(f"Wrote {config_path}")

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


def build_command() -> click.Command:
    """Build the bake command, with --emit-config taking an optional path."""
    app = typer.Typer(add_completion=False)
    app.command()(bake)
    command = typer.main.get_command(app)
    for index, param in enumerate(command.params):
        if param.name == "emit_config_to":
            command.params[index] = click.Option(
                ["--emit-config", "emit_config_to"],
                is_flag=False,
                flag_value="",
                type=str,
                help=param.help,
            )
    return command


def main(argv: list[str] | None = None) -> None:
    """Run the bake CLI on argv, or sys.argv when argv is None."""
    build_command()(args=argv, prog_name="dimos bake")
