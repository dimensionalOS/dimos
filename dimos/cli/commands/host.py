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

"""Commands for running and inspecting DimOS Hosts."""

from __future__ import annotations

from collections.abc import Iterator, Sequence
from contextlib import contextmanager
from dataclasses import asdict
import json
import signal
import threading
import time
from typing import TYPE_CHECKING, Any, NoReturn

import typer

from dimos.hosted.machine import load_host_id
from dimos.hosted.tags import format_tags
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hosted.discovery import Found
    from dimos.hosted.protocol import HostDescriptor
    from dimos.protocol.rpc.zenohrpc import ZenohRPC

host_app = typer.Typer(help="Run and inspect DimOS Hosts", no_args_is_help=True)
DEFAULT_DISCOVERY_TIMEOUT = 1.0
logger = setup_logger()


ConnectOption = typer.Option(
    None, "--connect", "-c", help="Router to join as a client; default: the local Host"
)


def _connect(connect: list[str] | None) -> list[str]:
    """``connect`` as given, else the running local daemon's router."""
    from dimos.hosted.machine import local_host

    if connect:
        return connect
    local = local_host()
    if local is None:
        _fail("No local Host is running: `dimos host start`, or pass --connect")
    return [local.client_endpoint]


@contextmanager
def _host_rpc(connect: list[str] | None) -> Iterator[ZenohRPC]:
    from dimos.hosted.client import root_rpc

    with root_rpc(_connect(connect)) as rpc:
        yield rpc


def _descriptor_dict(descriptor: HostDescriptor) -> dict[str, Any]:
    return {**asdict(descriptor), "active_run_ids": list(descriptor.active_run_ids)}


def _fail(message: str) -> NoReturn:
    typer.echo(f"Error: {message}", err=True)
    raise typer.Exit(1)


@host_app.command("id")
def host_id() -> None:
    """Show this machine's persistent Host ID."""
    try:
        typer.echo(load_host_id())
    except (OSError, ValueError) as exc:
        _fail(str(exc))


def _tags_text(tags: dict[str, str]) -> Any:
    from rich.text import Text

    text = Text()
    for i, (key, value) in enumerate(sorted(tags.items())):
        if i:
            text.append(" ")
        text.append(key, style="yellow")
        if value:
            text.append("=", style="dim")
            text.append(value, style="green")
    return text


def _print_hosts(found: Sequence[Found], revision: str) -> None:
    from rich.console import Console
    from rich.table import Table

    table = Table(box=None, header_style="bold", pad_edge=False)
    for header in ("NAME", "TAGS", "ADDRESSES", "REVISION", "RUNS"):
        table.add_column(header, overflow="fold")
    for row in found:
        host = row.descriptor
        rev_style = "green" if host.revision == revision else "red"
        table.add_row(
            host.name,
            _tags_text(host.tags),
            "\n".join(row.endpoints or host.listen) or "-",
            f"[{rev_style}]{host.revision[:10] or '-'}[/]",
            ",".join(host.active_run_ids) or "-",
        )
    Console().print(table)


@host_app.command("ls")
def ls(
    json_output: bool = typer.Option(False, "--json", help="Output descriptors as JSON"),
    timeout: float = typer.Option(
        DEFAULT_DISCOVERY_TIMEOUT, "--timeout", min=0.1, help="Discovery timeout in seconds"
    ),
    probe: list[str] = typer.Option([], "--probe", "-p", help="Extra router to probe"),
    scout: bool = typer.Option(True, help="Scout the Host multicast group"),
) -> None:
    """List dimos Hosts: the local one's fabric, HOST__CONNECT and --probe seeds, scouted routers."""
    from dimos.hosted.config import HostConfig
    from dimos.hosted.discovery import merge, probe_all, scouted_endpoints
    from dimos.hosted.machine import code_revision, local_host
    from dimos.protocol.service.zenohservice import split_csv

    config = HostConfig()
    local = local_host()
    endpoints = [*([local.client_endpoint] if local else []), *probe, *split_csv(config.connect)]
    if scout:
        endpoints += scouted_endpoints(config, timeout)
    endpoints = list(dict.fromkeys(endpoints))
    found = merge(probe_all(endpoints, timeout))
    if json_output:
        output = [
            {"descriptor": _descriptor_dict(row.descriptor), "endpoints": list(row.endpoints)}
            for row in found
        ]
        typer.echo(json.dumps(output, indent=2, sort_keys=True))
        return
    if not found:
        typer.echo(f"No dimos Hosts answered (tried {len(endpoints)} router(s))")
        return
    _print_hosts(found, code_revision())


@host_app.command()
def describe(
    host: str = typer.Argument(..., help="Host ID or unique exact name"),
    json_output: bool = typer.Option(False, "--json", help="Output descriptor as JSON"),
    timeout: float = typer.Option(
        DEFAULT_DISCOVERY_TIMEOUT,
        "--timeout",
        min=0.1,
        help="Discovery and RPC timeout in seconds",
    ),
    connect: list[str] = ConnectOption,
) -> None:
    """Describe one online Host by ID or unique exact name."""
    from dimos.hosted.client import describe_all

    try:
        with _host_rpc(connect) as rpc:
            matches = [d for d in describe_all(rpc, timeout) if host in (d.host_id, d.name)]
            if not matches:
                raise ValueError(f"No online Host matches {host!r}")
            if len(matches) > 1:
                ids = ", ".join(item.host_id for item in matches)
                raise ValueError(f"Host name {host!r} is ambiguous: {ids}")
            descriptor = matches[0]
    except Exception as exc:
        _fail(str(exc))

    if json_output:
        typer.echo(json.dumps(_descriptor_dict(descriptor), indent=2, sort_keys=True))
        return
    typer.echo(f"Host ID:       {descriptor.host_id}")
    typer.echo(f"Name:          {descriptor.name}")
    typer.echo(f"Tags:          {format_tags(descriptor.tags) or '-'}")
    typer.echo(f"Revision:      {descriptor.revision}")
    typer.echo(f"Runs:          {','.join(descriptor.active_run_ids) or '-'}")


@host_app.command()
def doctor(fix: bool = typer.Option(False, "--fix", help="Fix failing checks that can be")) -> None:
    """Check this machine's Host setup, one line per check."""
    import sys

    from dimos.hosted.doctor import run as run_checks

    tty = sys.stdout.isatty()
    marks = {"ok": "✓", "fail": "✗", "warn": "!"} if tty else {}
    results = run_checks(fix=fix)
    for result in results:
        state = "ok" if result.ok else "warn" if result.warning else "fail"
        line = f"{marks.get(state, state.upper()):<4}  {result.description}"
        if result.fix_note:
            line += f"  [{result.fix_note}]"
        if result.error and not result.ok:
            line += f"  ({result.error})"
        typer.echo(line)
    if not all(result.ok or result.warning for result in results):
        raise typer.Exit(1)


@host_app.command()
def start(
    foreground: bool = typer.Option(
        False, "--foreground", "-f", help="Run the daemon here instead of in the background"
    ),
) -> None:
    """Start this machine's Host: its zenoh router and fragment supervisor.

    In the background via the systemd unit if installed, else detached; --foreground runs
    it here, as the unit does. Settings come from HOST__<FIELD> in the environment or .env.
    """
    from dimos.hosted import machine

    if not foreground:
        typer.echo(machine.start())
        return

    from dimos.hosted.config import HostConfig
    from dimos.hosted.daemon import HostDaemon
    from dimos.hosted.tags import auto_tags

    daemon = HostDaemon(
        HostConfig(),
        host_id=load_host_id(),
        revision=machine.code_revision(),
        tags=auto_tags(),
        sync_checkout=machine.sync_checkout,
    )
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        machine.run_foreground(daemon, stop)
    except RuntimeError as exc:
        _fail(str(exc))


@host_app.command()
def sync(
    timeout: float = typer.Option(
        120.0, "--timeout", min=1.0, help="Seconds to wait for every Host to come back"
    ),
    connect: list[str] = ConnectOption,
) -> None:
    """Check out this machine's commit on every other Host and restart them into it."""
    from dimos.hosted import machine
    from dimos.hosted.client import describe_all, discover

    if machine._git("status", "--porcelain", "--untracked-files=no").strip():
        _fail("Commit your changes first: Hosts sync to a commit")
    commit = machine._git("rev-parse", "HEAD").strip()
    branch = machine._git("rev-parse", "--abbrev-ref", "HEAD").strip()
    branch = "" if branch == "HEAD" else branch
    if not machine._git("branch", "-r", "--contains", commit).strip():
        _fail(f"Push first: {commit[:10]} is not on origin")

    local = machine.local_host()
    pending: dict[str, str] = {}
    with _host_rpc(connect) as rpc:
        for client in discover(rpc, DEFAULT_DISCOVERY_TIMEOUT, rpc_timeout=30.0):
            if local is not None and client.host_id == local.host_id:
                continue
            descriptor = client.describe()
            name = descriptor.name
            if descriptor.revision == commit:
                typer.echo(f"{name}: already on {commit[:10]}")
                continue
            try:
                client.sync(branch, commit)
            except Exception as exc:
                typer.echo(f"{name}: {exc}")
                continue
            typer.echo(f"{name}: syncing to {branch or commit[:10]}")
            pending[client.host_id] = name
        deadline = time.monotonic() + timeout
        while pending and time.monotonic() < deadline:
            time.sleep(2.0)
            for descriptor in describe_all(rpc, DEFAULT_DISCOVERY_TIMEOUT):
                if descriptor.host_id in pending and descriptor.revision == commit:
                    typer.echo(f"{pending.pop(descriptor.host_id)}: on {commit[:10]}")
    if pending:
        _fail(
            f"Not back on {commit[:10]} after {timeout:g}s: {', '.join(sorted(pending.values()))}"
        )


def _write_settings(settings: list[str]) -> None:
    from dimos.hosted.config import HOST_ENV_FILE, write_settings

    if bad := [s for s in settings if "=" not in s]:
        _fail(f"Expected key=value, got {bad}")
    try:
        write_settings(dict(s.partition("=")[::2] for s in settings))
    except ValueError as exc:
        _fail(str(exc))
    typer.echo(f"Wrote {', '.join(settings)} to {HOST_ENV_FILE}")


@host_app.command("config")
def config_command(
    settings: list[str] = typer.Argument(None, help="key=value to set, key= to remove"),
) -> None:
    """Show this Host's settings, or set them in the checkout's .env."""
    from dimos.hosted.config import HostConfig

    if settings:
        _write_settings(settings)
        typer.echo("Apply with `dimos host restart`")
    for name, value in HostConfig().model_dump().items():
        typer.echo(f"{name:<20} {value}")


@host_app.command()
def install(
    config: list[str] = typer.Option(
        [], "--config", help="key=value to set first, as `dimos host config` does; repeatable"
    ),
    start: bool = typer.Option(False, "--start", help="Start the Host once installed"),
) -> None:
    """Install and enable the dimos-host systemd user unit for this checkout."""
    from dimos.hosted import machine

    if config:
        _write_settings(config)
    path = machine.install()
    typer.echo(f"Installed {path}")
    typer.echo(machine.start() if start else "Start it with `dimos host start`")
    typer.echo("To start at boot without a login: sudo loginctl enable-linger $USER")


@host_app.command()
def uninstall() -> None:
    """Stop, disable and remove the dimos-host systemd user unit."""
    from dimos.hosted import machine

    machine.uninstall()
    typer.echo(f"Removed {machine.unit_path()}")


@host_app.command()
def stop() -> None:
    """Stop the Host started by `dimos host start` or its systemd unit."""
    from dimos.hosted import machine

    typer.echo(machine.stop())


@host_app.command()
def restart() -> None:
    """Restart the local Host, e.g. after changing its HOST__* settings."""
    from dimos.hosted import machine

    typer.echo(machine.restart())


@host_app.command()
def status() -> None:
    """Whether the local Host runs, and what it advertises when reachable."""
    from dimos.hosted import machine
    from dimos.hosted.discovery import merge, probe

    typer.echo(machine.status())
    local = machine.local_host()
    if local is None:
        return
    result = probe(local.client_endpoint, DEFAULT_DISCOVERY_TIMEOUT)
    found = [r for r in merge([result] if result else []) if r.descriptor.host_id == local.host_id]
    if found:
        _print_hosts(found, machine.code_revision())
