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

"""Commands for running and inspecting DimOS Host services."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import ExitStack, contextmanager
from importlib.metadata import version as package_version
import json
from pathlib import Path
import socket
import threading
from typing import TYPE_CHECKING, Any, NoReturn
import uuid

from filelock import FileLock, Timeout
import typer

from dimos.constants import STATE_DIR

if TYPE_CHECKING:
    from dimos.hosted.daemon import HostDescriptor
    from dimos.protocol.rpc.zenohrpc import ZenohRPC

host_app = typer.Typer(help="Run and inspect DimOS Hosts", no_args_is_help=True)
HOST_ID_PATH = STATE_DIR / "hosted" / "host_id"
HOST_LOCK_PATH = STATE_DIR / "hosted" / "host.lock"
DEFAULT_DISCOVERY_TIMEOUT = 2.0


def _load_host_id(path: Path) -> str:
    try:
        host_id = path.read_text().strip()
    except FileNotFoundError:
        path.parent.mkdir(parents=True, exist_ok=True)
        host_id = uuid.uuid4().hex
        try:
            with path.open("x") as identity_file:
                identity_file.write(f"{host_id}\n")
        except FileExistsError:
            host_id = path.read_text().strip()
    if not host_id:
        raise ValueError(f"Host identity file is empty: {path}")
    return host_id


# Where host commands find the fabric: this machine's daemon, the local router.
DEFAULT_ROUTER = "tcp/127.0.0.1:7447"
ConnectOption = typer.Option(
    [DEFAULT_ROUTER], "--connect", "-c", help="Router to join as a client; repeatable"
)


@contextmanager
def _host_rpc(connect: list[str]) -> Iterator[ZenohRPC]:
    from dimos.protocol.rpc.zenohrpc import ZenohRPC
    from dimos.protocol.service.zenohservice import ZenohSessionPool

    pool = ZenohSessionPool()
    rpc = ZenohRPC(session_pool=pool, mode="client", connect=connect, multicast=False)
    with ExitStack() as cleanup:
        cleanup.callback(pool.close_all)
        rpc.start()
        cleanup.callback(rpc.stop)
        yield rpc


def _descriptor_dict(descriptor: HostDescriptor) -> dict[str, Any]:
    return {
        "host_id": descriptor.host_id,
        "epoch": descriptor.epoch,
        "name": descriptor.name,
        "tags": sorted(descriptor.tags),
        "versions": descriptor.versions,
        "state": descriptor.state,
        "active_run_ids": list(descriptor.active_run_ids),
    }


def _format_table(headers: tuple[str, ...], rows: list[tuple[str, ...]]) -> str:
    widths = [
        max(len(header), *(len(row[index]) for row in rows)) for index, header in enumerate(headers)
    ]

    def format_row(row: tuple[str, ...]) -> str:
        return "  ".join(value.ljust(widths[index]) for index, value in enumerate(row)).rstrip()

    return "\n".join(
        (
            format_row(headers),
            format_row(tuple("-" * width for width in widths)),
            *(format_row(row) for row in rows),
        )
    )


def _fail(message: str) -> NoReturn:
    typer.echo(f"Error: {message}", err=True)
    raise typer.Exit(1)


@host_app.command("id")
def host_id() -> None:
    """Show this machine's persistent Host ID."""
    try:
        typer.echo(_load_host_id(HOST_ID_PATH))
    except (OSError, ValueError) as exc:
        _fail(str(exc))


@host_app.command("list")
def list_hosts(
    json_output: bool = typer.Option(False, "--json", help="Output descriptors as JSON"),
    timeout: float = typer.Option(
        DEFAULT_DISCOVERY_TIMEOUT,
        "--timeout",
        min=0.1,
        help="Discovery and RPC timeout in seconds",
    ),
    connect: list[str] = ConnectOption,
) -> None:
    """List Hosts currently visible through Zenoh liveliness."""
    from dimos.hosted.client import discover_host_ids, get_host_descriptor

    try:
        with _host_rpc(connect) as rpc:
            host_ids = discover_host_ids(rpc, timeout)
            descriptors: list[HostDescriptor | dict[str, str]] = []
            for discovered_id in host_ids:
                try:
                    descriptors.append(get_host_descriptor(rpc, discovered_id, timeout))
                except Exception as exc:
                    descriptors.append({"host_id": discovered_id, "error": str(exc)})
    except Exception as exc:
        _fail(str(exc))

    if json_output:
        output = [
            item if isinstance(item, dict) else _descriptor_dict(item) for item in descriptors
        ]
        typer.echo(json.dumps(output, indent=2, sort_keys=True))
        return
    if not descriptors:
        typer.echo("No online Hosts found")
        return

    rows: list[tuple[str, ...]] = []
    for item in descriptors:
        if isinstance(item, dict):
            rows.append((item["host_id"], "-", "-", "unreachable", "-", "-"))
            continue
        rows.append(
            (
                item.host_id,
                item.name,
                ",".join(sorted(item.tags)) or "-",
                item.state,
                ",".join(item.active_run_ids) or "-",
                str(item.versions.get("dimos", "-")),
            )
        )
    typer.echo(_format_table(("ID", "NAME", "TAGS", "STATE", "RUNS", "DIMOS"), rows))


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
    from dimos.hosted.client import discover_host_ids, get_host_descriptor

    try:
        with _host_rpc(connect) as rpc:
            host_ids = discover_host_ids(rpc, timeout)
            if host in host_ids:
                descriptor = get_host_descriptor(rpc, host, timeout)
            else:
                matches = []
                for discovered_id in host_ids:
                    item = get_host_descriptor(rpc, discovered_id, timeout)
                    if item.name == host:
                        matches.append(item)
                if not matches:
                    raise ValueError(f"No online Host matches {host!r}")
                if len(matches) > 1:
                    ids = ", ".join(item.host_id for item in matches)
                    raise ValueError(f"Host name {host!r} is ambiguous: {ids}")
                descriptor = matches[0]
    except Exception as exc:
        _fail(str(exc))

    data = _descriptor_dict(descriptor)
    if json_output:
        typer.echo(json.dumps(data, indent=2, sort_keys=True))
        return
    typer.echo(f"Host ID:       {descriptor.host_id}")
    typer.echo(f"Epoch:         {descriptor.epoch}")
    typer.echo(f"Name:          {descriptor.name}")
    typer.echo(f"Tags:          {','.join(sorted(descriptor.tags)) or '-'}")
    typer.echo(f"State:         {descriptor.state}")
    typer.echo(f"Active run IDs: {','.join(descriptor.active_run_ids) or '-'}")
    typer.echo("Versions:")
    for name, value in sorted(descriptor.versions.items()):
        typer.echo(f"  {name}: {value}")


@host_app.command()
def doctor(connect: list[str] = ConnectOption) -> None:
    """Check the local Host identity, the router connection, and the code revision."""
    from dimos.hosted.daemon import code_revision

    def check_connection() -> str:
        with _host_rpc(connect) as rpc:
            link_count = len(list(rpc.session.info.links()))
        return f"joined {','.join(connect)} ({link_count} link(s))"

    checks: list[tuple[str, Callable[[], str]]] = [
        ("Host ID", lambda: _load_host_id(HOST_ID_PATH)),
        ("Zenoh connection", check_connection),
        ("DimOS version", lambda: package_version("dimos")),
        ("Code revision", code_revision),
    ]
    failures = 0
    for name, check in checks:
        try:
            detail = check()
        except Exception as exc:
            failures += 1
            typer.echo(f"FAIL  {name}: {exc}", err=True)
        else:
            typer.echo(f"PASS  {name}: {detail}")
    if failures:
        typer.echo(f"Host doctor found {failures} problem(s).", err=True)
        raise typer.Exit(1)
    typer.echo("Host doctor passed.")


@host_app.command()
def serve(
    name: str | None = typer.Option(None, "--name", help="Human-readable Host name"),
    tags: list[str] = typer.Option([], "--tag", "-t", help="Placement tag; repeatable"),
    listen: list[str] = typer.Option(
        ["tcp/0.0.0.0:7447"], "--listen", "-l", help="Router listen endpoint; repeatable"
    ),
    connect: list[str] = typer.Option(
        [], "--connect", "-c", help="Another Host's router to link to; repeatable"
    ),
) -> None:
    """Serve this machine's Host: its zenoh router plus the fragment supervisor."""
    from dimos.hosted.daemon import HOST_PROTOCOL_VERSION, HostDaemon, code_revision
    from dimos.hosted.fragment import FRAGMENT_SCHEMA_VERSION

    with ExitStack() as cleanup:
        try:
            cleanup.enter_context(FileLock(HOST_LOCK_PATH, timeout=0))
        except Timeout:
            _fail("Host service is already running on this machine")
        except OSError as exc:
            _fail(str(exc))

        host_id = _load_host_id(HOST_ID_PATH)
        daemon = HostDaemon(
            host_id,
            name=name,
            tags=set(tags),
            versions={
                "protocol": HOST_PROTOCOL_VERSION,
                "fragment_schema": FRAGMENT_SCHEMA_VERSION,
                "dimos": package_version("dimos"),
                "application_revision": code_revision(),
            },
            listen=listen,
            connect=connect,
        )
        cleanup.enter_context(daemon.serve())
        descriptor = daemon.describe()
        typer.echo(
            f"Host {descriptor.name} ({host_id}) is available, routing on {','.join(listen)}"
            + (f", linked to {','.join(connect)}" if connect else "")
        )
        try:
            threading.Event().wait()
        except KeyboardInterrupt:
            pass


@host_app.command()
def deploy(
    blueprint: str = typer.Argument(..., help="Blueprint name"),
    local_host: str = typer.Option(
        socket.gethostname(), "--local-host", help="Host that runs unplaced modules"
    ),
    connect: list[str] = ConnectOption,
    timeout: float = typer.Option(30.0, "--timeout", min=0.1, help="Host discovery timeout"),
) -> None:
    """Place a hosted blueprint on live Hosts and run it until Ctrl-C."""
    from dimos.hosted.deploy import deployed, wait_for_hosts
    from dimos.robot.get_all_blueprints import get_by_name_or_exit

    app = get_by_name_or_exit(blueprint)
    named = {p.host for p in app.hosted_placements if isinstance(p.host, str)} | {local_host}
    try:
        with _host_rpc(connect) as rpc:
            descriptors = wait_for_hosts(rpc, named, timeout)
            with deployed(
                app, rpc, local_host=local_host, application_name=blueprint, descriptors=descriptors
            ) as placement:
                for module, host in sorted(placement.items()):
                    typer.echo(f"{host:>16}  {module}")
                typer.echo("Running; Ctrl-C stops every fragment")
                try:
                    threading.Event().wait()
                except KeyboardInterrupt:
                    pass
    except (RuntimeError, TimeoutError, ValueError) as exc:
        _fail(str(exc))
