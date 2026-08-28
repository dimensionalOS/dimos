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
import fcntl
from importlib.metadata import version as package_version
import json
from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any, Never, TextIO
import uuid

import typer

from dimos.constants import STATE_DIR
from dimos.core.global_config import global_config

if TYPE_CHECKING:
    from dimos.hosted.daemon import HostDescriptor
    from dimos.protocol.rpc.zenohrpc import ZenohRPC

host_app = typer.Typer(help="Run and inspect DimOS Hosts", no_args_is_help=True)
HOST_ID_PATH = STATE_DIR / "hosted" / "host_id"
HOST_LOCK_PATH = STATE_DIR / "hosted" / "host.lock"
DISCOVERY_KEY = "dimos/hosts/*/live"
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


def _acquire_host_lock(path: Path) -> TextIO:
    path.parent.mkdir(parents=True, exist_ok=True)
    lock_file = path.open("a+")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock_file.close()
        raise RuntimeError("Host service is already running on this machine") from exc
    return lock_file


def _zenoh_kwargs() -> dict[str, Any]:
    return {
        "mode": global_config.zenoh_mode,
        "connect": [
            item.strip() for item in global_config.zenoh_connect.split(",") if item.strip()
        ],
        "scouting": global_config.zenoh_scouting,
        "scouting_interface": global_config.zenoh_interface,
        "multicast": global_config.zenoh_multicast,
        "gossip": global_config.zenoh_gossip,
        "connect_timeout": global_config.zenoh_connect_timeout,
    }


@contextmanager
def _host_rpc() -> Iterator[ZenohRPC]:
    from dimos.protocol.rpc.zenohrpc import ZenohRPC
    from dimos.protocol.service.zenohservice import ZenohSessionPool

    pool = ZenohSessionPool()
    rpc = ZenohRPC(session_pool=pool, **_zenoh_kwargs())
    with ExitStack() as cleanup:
        cleanup.callback(pool.close_all)
        rpc.start()
        cleanup.callback(rpc.stop)
        yield rpc


def _discover_host_ids(rpc: ZenohRPC, timeout: float) -> list[str]:
    replies = rpc.session.liveliness().get(DISCOVERY_KEY, timeout=timeout)
    host_ids: set[str] = set()
    for reply in replies:
        sample = reply.ok
        if sample is None:
            continue
        key = str(sample.key_expr)
        parts = key.split("/")
        if len(parts) == 4 and parts[:2] == ["dimos", "hosts"] and parts[3] == "live":
            host_ids.add(parts[2])
    return sorted(host_ids)


def _get_descriptor(rpc: ZenohRPC, host_id: str, timeout: float) -> HostDescriptor:
    from dimos.hosted.daemon import HOST_CONTROL_RPC_NAME, HostDescriptor

    control_name = HOST_CONTROL_RPC_NAME.format(host_id=host_id)
    result, unsubscribe = rpc.call_sync(
        f"{control_name}/describe",
        ([], {}),
        rpc_timeout=timeout,
    )
    try:
        if not isinstance(result, HostDescriptor):
            raise TypeError(f"Host {host_id} returned an invalid descriptor")
        return result
    finally:
        unsubscribe()


def _descriptor_dict(descriptor: HostDescriptor) -> dict[str, Any]:
    return {
        "host_id": descriptor.host_id,
        "epoch": descriptor.epoch,
        "name": descriptor.name,
        "tags": sorted(descriptor.tags),
        "versions": descriptor.versions,
        "state": descriptor.state,
        "active_run_id": descriptor.active_run_id,
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


def _fail(message: str) -> Never:
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
) -> None:
    """List Hosts currently visible through Zenoh liveliness."""
    try:
        with _host_rpc() as rpc:
            host_ids = _discover_host_ids(rpc, timeout)
            descriptors: list[HostDescriptor | dict[str, str]] = []
            for discovered_id in host_ids:
                try:
                    descriptors.append(_get_descriptor(rpc, discovered_id, timeout))
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
                item.active_run_id or "-",
                str(item.versions.get("dimos", "-")),
            )
        )
    typer.echo(_format_table(("ID", "NAME", "TAGS", "STATE", "RUN", "DIMOS"), rows))


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
) -> None:
    """Describe one online Host by ID or unique exact name."""
    try:
        with _host_rpc() as rpc:
            host_ids = _discover_host_ids(rpc, timeout)
            if host in host_ids:
                descriptor = _get_descriptor(rpc, host, timeout)
            else:
                matches = []
                for discovered_id in host_ids:
                    item = _get_descriptor(rpc, discovered_id, timeout)
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
    typer.echo(f"Active run ID: {descriptor.active_run_id or '-'}")
    typer.echo("Versions:")
    for name, value in sorted(descriptor.versions.items()):
        typer.echo(f"  {name}: {value}")


def _zenoh_config_detail() -> str:
    from dimos.protocol.service.zenohservice import ZenohConfig

    config = ZenohConfig(**_zenoh_kwargs())
    endpoints = ",".join(config.connect) or "none"
    return (
        f"mode={config.mode}, connect={endpoints}, "
        f"scouting={config.scouting}, multicast={config.multicast}"
    )


def _check_zenoh_connection() -> str:
    with _host_rpc() as rpc:
        link_count = len(list(rpc.session.info.links()))
    return f"session opened ({link_count} link(s))"


@host_app.command()
def doctor() -> None:
    """Check the local Host identity, Zenoh configuration, connection, and version."""
    checks: list[tuple[str, Callable[[], str]]] = [
        ("Host ID", lambda: _load_host_id(HOST_ID_PATH)),
        ("Zenoh config", _zenoh_config_detail),
        ("Zenoh connection", _check_zenoh_connection),
        ("DimOS version", lambda: package_version("dimos")),
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
) -> None:
    """Serve one Host over the configured Zenoh fabric."""
    from dimos.hosted.daemon import (
        FRAGMENT_SCHEMA_VERSION,
        HOST_CONTROL_RPC_NAME,
        HOST_LIVELINESS_KEY,
        HOST_PROTOCOL_VERSION,
        HostDaemon,
    )
    from dimos.protocol.rpc.zenohrpc import ZenohRPC
    from dimos.protocol.service.zenohservice import ZenohSessionPool

    try:
        lock_file = _acquire_host_lock(HOST_LOCK_PATH)
    except (OSError, RuntimeError) as exc:
        _fail(str(exc))

    with lock_file:
        host_id = _load_host_id(HOST_ID_PATH)
        daemon = HostDaemon(
            host_id,
            name=name,
            tags=set(tags),
            versions={
                "protocol": HOST_PROTOCOL_VERSION,
                "fragment_schema": FRAGMENT_SCHEMA_VERSION,
                "dimos": package_version("dimos"),
            },
        )
        pool = ZenohSessionPool()
        rpc = ZenohRPC(session_pool=pool, **_zenoh_kwargs())
        with ExitStack() as cleanup:
            cleanup.callback(pool.close_all)
            cleanup.callback(daemon.shutdown)
            rpc.start()
            cleanup.callback(rpc.stop)
            control_name = HOST_CONTROL_RPC_NAME.format(host_id=host_id)
            rpc.serve_rpc(daemon.describe, f"{control_name}/describe")  # type: ignore[arg-type]
            rpc.serve_rpc(daemon.start, f"{control_name}/start")  # type: ignore[arg-type]
            rpc.serve_rpc(daemon.status, f"{control_name}/status")  # type: ignore[arg-type]
            rpc.serve_rpc(daemon.stop, f"{control_name}/stop")  # type: ignore[arg-type]
            token = rpc.session.liveliness().declare_token(
                HOST_LIVELINESS_KEY.format(host_id=host_id)
            )
            cleanup.callback(token.undeclare)
            descriptor = daemon.describe()
            typer.echo(f"Host {descriptor.name} ({host_id}) is available")
            try:
                threading.Event().wait()
            except KeyboardInterrupt:
                pass
