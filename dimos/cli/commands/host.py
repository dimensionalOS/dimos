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

from collections.abc import Iterator, Sequence
from contextlib import ExitStack, contextmanager
from importlib.metadata import version as package_version
import json
import threading
from typing import TYPE_CHECKING, Any, NoReturn

from filelock import FileLock, Timeout
import typer

from dimos.hosted.service import HOST_LOCK_PATH, load_host_id
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hosted.daemon import HostDaemon, HostDescriptor
    from dimos.protocol.rpc.zenohrpc import ZenohRPC

host_app = typer.Typer(help="Run and inspect DimOS Hosts", no_args_is_help=True)
DEFAULT_DISCOVERY_TIMEOUT = 2.0
logger = setup_logger()


ConnectOption = typer.Option(
    None, "--connect", "-c", help="Router to join as a client; default: the local Host"
)


def _connect(connect: list[str] | None) -> list[str]:
    """``connect`` as given, else the running local daemon's router."""
    from dimos.hosted.service import local_host

    if connect:
        return connect
    local = local_host()
    if local is None:
        _fail("No local Host is running: `dimos host start`, or pass --connect")
    return [str(local["client_endpoint"])]


@contextmanager
def _host_rpc(connect: list[str] | None) -> Iterator[ZenohRPC]:
    from dimos.protocol.rpc.zenohrpc import ZenohRPC
    from dimos.protocol.service.zenohservice import ZenohSessionPool

    pool = ZenohSessionPool()
    rpc = ZenohRPC(session_pool=pool, mode="client", connect=_connect(connect), multicast=False)
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
        "router_zid": descriptor.router_zid,
        "listen": list(descriptor.listen),
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
        typer.echo(load_host_id())
    except (OSError, ValueError) as exc:
        _fail(str(exc))


def _host_rows(
    found: Sequence[tuple[HostDescriptor, tuple[str, ...]]], revision: str
) -> list[tuple[str, ...]]:
    rows: list[tuple[str, ...]] = []
    for host, endpoints in found:
        host_revision = str(host.versions.get("application_revision", "-"))
        rows.append(
            (
                host.name,
                ",".join(sorted(host.tags)) or "-",
                ",".join(endpoints or host.listen) or "-",
                host_revision[:10],
                "yes" if host_revision == revision else "NO",
                host.state,
                ",".join(host.active_run_ids) or "-",
            )
        )
    return rows


HOST_HEADERS = ("NAME", "TAGS", "ADDRESSES", "REVISION", "MATCH", "STATE", "RUNS")


@host_app.command("ls")
def ls(
    json_output: bool = typer.Option(False, "--json", help="Output descriptors as JSON"),
    timeout: float = typer.Option(
        DEFAULT_DISCOVERY_TIMEOUT, "--timeout", min=0.1, help="Discovery timeout in seconds"
    ),
    connect: list[str] = typer.Option([], "--connect", "-c", help="Extra router to probe"),
    scan: bool = typer.Option(True, help="Scout and run the Go2 LAN probe"),
) -> None:
    """List the dimos Hosts reachable from here: local, seeds, scouted and Go2-probed."""
    from dimos.hosted.daemon import HostConfig, code_revision, split_csv
    from dimos.hosted.discovery import candidates, merge, probe_all
    from dimos.hosted.service import local_host

    config = HostConfig()
    local = local_host()
    seeds = [
        *([str(local["client_endpoint"])] if local else []),
        *connect,
        *split_csv(config.connect),
    ]
    endpoints = candidates(
        seeds,
        scout=scan,
        go2=scan,
        scout_addr=config.scout_addr,
        scout_interface=config.scout_interface,
        timeout=timeout,
    )
    found = merge(probe_all(endpoints, timeout))
    if json_output:
        output = [{**_descriptor_dict(h), "endpoints": list(e)} for h, e in found]
        typer.echo(json.dumps(output, indent=2, sort_keys=True))
        return
    if not found:
        typer.echo(f"No dimos Hosts answered (tried {len(endpoints)} endpoint(s))")
        return
    typer.echo(_format_table(HOST_HEADERS, _host_rows(found, code_revision())))


host_app.command("list", hidden=True)(ls)


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
def doctor(fix: bool = typer.Option(False, "--fix", help="Fix failing checks that can be")) -> None:
    """Check this machine's Host setup, one line per check."""
    import sys

    from dimos.hosted.doctor import run as run_doctors

    ok_mark, bad_mark = ("\u2713", "\u2717") if sys.stdout.isatty() else ("ok", "FAIL")
    results = run_doctors(fix=fix)
    for result in results:
        line = f"{ok_mark if result.ok else bad_mark}  {result.description}"
        if result.fix_note:
            line += f"  [{result.fix_note}]"
        if result.error and not result.ok:
            line += f"  ({result.error})"
        typer.echo(line)
    if not all(result.ok for result in results):
        raise typer.Exit(1)


def _autodiscover(
    daemon: HostDaemon,
    rpc: ZenohRPC,
    interval: float,
    relink: threading.Event,
    stop: threading.Event,
) -> None:
    """Probe for Hosts not yet linked; relink with them while no run is active."""
    from dimos.hosted.discovery import candidates, endpoints_to_link, probe_all

    warned: set[str] = set()
    while not (stop.is_set() or relink.is_set()):
        try:
            probes = probe_all(
                candidates(
                    scout_addr=daemon.scout_addr,
                    scout_interface=daemon.scout_interface,
                    exclude_zid=daemon.router_zid,
                )
            )
            linked = [str(zid) for zid in rpc.session.info.routers_zid()]
            new = endpoints_to_link(
                probes, own_zid=daemon.router_zid, linked=linked, connect=daemon.connect
            )
        except Exception:
            logger.warning("Host autodiscovery round failed", exc_info=True)
            new = []
        if new and not daemon.describe().active_run_ids:
            logger.info("Linking to discovered Hosts", endpoints=new)
            daemon.connect += new
            relink.set()
            return
        for endpoint in set(new) - warned:
            logger.warning("Found a Host; it links once no run is active", endpoint=endpoint)
            warned.add(endpoint)
        stop.wait(interval)


@host_app.command()
def run(
    name: str | None = typer.Option(None, "--name", help="Human-readable Host name"),
    tags: list[str] = typer.Option([], "--tag", "-t", help="Placement tag; repeatable"),
    listen: list[str] = typer.Option([], "--listen", "-l", help="Router listen endpoint"),
    connect: list[str] = typer.Option([], "--connect", "-c", help="Router to always link to"),
    autodiscovery: bool | None = typer.Option(None, help="Find and link other Hosts"),
) -> None:
    """Run this machine's Host in the foreground: its zenoh router and fragment supervisor.

    Unset options come from HOST__<FIELD> in the environment or .env.
    """
    from dimos.hosted.daemon import (
        DEFAULT_LISTEN,
        HOST_PROTOCOL_VERSION,
        HostConfig,
        HostDaemon,
        code_revision,
        free_listen,
        split_csv,
    )
    from dimos.hosted.fragment import FRAGMENT_SCHEMA_VERSION
    from dimos.hosted.service import remove_host_file, write_host_file
    from dimos.hosted.tags import auto_tags

    config = HostConfig()
    if not listen:
        listen = [free_listen(DEFAULT_LISTEN) if config.listen == DEFAULT_LISTEN else config.listen]
        if listen[0] != config.listen:
            logger.warning("Default Host port is taken, listening elsewhere", listen=listen[0])
    all_tags = auto_tags() | set(split_csv(config.tags)) | set(tags)

    with ExitStack() as cleanup:
        try:
            cleanup.enter_context(FileLock(HOST_LOCK_PATH, timeout=0))
        except Timeout:
            _fail("Host service is already running on this machine")
        except OSError as exc:
            _fail(str(exc))

        host_id = load_host_id()
        daemon = HostDaemon(
            host_id,
            name=name or config.name,
            tags=all_tags,
            versions={
                "protocol": HOST_PROTOCOL_VERSION,
                "fragment_schema": FRAGMENT_SCHEMA_VERSION,
                "dimos": package_version("dimos"),
                "application_revision": code_revision(),
            },
            listen=listen,
            connect=connect or split_csv(config.connect),
            autodiscovery=config.autodiscovery if autodiscovery is None else autodiscovery,
            scout_addr=config.scout_addr,
            scout_interface=config.scout_interface,
        )
        cleanup.callback(remove_host_file)
        stop = threading.Event()
        while not stop.is_set():
            relink = threading.Event()
            with daemon.serve() as rpc:
                descriptor = daemon.describe()
                write_host_file(
                    {
                        "host_id": host_id,
                        "name": descriptor.name,
                        "listen": daemon.listen,
                        "client_endpoint": daemon.client_endpoint,
                    }
                )
                typer.echo(
                    f"Host {descriptor.name} ({host_id}) tags={sorted(descriptor.tags)} "
                    f"routing on {','.join(daemon.listen)}"
                    + (f", linked to {','.join(daemon.connect)}" if daemon.connect else "")
                )
                if daemon.autodiscovery:
                    threading.Thread(
                        target=_autodiscover,
                        args=(daemon, rpc, config.discovery_interval, relink, stop),
                        daemon=True,
                    ).start()
                try:
                    while not relink.wait(0.5):
                        pass
                except KeyboardInterrupt:
                    stop.set()


host_app.command("serve", hidden=True)(run)


@host_app.command()
def deploy(
    blueprint: str = typer.Argument(..., help="Blueprint name"),
    local_host: str | None = typer.Option(
        None, "--local-host", help="Host that runs unplaced modules; default: the local Host"
    ),
    connect: list[str] | None = ConnectOption,
    timeout: float = typer.Option(30.0, "--timeout", min=0.1, help="Host discovery timeout"),
) -> None:
    """Place a hosted blueprint on live Hosts and run it until Ctrl-C."""
    from dimos.hosted.deploy import deployed, wait_for_hosts
    from dimos.hosted.service import local_host as running_local_host
    from dimos.robot.get_all_blueprints import get_by_name_or_exit

    app = get_by_name_or_exit(blueprint)
    if local_host is None:
        local = running_local_host()
        if local is None:
            _fail("No local Host is running: `dimos host start`, or pass --local-host")
        local_host = str(local["host_id"])
    named = {p.host for p in app.hosted_placements if isinstance(p.host, str)} | {local_host}
    try:
        with _host_rpc(connect) as rpc:
            tag_sets = [p.tags for p in app.hosted_placements if p.tags]
            descriptors = wait_for_hosts(rpc, named, timeout, tag_sets)
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


@host_app.command()
def install() -> None:
    """Install and enable the dimos-host systemd user unit for this checkout."""
    from dimos.hosted import service

    path = service.install()
    typer.echo(f"Installed {path}; start it with `dimos host start`")
    typer.echo("To start at boot without a login: sudo loginctl enable-linger $USER")


@host_app.command()
def uninstall() -> None:
    """Stop, disable and remove the dimos-host systemd user unit."""
    from dimos.hosted import service

    service.uninstall()
    typer.echo(f"Removed {service.unit_path()}")


@host_app.command()
def start() -> None:
    """Start the Host: via its systemd unit if installed, else detached in the background."""
    from dimos.hosted import service

    typer.echo(service.start())


@host_app.command()
def stop() -> None:
    """Stop the Host started by `dimos host start` or its systemd unit."""
    from dimos.hosted import service

    typer.echo(service.stop())


@host_app.command()
def status() -> None:
    """Whether the local Host runs, and what it advertises when reachable."""
    from dimos.hosted import service
    from dimos.hosted.daemon import code_revision
    from dimos.hosted.discovery import merge, probe

    typer.echo(service.status())
    local = service.local_host()
    if local is None:
        return
    result = probe(str(local["client_endpoint"]), DEFAULT_DISCOVERY_TIMEOUT)
    found = [row for row in merge([result] if result else []) if row[0].host_id == local["host_id"]]
    if found:
        typer.echo(_format_table(HOST_HEADERS, _host_rows(found, code_revision())))
