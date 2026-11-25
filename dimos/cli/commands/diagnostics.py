# Copyright 2025-2026 Dimensional Inc.
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

"""Capture and inspect diagnostics without publishing robot commands."""

from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import subprocess
import time
from typing import Annotated
from uuid import uuid4

import requests
import typer

from dimos.diagnostics.analysis import analyze, load_bundle, perfetto
from dimos.diagnostics.capabilities import capabilities as describe_capabilities
from dimos.diagnostics.conformance import run_backend
from dimos.diagnostics.evaluation import compare_overhead, evaluate as evaluate_budgets, sample_ages
from dimos.diagnostics.host import snapshot
from dimos.diagnostics.network import monitor
from dimos.diagnostics.otlp import otlp, send as send_otlp
from dimos.diagnostics.probe import exchange
from dimos.diagnostics.report import write_report
from dimos.diagnostics.ros_trace import import_trace
from dimos.diagnostics.socket_probe import echo as serve_echo, socket_probe as measure_socket
from dimos.diagnostics.store import DiagnosticsStore
from dimos.diagnostics.topology import topology as build_topology

app = typer.Typer(help="Bounded runtime tracing and offline incident inspection.")


@app.command()
def capabilities() -> None:
    typer.echo(json.dumps(describe_capabilities(), indent=2))


@app.command()
def host(interface: str | None = None) -> None:
    typer.echo(json.dumps(snapshot(interface), indent=2))


@app.command()
def inspect(bundle: Path) -> None:
    manifests, events = load_bundle(bundle)
    typer.echo(
        json.dumps({**analyze(manifests, events), "sample_ages": sample_ages(events)}, indent=2)
    )


@app.command()
def topology(bundle: Path) -> None:
    manifests, events = load_bundle(bundle)
    typer.echo(json.dumps(build_topology(manifests, events), indent=2))


@app.command()
def evaluate(bundle: Path, profile: Path) -> None:
    manifests, events = load_bundle(bundle)
    result = evaluate_budgets(manifests, events, json.loads(profile.read_text()))
    typer.echo(json.dumps(result, indent=2))
    raise typer.Exit(0 if result["status"] == "pass" else 1 if result["status"] == "fail" else 2)


@app.command()
def overhead(baseline: Path, traced: Path, output: Path) -> None:
    """Compare off/trace results from the complete synthetic application fixture."""
    result = compare_overhead(json.loads(baseline.read_text()), json.loads(traced.read_text()))
    output.write_text(json.dumps(result, indent=2))
    typer.echo(str(output))
    states = set(result["checks"].values())
    raise typer.Exit(1 if "fail" in states else 2 if "inconclusive" in states else 0)


@app.command()
def report(bundle: Path, output: Path, profile: Path | None = None) -> None:
    """Write a standalone HTML report of observed routes, processing and causal samples."""
    manifests, events = load_bundle(bundle)
    write_report(output, manifests, events, json.loads(profile.read_text()) if profile else None)
    typer.echo(str(output))


@app.command()
def probe(
    url: str, output: Path, count: int = 20, interval: float = 0.5, interface: str | None = None
) -> None:
    """Probe an explicit /diagnostics/clock endpoint; never run a saturation test."""
    if not 1 <= count <= 10000 or interval < 0.1:
        raise typer.BadParameter("count must be 1..10000 and interval at least 0.1s")
    with requests.Session() as session, output.open("x") as file:
        file.write(
            json.dumps(
                {
                    "kind": "manifest",
                    "schema": "dimos.incident/v1",
                    "scope": "HTTP application probes",
                    "host_snapshot": snapshot(interface),
                }
            )
            + "\n"
        )
        for i in range(count):
            file.write(json.dumps(exchange(session, url)) + "\n")
            file.flush()
            if i + 1 < count:
                time.sleep(interval)
    typer.echo(str(output))


@app.command()
def export(
    bundle: Path,
    output: Path,
    clock_domain: str | None = None,
    format: str = "perfetto",
    otlp_endpoint: str | None = None,
) -> None:
    manifests, events = load_bundle(bundle)
    if format not in ("perfetto", "otlp-json") or (otlp_endpoint and format != "otlp-json"):
        raise typer.BadParameter("select perfetto or otlp-json; endpoint requires otlp-json")
    document = perfetto(events, clock_domain) if format == "perfetto" else otlp(manifests, events)
    output.write_text(json.dumps(document))
    if otlp_endpoint:
        with requests.Session() as session:
            typer.echo(json.dumps(send_otlp(session, otlp_endpoint, document)))
    typer.echo(str(output))


@app.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def capture(
    ctx: typer.Context,
    directory: Annotated[Path, typer.Option()],
    seconds: float = 60,
    run_id: str = "",
    mode: str = "trace",
) -> None:
    """Run a supplied command under diagnostics, bounded by a wall-time timeout.

    Example: dimos diagnostics capture --directory ./capture -- python demo.py
    Only the launched child is stopped at the deadline; this never attaches to a robot.
    """
    if not ctx.args or seconds <= 0 or mode not in ("trace", "metrics"):
        raise typer.BadParameter("supply a command, positive seconds, and trace/metrics mode")
    run_id = run_id or uuid4().hex
    store = DiagnosticsStore(directory)
    store.run(run_id)  # Validate before starting the child.
    env = {
        **os.environ,
        "DIMOS_DIAGNOSTICS": mode,
        "DIMOS_DIAGNOSTICS_RUN": run_id,
        "DIMOS_DIAGNOSTICS_DIR": str(directory.resolve()),
    }
    child = subprocess.Popen(ctx.args, env=env)
    try:
        code = child.wait(timeout=seconds)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        # Flush the local rings before signalling a child that may not run atexit.
        store.request(run_id, "stop")
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if all(s.get("state") == "stopped" for s in store.statuses(run_id)):
                break
            time.sleep(0.05)
        child.terminate()
        try:
            code = child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            code = child.wait()
    typer.echo(json.dumps({"run_id": run_id, "directory": str(directory), "exit_code": code}))


@app.command()
def status(directory: Path, run_id: str) -> None:
    typer.echo(json.dumps(DiagnosticsStore(directory).statuses(run_id), indent=2))


@app.command()
def mark(directory: Path, run_id: str, reason: str = "manual") -> None:
    typer.echo(DiagnosticsStore(directory).request(run_id, "mark", reason))


@app.command()
def stop(directory: Path, run_id: str) -> None:
    """Stop diagnostics collection only; never stop the robot or blueprint."""
    typer.echo(DiagnosticsStore(directory).request(run_id, "stop"))


@app.command()
def conformance(output: Path, backend: str = "all", provider_config: Path | None = None) -> None:
    """Run isolated transport peers; optional hosted providers require an explicit test config."""
    configured = json.loads(provider_config.read_text()) if provider_config else {}
    names = (
        ["zenoh", "lcm", "ros", "dds", "webrtc", "cloudflare"] if backend == "all" else [backend]
    )
    results = []
    for name in names:
        try:
            results.append(run_backend(name, configured.get(name)))
        except Exception as error:
            results.append({"backend": name, "status": "failed", "reason": type(error).__name__})
    output.write_text(json.dumps({"schema": "dimos.conformance/v1", "results": results}, indent=2))
    typer.echo(str(output))
    raise typer.Exit(
        1
        if any(r["status"] == "failed" for r in results)
        else 2
        if any(r["status"] == "skipped" for r in results)
        else 0
    )


@app.command()
def network(
    directory: Path,
    run_id: str,
    seconds: float = 3600,
    interval: float = 2,
    interface: str | None = None,
    probe_url: str | None = None,
    infrastructure: Path | None = None,
    peer: Annotated[
        list[str] | None,
        typer.Option(
            help="Peer IPs to probe alongside the selected interface's default gateway (maximum three)."
        ),
    ] = None,
    ap_host: str | None = None,
    ap_interface: str | None = None,
) -> None:
    """Collect endpoint routes/counters and optional infrastructure evidence; no robot subscriptions."""
    try:
        if peer and len(peer) > 3:
            raise ValueError("at most three peer addresses may be probed")
        if bool(ap_host) != bool(ap_interface):
            raise ValueError("AP collection requires both --ap-host and --ap-interface")
        monitor(
            directory,
            run_id,
            seconds,
            interval,
            interface,
            probe_url,
            infrastructure,
            peers=tuple(peer or ()),
            ap_host=ap_host,
            ap_interface=ap_interface,
        )
    except ValueError as error:
        raise typer.BadParameter(str(error)) from error


@app.command()
def socket_probe(address: str, port: int, output: Path) -> None:
    """Probe an explicitly started diagnostics echo endpoint once."""
    with output.open("x") as file:
        file.write(json.dumps(measure_socket(address, port)) + "\n")
    typer.echo(str(output))


@app.command()
def socket_echo(address: str, seconds: float = 60, port: int = 0) -> None:
    """Bind an explicitly chosen test interface; port 0 chooses a free ephemeral port."""
    if not 0 < seconds <= 3600 or not 0 <= port < 65536:
        raise typer.BadParameter("invalid duration or port")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((address, port))
        typer.echo(
            json.dumps(
                {
                    "address": sock.getsockname()[0],
                    "port": sock.getsockname()[1],
                    "seconds": seconds,
                }
            )
        )
        serve_echo(sock, seconds)


@app.command("import-ros-trace")
def import_ros_trace(source: Path, output: Path, run_id: str) -> None:
    """Import an existing ROS 2 CTF trace; requires Babeltrace 2 Python bindings."""
    typer.echo(json.dumps(import_trace(source, output, run_id)))
