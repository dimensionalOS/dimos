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

"""Read-only route/interface collection and low-rate probes independent of operator traffic."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import threading
import time
from typing import Any

import requests

from dimos.diagnostics.host import infrastructure_snapshot, snapshot
from dimos.diagnostics.infrastructure import ap_snapshot
from dimos.diagnostics.probe import exchange
from dimos.diagnostics.runtime import Runtime
from dimos.hardware.sensors.wifi.probes import probe_paths
from dimos.hardware.sensors.wifi.radio import RadioHistory, darwin_radio, linux_radio


def record_snapshot(runtime: Runtime, data: dict[str, Any]) -> None:
    runtime.event("network.capabilities", capabilities=data["capabilities"], host=data["host"])
    runtime.event("host.resources", cpu_times=data["cpu_times"], memory=data["memory"])
    for interface, counters in data["interfaces"].items():
        runtime.event("network.interface", interface=interface, counters=counters)
    for probe in data["probes"]:
        runtime.event(
            "network.endpoint_observation",
            command=probe["command"],
            status=probe.get("status"),
            returncode=probe.get("returncode"),
            output=probe.get("stdout", "")[:2048],
            error=probe.get("stderr", "")[:512],
        )
        command = probe["command"][0]
        if command not in ("ip", "route") or probe.get("returncode") != 0:
            continue
        raw = probe.get("stdout", "")
        routes = []
        if command == "ip":
            try:
                routes = [
                    {
                        "gateway": r.get("gateway"),
                        "interface": r.get("dev"),
                        "destination": r.get("dst"),
                    }
                    for r in json.loads(raw)
                ]
            except (ValueError, TypeError, KeyError):
                continue
        else:
            fields = dict(line.strip().split(":", 1) for line in raw.splitlines() if ":" in line)
            routes = [
                {
                    "gateway": fields.get("gateway", "").strip(),
                    "interface": fields.get("interface", "").strip(),
                    "destination": "default",
                }
            ]
        generation = hashlib.sha256(json.dumps(routes, sort_keys=True).encode()).hexdigest()[:16]
        for route in routes:
            if route["gateway"]:
                runtime.event(
                    "network.route",
                    source=f"{data['host']}/{route['interface']}",
                    target=route["gateway"],
                    route=route,
                    generation=generation,
                    evidence="OS route table; layer-2/AP/backhaul unresolved",
                )


def monitor(
    directory: Path,
    run: str,
    seconds: float,
    interval: float,
    interface: str | None = None,
    probe_url: str | None = None,
    infrastructure: Path | None = None,
    stop: threading.Event | None = None,
    peers: tuple[str, ...] = (),
    ap_host: str | None = None,
    ap_interface: str | None = None,
) -> None:
    if not 0 < seconds <= 86400 or not 1 <= interval <= 60:
        raise ValueError("monitor duration must be 0..86400s and interval 1..60s")
    stop = stop or threading.Event()
    runtime = Runtime(run, directory / run)
    history = RadioHistory()
    ap_histories: dict[str, RadioHistory] = {}
    try:
        with requests.Session() as session:
            end = time.monotonic() + seconds
            while not stop.is_set() and time.monotonic() < end:
                began = time.monotonic()
                data = snapshot(interface)
                record_snapshot(runtime, data)
                if interface:
                    radio = (linux_radio if data["system"] == "Linux" else darwin_radio)(
                        data["probes"], interface
                    )
                    observation = history.observe(radio, data["mono_ns"])
                    runtime.event("network.wifi", radio=observation, host=data["host"])
                    if observation["association_changed"]:
                        runtime.collector.trigger("wifi_association_changed")
                targets = {peer: "peer" for peer in peers}
                for probe in data["probes"]:
                    if probe.get("returncode") != 0:
                        continue
                    if probe["command"][0] == "ip":
                        for route in json.loads(probe.get("stdout", "[]")):
                            if (
                                route.get("dst") == "default"
                                and route.get("gateway")
                                and (not interface or route.get("dev") == interface)
                            ):
                                targets[route["gateway"]] = "default_gateway"
                    elif probe["command"][0] == "route":
                        fields = dict(
                            line.strip().split(":", 1)
                            for line in probe.get("stdout", "").splitlines()
                            if ":" in line
                        )
                        if fields.get("gateway") and (
                            not interface or fields.get("interface", "").strip() == interface
                        ):
                            targets[fields["gateway"].strip()] = "default_gateway"
                for measurement in probe_paths(targets):
                    runtime.event("network.path_probe", measurement=measurement, host=data["host"])
                if ap_host and ap_interface:
                    ap = ap_snapshot(ap_host, ap_interface)
                    runtime.event(
                        "network.infrastructure_status",
                        source=ap_host,
                        status=ap["status"],
                        coverage=ap.get("coverage"),
                        reason=ap.get("reason"),
                    )
                    for link in ap.get("links", []):
                        history_key = ap["clock_domain"] + link["source"] + link["target"]
                        if history_key not in ap_histories and len(ap_histories) >= 128:
                            ap_histories.pop(next(iter(ap_histories)))
                        measured = ap_histories.setdefault(history_key, RadioHistory()).observe(
                            link["radio"], link["observed_ns"]
                        )
                        runtime.event(
                            "network.ap_radio",
                            host=ap["source"],
                            radio={
                                **measured,
                                "interface": f"{ap_interface}/{link['client_mac']}",
                                "bssid": link["association"],
                                "source": link["evidence"],
                            },
                        )
                        runtime.event(
                            "network.route",
                            **{k: link[k] for k in ("source", "target", "association", "evidence")},
                            generation=history_key,
                            radio=measured,
                            exporter_clock=ap["clock_domain"],
                            observed_ns=link["observed_ns"],
                        )
                if probe_url:
                    observation = exchange(session, probe_url)
                    runtime.collector.emit(
                        {"schema": "dimos.trace/v1", "process": runtime.process, **observation}
                    )
                if infrastructure:
                    try:
                        data = infrastructure_snapshot(infrastructure)
                        for link in data["links"]:
                            runtime.event(
                                "network.route",
                                **{
                                    k: link.get(k)
                                    for k in (
                                        "source",
                                        "target",
                                        "evidence",
                                        "association",
                                        "freshness_at_export",
                                        "local_import_age_ns",
                                    )
                                },
                                generation=data["generation"],
                                exporter_clock=data["clock_domain"],
                            )
                    except (OSError, ValueError, TypeError, KeyError) as error:
                        runtime.event(
                            "network.infrastructure_unavailable", reason=type(error).__name__
                        )
                elapsed = time.monotonic() - began
                runtime.event(
                    "network.collector_cycle",
                    duration_ns=int(elapsed * 1e9),
                    schedule_overrun=elapsed > interval,
                )
                stop.wait(max(0, min(end - time.monotonic(), interval - elapsed)))
    finally:
        runtime.close()
