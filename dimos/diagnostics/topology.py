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

"""Build an evidence graph without inventing network hops from topic names."""

from __future__ import annotations

import json
from typing import Any


def topology(manifests: list[dict[str, Any]], events: list[dict[str, Any]]) -> dict[str, Any]:
    nodes: dict[str, dict[str, Any]] = {}
    edges: dict[str, dict[str, Any]] = {}

    def node(kind: str, *parts: str, **attributes: Any) -> str:
        identity = json.dumps([kind, *parts], separators=(",", ":"))
        nodes[identity] = {"id": identity, "kind": kind, **attributes}
        return identity

    def edge(source: str, target: str, kind: str, **attributes: Any) -> None:
        identity = json.dumps([source, target, kind, attributes.get("generation")])
        edges[identity] = {"source": source, "target": target, "kind": kind, **attributes}

    for manifest in manifests:
        process = str(manifest.get("process", "unknown"))
        host = str(manifest.get("host", "unknown"))
        h = node("host", host, label=host)
        p = node(
            "process",
            process,
            label=process,
            pid=manifest.get("pid"),
            clock_domain=manifest.get("clock_domain"),
        )
        edge(p, h, "runs_on", evidence="manifest")

    for event in events:
        if event.get("event") != "topology.edge":
            continue
        process = str(event.get("process", "unknown"))
        module = str(event.get("module", "direct"))
        endpoint = str(event.get("endpoint", event.get("channel", "unknown")))
        p = json.dumps(["process", process], separators=(",", ":"))
        if p not in nodes:
            node("process", process, label=process)
        m = node("module", process, module, label=module)
        s = node("stream", process, endpoint, label=endpoint)
        # Same topic name does not prove two processes share a transport domain.
        c = node(
            "channel",
            process,
            str(event.get("transport")),
            str(event.get("channel")),
            label=event.get("channel"),
            transport=event.get("transport"),
        )
        edge(m, p, "runs_in", evidence="runtime")
        edge(s, m, "belongs_to", evidence="runtime")
        if event.get("direction") == "publish":
            edge(s, c, "publishes", evidence="observed_call")
        else:
            edge(c, s, "subscribes", evidence="bound_subscription")

    for event in events:
        if event.get("event") == "topology.declared":
            generation = str(event.get("generation", "unknown"))
            m = node("declared_module", generation, str(event["module"]), label=event["module"])
            c = node(
                "declared_channel",
                generation,
                str(event["channel"]),
                label=event["channel"],
                transport=event["transport"],
            )
            edge(
                m if event["direction"] == "out" else c,
                c if event["direction"] == "out" else m,
                "declared_stream",
                evidence="resolved_blueprint",
                port=event["endpoint"],
                generation=generation,
            )
        elif event.get("event") == "network.route":
            a = node("network_endpoint", str(event.get("source")), label=event.get("source"))
            b = node("network_endpoint", str(event.get("target")), label=event.get("target"))
            edge(
                a,
                b,
                "observed_route",
                evidence=event.get("evidence"),
                route=event.get("route"),
                generation=event.get("generation"),
                mono_ns=event.get("mono_ns"),
                clock_domain=event.get("clock_domain"),
                radio=event.get("radio"),
                association=event.get("association"),
                exporter_clock=event.get("exporter_clock"),
                observed_ns=event.get("observed_ns"),
            )
        elif event.get("event") == "network.wifi" and isinstance(event.get("radio"), dict):
            radio = event["radio"]
            host = str(event.get("host", "unknown"))
            h = node("host", host, label=host)
            interface = node(
                "network_endpoint",
                f"{host}/{radio['interface']}",
                label=f"{host}/{radio['interface']}",
            )
            edge(h, interface, "uses_interface", evidence=radio.get("source"))
            if radio.get("bssid"):
                access_point = node("access_point", radio["bssid"], label=f"AP {radio['bssid']}")
                edge(
                    interface,
                    access_point,
                    "wireless_association",
                    evidence=radio.get("source"),
                    mono_ns=event.get("mono_ns"),
                    clock_domain=event.get("clock_domain"),
                    radio=radio,
                    generation=radio["bssid"],
                )
        elif event.get("event") == "network.path_probe":
            measurement = event.get("measurement", {})
            h = node("host", str(event.get("host", "unknown")), label=event.get("host", "unknown"))
            endpoint = node(
                "network_endpoint",
                str(measurement.get("address")),
                label=measurement.get("address"),
            )
            edge(
                h,
                endpoint,
                "round_trip_probe",
                evidence=measurement.get("scope"),
                measurement=measurement,
                mono_ns=event.get("mono_ns"),
                clock_domain=event.get("clock_domain"),
            )
        elif event.get("event") == "network.ice_path" and isinstance(event.get("path"), dict):
            path = event["path"]
            endpoints = []
            for side in ("local", "remote"):
                address = path.get(side, {})
                endpoints.append(
                    node(
                        "ice_endpoint",
                        str(path.get("peerId")),
                        side,
                        str(event.get("generation")),
                        label=address.get("address", "unavailable"),
                        **address,
                    )
                )
            edge(
                endpoints[0],
                endpoints[1],
                "selected_ice_path",
                evidence=event.get("evidence"),
                generation=event.get("generation"),
                mono_ns=event.get("mono_ns"),
                clock_domain=event.get("clock_domain"),
                physical_intermediate_hops="unresolved",
            )

    # The origin publisher can survive several relays. Shared sample identity
    # establishes provenance, not a direct physical or transport hop.
    for event in events:
        ctx = event.get("context")
        if event.get("event") != "transport.received" or not isinstance(ctx, dict):
            continue
        source, target = ctx.get("publisher"), event.get("process")
        if not source or not target or source == target or ctx.get("lineage") == "unknown":
            continue
        a = json.dumps(["process", source], separators=(",", ":"))
        b = json.dumps(["process", target], separators=(",", ":"))
        if a not in nodes:
            node("process", source, label=source, capture="missing")
        if b not in nodes:
            node("process", target, label=target, capture="missing")
        edge(
            a,
            b,
            "sample_provenance",
            evidence="trace_identity",
            physical_route="unresolved",
            direct_sender="not_established",
        )

    return {
        "schema": "dimos.topology/v1",
        "nodes": list(nodes.values()),
        "edges": list(edges.values()),
        "physical_route_coverage": "requires endpoint route, interface and infrastructure evidence",
    }
