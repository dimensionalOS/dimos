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

"""Self-contained evidence report from any robot's ordinary diagnostic bundle."""

from __future__ import annotations

from collections import defaultdict
import json
import math
from pathlib import Path
from typing import Any

from dimos.diagnostics.analysis import analyze
from dimos.diagnostics.evaluation import evaluate, matched_paths
from dimos.diagnostics.model import SCHEMA, TraceContext
from dimos.diagnostics.topology import topology


def network_summary(events: list[dict[str, Any]]) -> dict[str, Any]:
    radios: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    probes: dict[tuple[str, str, str], list[dict[str, Any]]] = defaultdict(list)
    for event in events:
        host = str(event.get("host", event.get("process", "unknown")))
        if event.get("event") in ("network.wifi", "network.ap_radio"):
            radio = event["radio"]
            radios[(host, radio["interface"])].append(radio)
        elif event.get("event") == "network.path_probe":
            probe = event["measurement"]
            probes[(host, probe["address"], probe["role"])].append(probe)
    paths = []
    for (host, target, role), samples in probes.items():
        rtt = sorted(s["rtt_ms"] for s in samples if s.get("status") == "ok")
        paths.append(
            {
                "host": host,
                "target": target,
                "role": role,
                "samples": len(samples),
                "replies": len(rtt),
                "missing_replies": sum(s.get("status") in ("no_reply", "timeout") for s in samples),
                "probe_errors": sum(s.get("status") in ("error", "unavailable") for s in samples),
                "p50_ms": rtt[math.ceil(len(rtt) * 0.5) - 1] if rtt else None,
                "p95_ms": rtt[math.ceil(len(rtt) * 0.95) - 1] if rtt else None,
                "max_ms": max(rtt) if rtt else None,
                "scope": samples[-1]["scope"],
            }
        )
    return {
        "radios": [
            {
                "host": host,
                "interface": interface,
                "samples": len(samples),
                "association_changes": sum(s.get("association_changed", False) for s in samples)
                if any("association_changed" in s for s in samples)
                else None,
                "latest": max(samples, key=lambda s: s.get("mono_ns", 0)),
            }
            for (host, interface), samples in radios.items()
        ],
        "paths": paths,
        "attribution": "Radio counters and path RTT are measured separately. An ICMP round trip does not isolate radio airtime, AP processing or backhaul.",
    }


def report_data(
    manifests: list[dict[str, Any]],
    events: list[dict[str, Any]],
    profile: dict[str, Any] | None = None,
) -> dict[str, Any]:
    analysis = analyze(manifests, events)
    latest = {}
    for manifest in sorted(manifests, key=lambda m: m.get("captured_ns", m.get("mono_ns", 0))):
        latest[str(manifest.get("process", "unknown"))] = manifest
    samples: dict[str, dict[str, Any]] = {}
    # A bounded interactive excerpt; complete events remain in the exported bundle.
    for event in events[-10000:]:
        context = event.get("context")
        if not isinstance(context, dict):
            continue
        try:
            trace = TraceContext.decode(json.dumps({"schema": SCHEMA, **context}).encode())
        except ValueError:
            continue
        sample = samples.setdefault(
            trace.sample_id,
            {
                "id": trace.sample_id,
                "stream": trace.stream,
                "sequence": trace.sequence,
                "parents": list(trace.parents),
                "lineage": trace.lineage,
                "events": [],
            },
        )
        sample["events"].append(
            {
                key: event[key]
                for key in (
                    "event",
                    "mono_ns",
                    "clock_domain",
                    "process",
                    "duration_ns",
                    "span_id",
                    "outcome",
                    "reason",
                    "channel",
                    "queue",
                    "age_ns",
                )
                if key in event
            }
        )
    graph = topology(manifests, events)
    process_stages: dict[str, set[str]] = defaultdict(set)
    for event in events:
        process_stages[str(event.get("process", "unknown"))].add(str(event.get("event", "")))
    for node in graph["nodes"]:
        if node["kind"] != "process":
            continue
        process = json.loads(node["id"])[1]
        stages = process_stages[process]
        role = (
            "Gateway"
            if any(s.startswith("gateway.") for s in stages)
            else "Browser"
            if "video.presented" in stages or "operator.input" in stages
            else "Control worker"
            if any(s.startswith("control.") for s in stages)
            else "Worker"
        )
        node["label"] = f"{role} · {node.get('pid') or process[:8]}"
        node["role_basis"] = "observed boundary events"
        node["timing"] = [
            group for group in analysis["stage_groups"] if group["process"] == process
        ]
        node["budget_breaches"] = [
            group["stage"]
            for group in node["timing"]
            if group["p99_ms"]
            > (profile or {}).get("stages", {}).get(group["stage"], {}).get("p99_ms", math.inf)
        ]
        node["faults"] = [
            {key: fault[key] for key in ("event", "reason", "outcome", "mono_ns") if key in fault}
            for fault in analysis["faults"]
            if fault.get("process") == process
        ][-20:]
        node["coverage"] = latest.get(process, {}).get("health", {})
    return {
        "schema": "dimos.report/v1",
        "event_count": len(events),
        "processes": list(latest.values()),
        "graph": graph,
        "analysis": {
            key: analysis[key]
            for key in (
                "stages",
                "stage_groups",
                "unmatched_spans",
                "observer_drops",
                "delivery_sequences",
            )
        },
        "network": network_summary(events),
        "evaluation": evaluate(manifests, events, profile)
        if profile
        else {
            "status": "not_evaluated",
            "reason": "No latency profile supplied; numbers are observations, not a pass.",
        },
        "samples": list(samples.values()),
        "matched_paths": {
            "input_to_dispatch": matched_paths(events, "operator.input", "control.dispatch"),
            "camera_to_render": matched_paths(events, "camera.acquired", "video.render_submit"),
            "camera_to_xr": matched_paths(events, "camera.acquired", "video.xr_submit"),
        },
        "timeline_excerpt": "Interactive timelines contain the last 10,000 boundary records with trace identity. Full events are in the source bundle. Each clock domain has its own time origin.",
    }


def write_report(
    path: Path,
    manifests: list[dict[str, Any]],
    events: list[dict[str, Any]],
    profile: dict[str, Any] | None = None,
) -> None:
    data = report_data(manifests, events, profile)
    encoded = json.dumps(data, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    template = Path(__file__).with_name("report.html").read_text()
    path.write_text(template.replace("__REPORT_DATA__", encoded))
