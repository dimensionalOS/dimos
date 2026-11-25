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

"""Offline analysis preserves incomplete coverage and never treats missing data as healthy."""

from __future__ import annotations

from collections import defaultdict
from itertools import pairwise
import json
import math
from pathlib import Path
from typing import Any


def load_bundle(path: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    manifests: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    paths = sorted(path.rglob("incident-*.jsonl")) if path.is_dir() else [path]
    for file in paths:
        for line in file.read_text().splitlines():
            row = json.loads(line)
            (manifests if row.get("kind") == "manifest" else events).append(row)
    # Overlapping incident windows must not multiply counts.
    events = list({json.dumps(e, sort_keys=True): e for e in events}.values())
    return manifests, resolve_rtp_contexts(events)


def current_manifests(manifests: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Select checkpoint health by recorded time, never by incident filename.

    Actual triggered incidents and old records without timestamps retain their
    individual coverage checks. A later checkpoint cannot repair such an incident.
    """
    checkpoints: dict[tuple[str, str], dict[str, Any]] = {}
    retained = []
    for manifest in manifests:
        if manifest.get("reason") != "checkpoint" or not isinstance(
            manifest.get("captured_ns"), int
        ):
            retained.append(manifest)
            continue
        key = (manifest.get("process", ""), manifest.get("clock_domain", ""))
        previous = checkpoints.get(key)
        if previous is None or manifest["captured_ns"] > previous["captured_ns"]:
            checkpoints[key] = manifest
    return [*retained, *checkpoints.values()]


def resolve_rtp_contexts(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Join pacer evidence by process/pacer/SSRC/RTP incarnation, never by arrival time."""
    frames: dict[tuple[Any, ...], dict[str, dict[str, Any]]] = defaultdict(dict)
    for event in events:
        if event.get("event") != "video.rtp_frame" or event.get("rtp_timestamp") is None:
            continue
        if not isinstance(event.get("context"), dict):
            continue
        key = tuple(event.get(k) for k in ("process", "pacer", "ssrc", "rtp_timestamp"))
        if None in key:
            continue
        context = event["context"]
        frames[key][json.dumps(context, sort_keys=True)] = context
    output = []
    for event in events:
        if event.get("event") not in ("video.pacer_drop", "video.pacer_departure"):
            output.append(event)
            continue
        key = tuple(event.get(k) for k in ("process", "pacer", "ssrc", "rtp_timestamp"))
        contexts = frames.get(key, {})
        output.append(
            {
                **event,
                "context": next(iter(contexts.values())) if len(contexts) == 1 else None,
                "correlation": "matched_rtp" if len(contexts) == 1 else "ambiguous_or_missing_rtp",
            }
        )
    return output


def analyze(manifests: list[dict[str, Any]], events: list[dict[str, Any]]) -> dict[str, Any]:
    starts: dict[tuple[str, str], dict[str, Any]] = {}
    durations: dict[str, list[float]] = defaultdict(list)
    groups: dict[tuple[str, str, str], list[float]] = defaultdict(list)
    faults, edges = [], {}
    unmatched = 0
    for e in sorted(events, key=lambda e: e.get("mono_ns", 0)):
        name = e.get("event", "")
        if name == "queue.delay" and e.get("duration_ns", -1) >= 0:
            durations["queue." + e.get("queue", "unknown")].append(e["duration_ns"] / 1e6)
        key = (e.get("process", ""), e.get("span_id", ""))
        if name == "topology.edge":
            identity = (e.get("process"), e.get("channel"), e.get("direction"), e.get("transport"))
            edges[identity] = e
        if name.endswith(".start") and key[1]:
            starts[key] = e
        elif name.endswith(".end") and key[1]:
            start = starts.pop(key, None)
            if (
                start is None
                or not start.get("clock_domain")
                or start.get("clock_domain") != e.get("clock_domain")
                or start.get("event", "")[:-6] != name[:-4]
                or e["mono_ns"] < start["mono_ns"]
            ):
                unmatched += 1
                continue
            duration = (e["mono_ns"] - start["mono_ns"]) / 1e6
            durations[name[:-4]].append(duration)
            labels = {
                k: start[k]
                for k in ("channel", "transport", "task", "hardware", "queue")
                if k in start
            }
            groups[
                (name[:-4], start.get("process", "unknown"), json.dumps(labels, sort_keys=True))
            ].append(duration)
        if e.get("outcome") == "error" or name in (
            "control.no_output",
            "control.ik_no_output",
            "trace.invalid_context",
        ):
            faults.append(e)

    def spread(xs: list[float]) -> dict[str, Any]:
        xs = sorted(xs)
        return {
            "n": len(xs),
            "p50_ms": xs[math.ceil(len(xs) * 0.5) - 1],
            "p95_ms": xs[math.ceil(len(xs) * 0.95) - 1],
            "p99_ms": xs[math.ceil(len(xs) * 0.99) - 1],
            "max_ms": xs[-1],
        }

    return {
        "schema": "dimos.analysis/v1",
        "qualification": "incomplete",
        "reason": "Stage spans alone do not certify command age, physical response or photon latency.",
        "manifests": manifests,
        "stages": {k: spread(v) for k, v in durations.items()},
        "stage_groups": [
            {"stage": stage, "process": process, "labels": json.loads(labels), **spread(values)}
            for (stage, process, labels), values in groups.items()
        ],
        "delivery_sequences": delivery_sequences(events),
        "unmatched_spans": unmatched + len(starts),
        "faults": faults,
        "topology": list(edges.values()),
        "observer_drops": sum(
            max(
                m.get("health", {}).get("observer_drops", 0)
                for m in manifests
                if m.get("process") == process
            )
            for process in {m.get("process") for m in manifests}
        ),
    }


def delivery_sequences(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    streams: dict[tuple[str, ...], list[int]] = defaultdict(list)
    for event in events:
        context = event.get("context")
        if (
            event.get("event") != "transport.received"
            or not context
            or context.get("lineage") == "unknown"
        ):
            continue
        key = tuple(str(context.get(k, "")) for k in ("run_id", "publisher", "stream")) + tuple(
            str(event.get(k, "")) for k in ("process", "channel", "transport")
        )
        streams[key].append(context["sequence"])
    result = []
    for key, sequences in streams.items():
        unique = set(sequences)
        result.append(
            {
                "source_run_publisher_stream_receiver_channel_transport": key,
                "received": len(sequences),
                "duplicates": len(sequences) - len(unique),
                "out_of_order": sum(b < a for a, b in pairwise(sequences)),
                "unobserved_sequences": max(unique) - min(unique) + 1 - len(unique),
                "gap_cause": "unattributed: capture loss, transport loss or intentional supersession",
            }
        )
    return result


def perfetto(events: list[dict[str, Any]], clock_domain: str | None = None) -> dict[str, Any]:
    domains = {e["clock_domain"] for e in events if e.get("span_id") and e.get("clock_domain")}
    if clock_domain is None and len(domains) > 1:
        raise ValueError("select one --clock-domain; unaligned clocks cannot share a timeline")
    if clock_domain is not None:
        events = [e for e in events if e.get("clock_domain") == clock_domain]
    processes = {
        p: i + 1 for i, p in enumerate(sorted({e.get("process", "unknown") for e in events}))
    }
    rows = []
    for e in events:
        name = e.get("event", "")
        if not (name.endswith(".start") or name.endswith(".end")) or not e.get("span_id"):
            continue
        # Async spans retain pairing even when process callbacks interleave.
        rows.append(
            {
                "name": name.rsplit(".", 1)[0],
                "cat": e.get("clock_domain", "unknown"),
                "ph": "b" if name.endswith(".start") else "e",
                "id": e["span_id"],
                "pid": processes[e.get("process", "unknown")],
                "tid": 0,
                "ts": e["mono_ns"] / 1000,
                "args": {
                    "process": e.get("process"),
                    "context": e.get("context"),
                    "outcome": e.get("outcome"),
                },
            }
        )
    return {
        "traceEvents": rows,
        "displayTimeUnit": "ms",
        "metadata": {
            "clock_warning": "Domains are not aligned; compare durations within a domain only."
        },
    }
