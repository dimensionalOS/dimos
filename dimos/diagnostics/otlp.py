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

"""Offline OTLP/HTTP JSON export; never put an exporter on the control path."""

from __future__ import annotations

import hashlib
from typing import Any

import requests


def otlp(manifests: list[dict[str, Any]], events: list[dict[str, Any]]) -> dict[str, Any]:
    anchors = {
        m.get("process"): m
        for m in manifests
        if isinstance(m.get("wall_ns"), int) and isinstance(m.get("mono_ns"), int)
    }
    starts: dict[tuple[str, str], dict[str, Any]] = {}
    spans = []
    omitted = 0
    for event in sorted(events, key=lambda e: e.get("mono_ns", 0)):
        name = str(event.get("event", ""))
        key = (str(event.get("process", "")), str(event.get("span_id", "")))
        if name.endswith(".start") and key[1]:
            starts[key] = event
        elif name.endswith(".end") and key[1]:
            start = starts.pop(key, None)
            anchor = anchors.get(key[0])
            if (
                not start
                or not anchor
                or start.get("clock_domain") != event.get("clock_domain")
                or anchor.get("clock_domain") != event.get("clock_domain")
                or event["mono_ns"] < start["mono_ns"]
            ):
                omitted += 1
                continue
            if start["event"][:-6] != name[:-4]:
                omitted += 1
                continue
            base = anchor["wall_ns"] - anchor["mono_ns"]
            trace_id = hashlib.sha256(str(anchor.get("run_id", "unknown")).encode()).hexdigest()[
                :32
            ]
            span_id = hashlib.sha256(repr(key).encode()).hexdigest()[:16]
            attributes = {
                "dimos.process": key[0],
                "dimos.clock_domain": event["clock_domain"],
                "dimos.timestamp_provenance": "local wall/monotonic anchor; not cross-host latency evidence",
                "dimos.channel": start.get("channel", ""),
            }
            spans.append(
                {
                    "traceId": trace_id,
                    "spanId": span_id,
                    "name": name[:-4],
                    "kind": 1,
                    "startTimeUnixNano": str(base + start["mono_ns"]),
                    "endTimeUnixNano": str(base + event["mono_ns"]),
                    "attributes": [
                        {"key": k, "value": {"stringValue": str(v)}} for k, v in attributes.items()
                    ],
                }
            )
    return {
        "resourceSpans": [
            {
                "resource": {
                    "attributes": [
                        {"key": "service.name", "value": {"stringValue": "dimos-diagnostics"}}
                    ]
                },
                "scopeSpans": [
                    {"scope": {"name": "dimos.diagnostics", "version": "1"}, "spans": spans}
                ],
            }
        ],
        "dimosExportHealth": {"omittedSpans": omitted + len(starts), "clockAnchors": len(anchors)},
    }


def send(session: requests.Session, endpoint: str, document: dict[str, Any]) -> dict[str, Any]:
    """Explicit caller-selected OTLP/HTTP endpoint; no automatic discovery or retry."""
    health = document.get("dimosExportHealth", {})
    response = session.post(
        endpoint,
        json={"resourceSpans": document["resourceSpans"]},
        timeout=10,
        allow_redirects=False,
    )
    response.raise_for_status()
    if response.status_code >= 300:
        raise ValueError("OTLP redirects are not followed")
    return {"status": "sent", "export_health": health}
