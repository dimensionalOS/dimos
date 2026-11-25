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

"""Offline ROS 2/LTTng executor evidence, kept in the CTF clock domain.

Tracepoint definitions: https://github.com/ros2/ros2_tracing/tree/rolling/tracetools
Reader API: https://babeltrace.org/docs/v2.0/python/bt2/examples.html
This imports existing traces; it does not enable system-wide tracing or claim
that a callback pointer identifies a transported robot sample.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator
from importlib import import_module
import json
from pathlib import Path
from typing import Any
from uuid import uuid4

_FIELDS = {"callback", "executor", "timeout", "subscription_handle", "publisher_handle", "message"}


def ctf_rows(directory: Path) -> Iterator[dict[str, Any]]:
    bt = import_module("bt2")
    for message in bt.TraceCollectionMessageIterator(str(directory)):
        if not isinstance(message, bt._EventMessageConst):
            continue
        event = message.event
        if not event.name.startswith("ros2:"):
            continue
        clock = message.default_clock_snapshot
        domain = str(clock.clock_class.uuid or f"stream-{event.stream.id}")
        # Raw cycles avoid treating CTF's epoch offset as clock synchronization.
        at = int(clock.value) * 1_000_000_000 // int(clock.clock_class.frequency)
        context = event.common_context_field
        payload = event.payload_field
        yield {
            "name": event.name,
            "mono_ns": at,
            "clock": domain,
            "pid": int(context["vpid"]) if context is not None and "vpid" in context else None,
            "tid": int(context["vtid"]) if context is not None and "vtid" in context else None,
            "fields": {
                key: int(payload[key]) for key in _FIELDS if payload is not None and key in payload
            },
        }


def boundaries(rows: Iterable[dict[str, Any]], run: str, capture: str) -> Iterator[dict[str, Any]]:
    active: dict[tuple[Any, ...], list[str]] = {}
    sequence = 0
    for row in rows:
        name = row["name"].removeprefix("ros2:")
        fields = row["fields"]
        common = {
            "schema": "dimos.trace/v1",
            "process": f"ros-{capture}-{row['pid']}",
            "clock_domain": f"ros-ctf/{capture}/{row['clock']}",
            "mono_ns": row["mono_ns"],
            "context": None,
            "source": "ros2_tracing",
            "pid": row["pid"],
            "tid": row["tid"],
            "fields": fields,
            "sample_correlation": "unavailable",
        }
        if name in ("callback_start", "callback_end"):
            key = (row["clock"], row["pid"], row["tid"], fields.get("callback"))
            if name == "callback_start":
                if len(active) >= 4096 or len(active.get(key, [])) >= 64:
                    yield {
                        **common,
                        "event": "trace.observer_drop",
                        "reason": "ROS callback pairing capacity",
                    }
                    continue
                sequence += 1
                identity = f"{capture}-{sequence}"
                active.setdefault(key, []).append(identity)
                yield {**common, "event": "ros.callback.start", "span_id": identity}
            else:
                stack = active.get(key, [])
                identity = stack.pop() if stack else f"unmatched-{sequence}"
                if not stack:
                    active.pop(key, None)
                yield {**common, "event": "ros.callback.end", "span_id": identity}
        elif name.startswith("rclcpp_executor_"):
            # These are actual executor points, not paired wait/compute spans:
            # get_next_ready may follow several wait phases and is not callback start.
            yield {**common, "event": "ros.executor." + name.removeprefix("rclcpp_executor_")}
        else:
            yield {**common, "event": "ros.tracepoint", "tracepoint": name}


def import_trace(directory: Path, output: Path, run: str) -> dict[str, Any]:
    capture = uuid4().hex
    count = 0
    size = 0
    manifest = {
        "kind": "manifest",
        "schema": "dimos.incident/v1",
        "run_id": run,
        "process": "ros-import-" + capture,
        "source": "ros2_tracing/CTF",
        "scope": "local executor/callback timing; cross-clock and sample identity unmapped",
    }
    with output.open("x") as destination:
        destination.write(json.dumps(manifest) + "\n")
        for event in boundaries(ctf_rows(directory), run, capture):
            line = json.dumps(event) + "\n"
            size += len(line.encode())
            if size > 1024 * 1024 * 1024:
                raise ValueError("ROS trace export exceeds 1 GiB; split the source trace")
            destination.write(line)
            count += 1
    return {
        "events": count,
        "bytes": size,
        "output": str(output),
        "qualification": "local ROS timing only",
    }
