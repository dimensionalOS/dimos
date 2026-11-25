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

"""Budget evaluation distinguishes a measured breach from insufficient evidence."""

from __future__ import annotations

from itertools import pairwise
import json
import math
from typing import Any

from dimos.diagnostics.analysis import analyze, current_manifests
from dimos.diagnostics.model import ClockMapping, TraceContext


def compare_overhead(baseline: dict[str, Any], traced: dict[str, Any]) -> dict[str, Any]:
    """Compare whole-fixture measurements without erasing clock uncertainty."""
    if baseline.get("mode") != "off" or traced.get("mode") != "trace":
        raise ValueError("supply off and trace measurements in that order")
    if any(
        baseline.get(k) != traced.get(k)
        for k in (
            "arms",
            "cameras",
            "scope",
            "extra_viewers",
            "recording",
            "decode_delay_ms",
            "control_delay_ms",
        )
    ):
        raise ValueError("measurements must use the same fixture workload")
    before, after = baseline["dispatch_p99_bounds_ms"], traced["dispatch_p99_bounds_ms"]
    extra = [after[0] - before[1], after[1] - before[0]]
    cpu = traced["host_cpu_percent"] - baseline["host_cpu_percent"]
    return {
        "schema": "dimos.overhead/v1",
        "scope": baseline["scope"],
        "added_host_cpu_percentage_points": cpu,
        "added_dispatch_p99_bounds_ms": extra,
        "telemetry_bytes_per_s": traced["telemetry_bytes_per_s"],
        "checks": {
            "cpu": "pass" if cpu < 2 else "fail",
            "command_age": "pass" if extra[1] < 1 else "fail" if extra[0] >= 1 else "inconclusive",
            "telemetry": "pass" if traced["telemetry_bytes_per_s"] <= 100 * 1024 else "fail",
        },
        "qualification": "synthetic workload only; not hardware or optical qualification",
    }


def sample_ages(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Age of the identified publication at receive/dispatch, not physical motion latency."""
    mappings = [
        ClockMapping(**e["mapping"])
        for e in events
        if e.get("event") == "network.probe" and e.get("status") == "ok"
    ]
    ages = []
    for e in events:
        if e.get("event") not in ("transport.received", "control.dispatch") or not e.get("context"):
            continue
        context = TraceContext.decode(
            json.dumps({"schema": "dimos.trace/v1", **e["context"]}).encode()
        )
        interval = elapsed_bounds(
            {"mono_ns": context.origin_ns, "clock_domain": context.clock_domain}, e, mappings
        )
        ages.append(
            {
                "sample_id": context.sample_id,
                "stage": e["event"],
                "bounds_ms": [v / 1e6 for v in interval] if interval else None,
                "status": "bounded" if interval else "clock_unmapped_or_expired",
                "lineage": context.lineage,
            }
        )
    return ages


def evaluate(
    manifests: list[dict[str, Any]], events: list[dict[str, Any]], profile: dict[str, Any]
) -> dict[str, Any]:
    """Evaluate explicitly supplied stage budgets; no implied teleop certification."""
    if profile.get("schema") != "dimos.budgets/v1" or not any(
        profile.get(k) for k in ("stages", "paths", "freshness")
    ):
        raise ValueError("a versioned profile with required stages is required")
    report = analyze(manifests, events)
    checks = []
    for stage, budget in profile.get("stages", {}).items():
        maximum = float(budget["p99_ms"])
        minimum = int(budget.get("min_samples", 100))
        if not math.isfinite(maximum) or maximum <= 0 or minimum < 1:
            raise ValueError("budgets must be finite and positive")
        measured = report["stages"].get(stage)
        state = "inconclusive"
        reason = "missing_stage_or_insufficient_samples"
        if measured and measured["n"] >= minimum:
            if measured["p99_ms"] > maximum:
                state, reason = "fail", "measured_stage_exceeds_budget"
            elif report["observer_drops"] or report["unmatched_spans"]:
                reason = "incomplete_capture"
            else:
                state, reason = "pass", "measured_stage_within_budget"
        checks.append(
            {
                "stage": stage,
                "status": state,
                "reason": reason,
                "budget_ms": maximum,
                "measured": measured,
            }
        )
    checks.extend(evaluate_paths(events, profile.get("paths", {})))
    checks.extend(
        evaluate_freshness(events, profile.get("freshness", {}), profile.get("min_duration_s", 0))
    )
    for manifest in current_manifests(manifests):
        health = manifest.get("health", {})
        complete = health.get("capture_window_complete")
        # Older collectors advertised a window but never enforced it.
        if complete is None and health.get("requested_pre_seconds") is not None:
            complete = health.get("retained_seconds", 0) >= health["requested_pre_seconds"]
        if complete is False:
            checks.append(
                {
                    "status": "inconclusive",
                    "reason": "incomplete_capture_window",
                    "process": manifest.get("process"),
                    "retained_seconds": health.get("retained_seconds"),
                    "requested_pre_seconds": health.get("requested_pre_seconds"),
                    "capacity_truncated": health.get("capacity_truncated"),
                }
            )
    if profile.get("min_duration_s") is not None:
        domains: dict[str, list[int]] = {}
        for event in events:
            if event.get("clock_domain") and isinstance(event.get("mono_ns"), int):
                domains.setdefault(event["clock_domain"], []).append(event["mono_ns"])
        duration = min(((max(v) - min(v)) / 1e9 for v in domains.values()), default=0)
        minimum_duration = float(profile["min_duration_s"])
        if not math.isfinite(minimum_duration) or minimum_duration <= 0:
            raise ValueError("duration must be finite and positive")
        checks.append(
            {
                "status": "pass" if duration >= minimum_duration else "inconclusive",
                "reason": "capture_duration",
                "minimum_s": minimum_duration,
                "shortest_domain_s": duration,
            }
        )
    observer_incomplete = (
        report["observer_drops"]
        or any(
            m.get("health", {}).get("io_errors", 0) or m.get("health", {}).get("observer_drops", 0)
            for m in manifests
        )
        or any(e.get("event") == "trace.observer_drop" for e in events)
    )
    if observer_incomplete:
        for check in checks:
            if check["status"] == "pass":
                check["status"], check["reason"] = "inconclusive", "observer_loss"
    states = {check["status"] for check in checks}
    return {
        "schema": "dimos.evaluation/v1",
        "scope": "specified matched paths, stages and freshness budgets",
        "status": "fail"
        if "fail" in states
        else "inconclusive"
        if "inconclusive" in states
        else "pass",
        "checks": checks,
        "teleop_qualification": "not_established",
    }


def elapsed_bounds(
    start: dict[str, Any], end: dict[str, Any], mappings: list[ClockMapping]
) -> tuple[int, int] | None:
    """Bound a matched duration in either direction, including asymmetric clock error."""
    sent, received = start["mono_ns"], end["mono_ns"]
    if start.get("clock_domain") == end.get("clock_domain"):
        return (received - sent,) * 2 if received >= sent else None
    intervals = []
    for mapping in mappings:
        if (mapping.local_domain, mapping.remote_domain) == (
            start.get("clock_domain"),
            end.get("clock_domain"),
        ):
            value = mapping.latency_bounds(sent, received, max(sent, received - mapping.lower_ns))
            if value is not None:
                intervals.append(value)
        elif (mapping.remote_domain, mapping.local_domain) == (
            start.get("clock_domain"),
            end.get("clock_domain"),
        ):
            age = received - mapping.measured_ns
            if 0 <= age <= mapping.valid_for_ns:
                drift = math.ceil(age * mapping.drift_ppm / 1_000_000)
                low, high = (
                    received - sent + mapping.lower_ns - drift,
                    received - sent + mapping.upper_ns + drift,
                )
                if high >= 0:
                    intervals.append((max(0, low), high))
    return min(intervals, key=lambda value: value[1] - value[0]) if intervals else None


def matched_paths(
    events: list[dict[str, Any]], source: str, destination: str
) -> list[dict[str, Any]]:
    """Trace actual causal ancestry. Missing parents and cycles cannot manufacture matches."""
    contexts: dict[str, TraceContext] = {}
    starts: dict[str, dict[str, Any]] = {}
    endpoints: list[tuple[str, dict[str, Any]]] = []
    mappings = []
    for event in events:
        if event.get("event") == "network.probe" and event.get("status") == "ok":
            try:
                mappings.append(ClockMapping(**event["mapping"]))
            except (TypeError, ValueError, KeyError):
                pass
        if not isinstance(event.get("context"), dict):
            continue
        try:
            context = TraceContext.decode(
                json.dumps({"schema": "dimos.trace/v1", **event["context"]}).encode()
            )
        except ValueError:
            continue
        contexts[context.sample_id] = context
        if event.get("event") == source:
            starts.setdefault(context.sample_id, event)
        if event.get("event") == destination:
            endpoints.append((context.sample_id, event))
    result: list[dict[str, Any]] = []
    for identity, end in endpoints:
        todo = [identity]
        visited: set[str] = set()
        roots: set[str] = set()
        missing: set[str] = set()
        while todo and len(visited) < 4096:
            current = todo.pop()
            if current in visited:
                continue
            visited.add(current)
            if current in starts:
                roots.add(current)
            elif current in contexts and contexts[current].parents:
                todo.extend(contexts[current].parents)
            else:
                missing.add(current)
        if todo:
            missing.add("ancestry_limit")
        if not roots:
            result.append(
                {
                    "sample_id": identity,
                    "source": source,
                    "destination": destination,
                    "status": "missing_source_lineage",
                    "bounds_ms": None,
                }
            )
        for root in sorted(roots):
            bounds = elapsed_bounds(starts[root], end, mappings)
            result.append(
                {
                    "sample_id": identity,
                    "source_id": root,
                    "source": source,
                    "destination": destination,
                    "channel": starts[root].get("channel", contexts[root].stream),
                    "status": "partial_lineage"
                    if missing
                    else "bounded"
                    if bounds
                    else "clock_unmapped_or_expired",
                    "bounds_ms": [n / 1e6 for n in bounds] if bounds else None,
                    "missing_parents": sorted(missing),
                }
            )
    return result


def evaluate_paths(events: list[dict[str, Any]], profiles: dict[str, Any]) -> list[dict[str, Any]]:
    checks = []
    for name, profile in profiles.items():
        paths = matched_paths(events, profile["source"], profile["destination"])
        channels = profile.get("channels", [])
        valid = [p["bounds_ms"] for p in paths if p["status"] == "bounded"]
        percentile = float(profile.get("percentile", 0.95))
        budget = float(profile["max_ms"])
        minimum = int(profile.get("min_samples", 100))
        channel_counts = {
            channel: sum(p.get("channel") == channel and p["status"] == "bounded" for p in paths)
            for channel in channels
        }
        missing_channels = [channel for channel, count in channel_counts.items() if count < minimum]
        if not 0 < percentile <= 1 or not math.isfinite(budget) or budget <= 0 or minimum < 1:
            raise ValueError("invalid path budget")
        state, reason, measured = "inconclusive", "missing_or_uncertain_samples", None
        if len(valid) >= minimum:
            index = max(0, math.ceil(percentile * len(valid)) - 1)
            measured = [sorted(v[i] for v in valid)[index] for i in (0, 1)]
            if measured[0] > budget:
                state, reason = "fail", "matched_latency_exceeds_budget"
            elif measured[1] <= budget and len(valid) == len(paths) and not missing_channels:
                state, reason = "pass", "matched_latency_within_budget"
            else:
                reason = "clock_uncertainty_or_missing_lineage"
        per_channel = {}
        for channel in channels:
            selected = [p for p in paths if p.get("channel") == channel]
            bounds = [p["bounds_ms"] for p in selected if p["status"] == "bounded"]
            channel_state = "inconclusive"
            channel_measured = None
            if len(bounds) >= minimum:
                index = max(0, math.ceil(percentile * len(bounds)) - 1)
                channel_measured = [sorted(v[i] for v in bounds)[index] for i in (0, 1)]
                if channel_measured[0] > budget:
                    channel_state = "fail"
                elif channel_measured[1] <= budget and len(bounds) == len(selected):
                    channel_state = "pass"
            per_channel[channel] = {"status": channel_state, "bounds_ms": channel_measured}
        if any(c["status"] == "fail" for c in per_channel.values()):
            state, reason = "fail", "required_channel_exceeds_budget"
        elif state == "pass" and any(c["status"] != "pass" for c in per_channel.values()):
            state, reason = "inconclusive", "required_channel_missing_or_uncertain"
        checks.append(
            {
                "path": name,
                "status": state,
                "reason": reason,
                "matched": len(valid),
                "unmatched": len(paths) - len(valid),
                "percentile": percentile,
                "bounds_ms": measured,
                "budget_ms": budget,
                "missing_channels": missing_channels,
                "samples_per_channel": channel_counts,
                "per_channel": per_channel,
            }
        )
    return checks


def evaluate_freshness(
    events: list[dict[str, Any]], profiles: dict[str, Any], minimum_duration_s: float = 0
) -> list[dict[str, Any]]:
    """Require coverage of each observer's active intervals, including both edges.

    Another stream or browser cannot fill a missing observer's history. Explicit
    watch deactivation ends an interval; silence never implicitly deactivates it.
    """
    observers: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for event in events:
        if isinstance(event.get("mono_ns"), int) and event.get("clock_domain"):
            key = (event.get("process", ""), event["clock_domain"])
            observers.setdefault(key, []).append(event)
    checks = []
    for name, profile in profiles.items():
        values = [e for e in events if e.get("event") == "freshness.age" and e.get("watch") == name]
        maximum = max((e["age_ns"] / 1e6 for e in values), default=None)
        allowed = float(profile["max_ms"])
        allowed_gap = float(profile.get("max_observation_gap_ms", 500))
        required_duration = float(profile.get("min_duration_s", minimum_duration_s))
        minimum = int(profile.get("min_samples", 2))
        if (
            not math.isfinite(allowed)
            or allowed <= 0
            or not math.isfinite(allowed_gap)
            or allowed_gap <= 0
            or not math.isfinite(required_duration)
            or required_duration < 0
            or minimum < 1
        ):
            raise ValueError("invalid freshness budget")
        status = "inconclusive" if maximum is None else "fail" if maximum > allowed else "pass"
        coverage = []
        for (process, domain), records in observers.items():
            records = sorted(records, key=lambda e: e["mono_ns"])
            watch_records = [
                e
                for e in records
                if e.get("watch") == name and e["event"] in ("freshness.age", "freshness.observe")
            ]
            if not watch_records:
                continue
            intervals = []
            start = None
            for event in watch_records:
                if event["event"] == "freshness.observe" and not event.get("active", True):
                    if start is not None:
                        intervals.append((start, event["mono_ns"]))
                    start = None
                elif start is None:
                    # Without an explicit activation, history must cover the
                    # retained observer interval, not just its delivered samples.
                    start = (
                        event["mono_ns"]
                        if event["event"] == "freshness.observe"
                        else records[0]["mono_ns"]
                    )
            if start is not None:
                intervals.append((start, records[-1]["mono_ns"]))
            samples = [e["mono_ns"] for e in watch_records if e["event"] == "freshness.age"]
            gaps: list[int] = []
            for begin, end in intervals:
                points = [begin, *(at for at in samples if begin <= at <= end), end]
                gaps.extend(b - a for a, b in pairwise(points))
            duration = sum(end - begin for begin, end in intervals) / 1e9
            gap = max(gaps, default=0) / 1e6
            coverage.append(
                {
                    "process": process,
                    "clock_domain": domain,
                    "samples": len(samples),
                    "active_duration_s": duration,
                    "max_observation_gap_ms": gap,
                    "complete": len(samples) >= minimum
                    and gap <= allowed_gap
                    and duration >= required_duration,
                }
            )
        missing_observers = sorted(
            set(profile.get("processes", [])) - {c["process"] for c in coverage}
        )
        gap_ms = max((c["max_observation_gap_ms"] for c in coverage), default=0)
        coverage_missing = (
            not coverage or bool(missing_observers) or any(not c["complete"] for c in coverage)
        )
        if status == "pass" and coverage_missing:
            status = "inconclusive"
        checks.append(
            {
                "watch": name,
                "status": status,
                "max_age_ms": maximum,
                "budget_ms": allowed,
                "n": len(values),
                "max_observation_gap_ms": gap_ms,
                "coverage": coverage,
                "missing_observers": missing_observers,
            }
        )
    return checks
