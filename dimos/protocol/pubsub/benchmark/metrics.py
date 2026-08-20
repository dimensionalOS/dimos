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

"""Metric aggregation for transport benchmark trials."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import asdict
import math
import random
from typing import Any

import numpy as np

from dimos.protocol.pubsub.benchmark.model import SCHEMA_VERSION, TrialRecord


def percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), quantile))


def summarize_trial(record: TrialRecord) -> dict[str, Any]:
    sent = {(sample.topic, sample.sequence) for sample in record.samples}
    delivered_samples = [sample for sample in record.samples if sample.received_ns is not None]
    delivered = {(sample.topic, sample.sequence, sample.receiver) for sample in delivered_samples}
    expected = len(sent) * record.spec.subscribers
    duplicates = sum(max(0, sample.delivery_count - 1) for sample in delivered_samples)
    out_of_order = sum(sample.out_of_order for sample in delivered_samples)
    latency_ms = [
        sample.latency_ns / 1e6 for sample in delivered_samples if sample.latency_ns is not None
    ]
    publish_ms = [sample.publish_call_ns / 1e6 for sample in record.samples]
    payload_bytes = sum(sample.payload_bytes for sample in delivered_samples)
    offered_bytes = sum(sample.payload_bytes for sample in record.samples) / record.spec.subscribers
    duration = record.measurement_s or record.spec.duration_s
    p99 = percentile(latency_ms, 99)
    p50 = percentile(latency_ms, 50)
    return {
        "schema_version": SCHEMA_VERSION,
        **record.spec.to_dict(),
        "sent": len(sent),
        "offered_messages_s": len(sent) / duration if duration else 0.0,
        "offered_bytes_s": offered_bytes / duration if duration else 0.0,
        "expected_deliveries": expected,
        "delivered": len(delivered),
        "loss_pct": 100.0 * max(0, expected - len(delivered)) / expected if expected else 0.0,
        "sequence_gaps": max(0, expected - len(delivered)),
        "duplicates": duplicates,
        "out_of_order": out_of_order,
        "goodput_bytes_s": payload_bytes / duration if duration else 0.0,
        "delivered_messages_s": len(delivered) / duration if duration else 0.0,
        "latency_ms_p50": p50,
        "latency_ms_p90": percentile(latency_ms, 90),
        "latency_ms_p99": p99,
        "latency_ms_p999": percentile(latency_ms, 99.9) if len(latency_ms) >= 1000 else None,
        "latency_ms_max": max(latency_ms, default=None),
        "jitter_ms_p99_p50": p99 - p50 if p99 is not None and p50 is not None else None,
        "publish_ms_p50": percentile(publish_ms, 50),
        "publish_ms_p99": percentile(publish_ms, 99),
        "publish_ms_max": max(publish_ms, default=None),
        "readiness_s": record.readiness_s,
        "drain_s": record.drain_s,
        "cpu_seconds": sum(usage.cpu_seconds for usage in record.usage),
        "peak_rss_bytes": sum(usage.peak_rss_bytes for usage in record.usage),
        "usage": [asdict(usage) for usage in record.usage],
        "error": record.error,
    }


_CAMPAIGN_METRICS = (
    "loss_pct",
    "goodput_bytes_s",
    "delivered_messages_s",
    "latency_ms_p50",
    "latency_ms_p99",
    "jitter_ms_p99_p50",
    "publish_ms_p99",
    "cpu_seconds",
    "peak_rss_bytes",
)

_ROS_RELATIVE_METRICS = (
    "latency_ms_p50",
    "latency_ms_p99",
    "publish_ms_p99",
    "goodput_bytes_s",
    "cpu_seconds",
    "peak_rss_bytes",
)


def aggregate_summaries(
    summaries: Iterable[dict[str, Any]], *, seed: int = 7
) -> list[dict[str, Any]]:
    """Aggregate independent trial repetitions without pooling their samples."""
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for summary in summaries:
        if summary.get("error"):
            continue
        workload = summary["workload"]
        key = (
            summary["stack"],
            summary["cohort"],
            summary["environment"],
            summary["profile"],
            workload["name"],
            summary["subscribers"],
        )
        groups.setdefault(key, []).append(summary)

    aggregates = []
    for key, rows in sorted(groups.items()):
        stack, cohort, environment, profile, workload, subscribers = key
        aggregate: dict[str, Any] = {
            "stack": stack,
            "cohort": cohort,
            "environment": environment,
            "profile": profile,
            "workload": workload,
            "subscribers": subscribers,
            "repetitions": len(rows),
        }
        for metric in _CAMPAIGN_METRICS:
            values = [float(row[metric]) for row in rows if row.get(metric) is not None]
            interval = bootstrap_median_interval(values, seed=seed)
            if interval is None:
                continue
            median, low, high = interval
            aggregate[metric] = median
            aggregate[f"{metric}_ci_low"] = low
            aggregate[f"{metric}_ci_high"] = high
        aggregates.append(aggregate)
    return aggregates


def bootstrap_median_interval(
    values: Iterable[float], *, seed: int = 7, iterations: int = 2000
) -> tuple[float, float, float] | None:
    population = list(values)
    if not population:
        return None
    median = float(np.median(population))
    if len(population) == 1:
        return median, median, median
    rng = random.Random(seed)
    medians = []
    for _ in range(iterations):
        sample = [population[rng.randrange(len(population))] for _ in population]
        medians.append(float(np.median(sample)))
    medians.sort()
    low_index = math.floor(0.025 * (len(medians) - 1))
    high_index = math.ceil(0.975 * (len(medians) - 1))
    return median, medians[low_index], medians[high_index]


def _bootstrap_comparison_interval(
    candidate: list[float],
    baseline: list[float],
    *,
    difference: bool,
    seed: int,
    iterations: int = 2000,
) -> tuple[float, float, float] | None:
    if not candidate or not baseline:
        return None

    def compare(left: list[float], right: list[float]) -> float | None:
        left_median = float(np.median(left))
        right_median = float(np.median(right))
        if difference:
            return left_median - right_median
        if right_median == 0:
            return None
        return 100.0 * (left_median / right_median - 1.0)

    point = compare(candidate, baseline)
    if point is None:
        return None
    rng = random.Random(seed)
    estimates = []
    for _ in range(iterations):
        left = [candidate[rng.randrange(len(candidate))] for _ in candidate]
        right = [baseline[rng.randrange(len(baseline))] for _ in baseline]
        estimate = compare(left, right)
        if estimate is not None:
            estimates.append(estimate)
    estimates.sort()
    if not estimates:
        return point, point, point
    low_index = math.floor(0.025 * (len(estimates) - 1))
    high_index = math.ceil(0.975 * (len(estimates) - 1))
    return point, estimates[low_index], estimates[high_index]


def compare_to_ros(summaries: Iterable[dict[str, Any]], *, seed: int = 7) -> list[dict[str, Any]]:
    """Compare matched stack repetitions with ROS 2 over Zenoh."""
    groups: dict[tuple[Any, ...], dict[str, list[dict[str, Any]]]] = {}
    for summary in summaries:
        if summary.get("error"):
            continue
        key = (
            summary["cohort"],
            summary["environment"],
            summary["profile"],
            summary["workload"]["name"],
            summary["subscribers"],
        )
        groups.setdefault(key, {}).setdefault(summary["stack"], []).append(summary)

    comparisons = []
    for key, stacks in sorted(groups.items()):
        baseline = stacks.get("ros2-zenoh")
        if baseline is None:
            continue
        cohort, environment, profile, workload, subscribers = key
        for candidate_stack in ("lcm", "zenoh"):
            candidate = stacks.get(candidate_stack)
            if candidate is None:
                continue
            comparison: dict[str, Any] = {
                "candidate_stack": candidate_stack,
                "baseline_stack": "ros2-zenoh",
                "cohort": cohort,
                "environment": environment,
                "profile": profile,
                "workload": workload,
                "subscribers": subscribers,
                "candidate_repetitions": len(candidate),
                "baseline_repetitions": len(baseline),
            }
            for metric in _ROS_RELATIVE_METRICS:
                candidate_values = [
                    float(row[metric]) for row in candidate if row.get(metric) is not None
                ]
                baseline_values = [
                    float(row[metric]) for row in baseline if row.get(metric) is not None
                ]
                interval = _bootstrap_comparison_interval(
                    candidate_values,
                    baseline_values,
                    difference=False,
                    seed=seed,
                )
                if interval is None:
                    continue
                value, low, high = interval
                comparison[f"{metric}_delta_pct"] = value
                comparison[f"{metric}_delta_pct_ci_low"] = low
                comparison[f"{metric}_delta_pct_ci_high"] = high

            loss_interval = _bootstrap_comparison_interval(
                [float(row["loss_pct"]) for row in candidate],
                [float(row["loss_pct"]) for row in baseline],
                difference=True,
                seed=seed,
            )
            if loss_interval is not None:
                value, low, high = loss_interval
                comparison["loss_pct_delta_pp"] = value
                comparison["loss_pct_delta_pp_ci_low"] = low
                comparison["loss_pct_delta_pp_ci_high"] = high
            comparisons.append(comparison)
    return comparisons
