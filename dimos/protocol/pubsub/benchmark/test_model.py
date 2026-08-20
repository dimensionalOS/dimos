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

from collections import Counter

from dimos.protocol.pubsub.benchmark.matrix import build_matrix
from dimos.protocol.pubsub.benchmark.metrics import (
    aggregate_summaries,
    bootstrap_median_interval,
    compare_to_ros,
    summarize_trial,
)
from dimos.protocol.pubsub.benchmark.model import (
    Cohort,
    Environment,
    MessageSample,
    NetworkProfile,
    ProcessUsage,
    Stack,
    TrialRecord,
    trial_spec_from_dict,
)


def test_trial_spec_round_trips_through_manifest_dict() -> None:
    original = build_matrix("smoke")[0]

    restored = trial_spec_from_dict(original.to_dict())

    assert restored == original


def test_public_matrix_contains_semantic_cohorts_and_focused_fanout() -> None:
    matrix = build_matrix("public")

    assert len(matrix) == 730
    assert all(spec.warmup_s == 2.0 for spec in matrix)
    assert all(spec.duration_s == 20.0 for spec in matrix)
    assert all(spec.drain_s == 1.0 for spec in matrix)
    assert not any(spec.stack == Stack.LCM and spec.cohort == Cohort.RELIABLE for spec in matrix)
    assert {(spec.environment, spec.profile) for spec in matrix} == {
        (Environment.LOCAL, NetworkProfile.CLEAN),
        (Environment.EMULATED, NetworkProfile.CLEAN),
        (Environment.EMULATED, NetworkProfile.CONSTRAINED),
        (Environment.EMULATED, NetworkProfile.DEGRADED),
    }
    assert {spec.subscribers for spec in matrix} == {1, 2, 4, 8}
    assert all(spec.profile == NetworkProfile.CLEAN for spec in matrix if spec.subscribers > 1)
    assert not any(
        spec.profile in {NetworkProfile.CONSTRAINED, NetworkProfile.DEGRADED}
        and spec.workload.name
        not in {
            "bytes-1024-1000hz",
            "bytes-1048576-30hz",
            "vision-lidar-stack",
        }
        for spec in matrix
    )
    cells = {
        (
            spec.stack,
            spec.cohort,
            spec.environment,
            spec.profile,
            spec.workload.name,
            spec.subscribers,
        )
        for spec in matrix
    }
    assert len(cells) == 146
    repetitions = Counter(
        (
            spec.stack,
            spec.cohort,
            spec.environment,
            spec.profile,
            spec.workload.name,
            spec.subscribers,
        )
        for spec in matrix
    )
    assert set(repetitions.values()) == {5}
    saturation = [spec for spec in matrix if spec.workload.saturation]
    assert saturation
    assert all(spec.workload.saturation_max_messages == 100_000 for spec in saturation)
    assert all(spec.workload.saturation_max_bytes == 4 * 1024**3 for spec in saturation)


def test_matrix_randomization_is_seeded() -> None:
    first = [spec.trial_id for spec in build_matrix("smoke", seed=9)]
    second = [spec.trial_id for spec in build_matrix("smoke", seed=9)]
    different = [spec.trial_id for spec in build_matrix("smoke", seed=10)]

    assert first == second
    assert first != different


def test_summary_counts_loss_duplicates_latency_and_resources() -> None:
    spec = build_matrix("smoke", duration_s=1.0)[0]
    record = TrialRecord(
        spec=spec,
        samples=[
            MessageSample(
                spec.trial_id,
                "payload",
                0,
                0,
                100,
                1,
                10,
                20,
                1_000_010,
                delivery_count=2,
                out_of_order=True,
            ),
            MessageSample(spec.trial_id, "payload", 0, 1, 100, 2, 30, 40, None),
        ],
        usage=[ProcessUsage("publisher", 0.25, 1000), ProcessUsage("subscriber", 0.5, 2000)],
        measurement_s=0.5,
    )

    summary = summarize_trial(record)

    assert summary["sent"] == 2
    assert summary["delivered"] == 1
    assert summary["loss_pct"] == 50.0
    assert summary["sequence_gaps"] == 1
    assert summary["duplicates"] == 1
    assert summary["out_of_order"] == 1
    assert summary["latency_ms_p50"] == 1.0
    assert summary["goodput_bytes_s"] == 200.0
    assert summary["cpu_seconds"] == 0.75
    assert summary["peak_rss_bytes"] == 3000


def test_bootstrap_interval_is_deterministic_and_contains_median() -> None:
    interval = bootstrap_median_interval([1.0, 2.0, 3.0, 4.0, 5.0], seed=3)

    assert interval is not None
    median, low, high = interval
    assert median == 3.0
    assert low <= median <= high


def test_campaign_aggregation_keeps_delivery_cohorts_separate() -> None:
    specs = build_matrix("smoke", repetitions=2)
    summaries = [summarize_trial(TrialRecord(spec=spec)) for spec in specs]

    aggregates = aggregate_summaries(summaries)

    assert {(row["stack"], row["cohort"]) for row in aggregates} == {
        (spec.stack.value, spec.cohort.value) for spec in specs
    }


def test_ros_comparison_reports_percent_and_percentage_point_deltas() -> None:
    common = {
        "cohort": "freshness",
        "environment": "emulated",
        "profile": "clean",
        "workload": {"name": "payload"},
        "subscribers": 1,
        "error": None,
        "publish_ms_p99": 1.0,
        "goodput_bytes_s": 100.0,
        "cpu_seconds": 1.0,
        "peak_rss_bytes": 100.0,
    }
    summaries = [
        {
            **common,
            "stack": "lcm",
            "latency_ms_p50": 1.0,
            "latency_ms_p99": 2.0,
            "loss_pct": 1.5,
        },
        {
            **common,
            "stack": "ros2-zenoh",
            "latency_ms_p50": 2.0,
            "latency_ms_p99": 4.0,
            "loss_pct": 0.5,
        },
    ]

    comparison = compare_to_ros(summaries)[0]

    assert comparison["latency_ms_p99_delta_pct"] == -50.0
    assert comparison["goodput_bytes_s_delta_pct"] == 0.0
    assert comparison["loss_pct_delta_pp"] == 1.0
