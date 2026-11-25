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

"""Replayed timing faults must be localized without treating coverage gaps as health."""

from dataclasses import asdict

import pytest

from dimos.diagnostics.analysis import analyze
from dimos.diagnostics.evaluation import (
    compare_overhead,
    elapsed_bounds,
    evaluate,
    matched_paths,
    sample_ages,
)
from dimos.diagnostics.model import ClockMapping, TraceContext
from dimos.diagnostics.topology import topology


def test_overhead_clock_uncertainty_cannot_be_rounded_into_a_pass():
    baseline = dict(
        mode="off",
        host_cpu_percent=10,
        dispatch_p99_bounds_ms=[17.5, 18.7],
        arms=2,
        cameras=3,
        scope="same fixture",
    )
    traced = {
        **baseline,
        "mode": "trace",
        "host_cpu_percent": 10.5,
        "dispatch_p99_bounds_ms": [17.6, 19.1],
        "telemetry_bytes_per_s": 9000,
    }
    assert compare_overhead(baseline, traced)["checks"] == {
        "cpu": "pass",
        "command_age": "inconclusive",
        "telemetry": "pass",
    }


def test_fast_head_camera_cannot_hide_slow_wrist_camera():
    events = []
    for channel, count, latency_ms in [("head", 100, 10), ("wrist", 2, 200)]:
        for sequence in range(count):
            context = asdict(TraceContext("run", "camera", channel, sequence, 0, "clock"))
            events.extend(
                [
                    dict(event="camera.acquired", mono_ns=0, clock_domain="clock", context=context),
                    dict(
                        event="video.render_submit",
                        mono_ns=latency_ms * 1_000_000,
                        clock_domain="clock",
                        context=context,
                    ),
                ]
            )
    result = evaluate(
        [],
        events,
        {
            "schema": "dimos.budgets/v1",
            "paths": {
                "video": {
                    "source": "camera.acquired",
                    "destination": "video.render_submit",
                    "channels": ["head", "wrist"],
                    "min_samples": 2,
                    "max_ms": 100,
                }
            },
        },
    )
    check = result["checks"][0]
    assert check["bounds_ms"] == [10, 10]
    assert check["per_channel"]["wrist"] == {"status": "fail", "bounds_ms": [200, 200]}
    assert result["status"] == "fail"


def spans(stage, duration_ns, process="worker"):
    return [
        dict(
            event=f"{stage}.{suffix}",
            mono_ns=t,
            clock_domain="clock",
            process=process,
            span_id=stage,
        )
        for suffix, t in [("start", 0), ("end", duration_ns)]
    ]


@pytest.mark.parametrize("duration,status", [(5_000_000, "pass"), (30_000_000, "fail")])
def test_budget_evaluation_distinguishes_fast_and_injected_slow_stages(duration, status):
    profile = {
        "schema": "dimos.budgets/v1",
        "stages": {"video.encode": {"p99_ms": 10, "min_samples": 1}},
    }
    result = evaluate([], spans("video.encode", duration), profile)
    assert result["status"] == status
    assert result["teleop_qualification"] == "not_established"


def test_missing_stages_and_observer_loss_are_inconclusive():
    profile = {"schema": "dimos.budgets/v1", "stages": {"ik": {"p99_ms": 10, "min_samples": 1}}}
    assert evaluate([], [], profile)["status"] == "inconclusive"
    manifests = [{"process": "p", "health": {"observer_drops": 2}}] * 2
    result = evaluate(manifests, spans("ik", 1_000_000), profile)
    assert result["status"] == "inconclusive"
    assert analyze(manifests, [])["observer_drops"] == 2


def test_short_retention_cannot_pass_even_when_measured_stages_are_fast():
    profile = {"schema": "dimos.budgets/v1", "stages": {"ik": {"p99_ms": 10, "min_samples": 1}}}
    manifests = [
        {
            "process": "gateway",
            "health": {"requested_pre_seconds": 60, "retained_seconds": 22, "ring_evictions": 700},
        }
    ]
    result = evaluate(manifests, spans("ik", 1_000_000), profile)
    assert result["status"] == "inconclusive"
    assert result["checks"][-1]["reason"] == "incomplete_capture_window"


def test_asymmetric_clock_bounds_expire_and_never_become_half_rtt():
    mapping = ClockMapping.exchange("A", "B", 0, 130, 140, 50, valid_for_ns=1000, drift_ppm=0)
    context = TraceContext("run", "publisher", "hand", 1, 60, "A")
    receive = dict(
        event="transport.received", mono_ns=200, clock_domain="B", context=asdict(context)
    )
    probe = dict(event="network.probe", status="ok", mapping=asdict(mapping))
    assert sample_ages([receive])[0]["bounds_ms"] is None
    assert sample_ages([probe, receive])[0]["bounds_ms"] == [0.00001, 0.00005]
    receive["mono_ns"] = 2000
    assert sample_ages([probe, receive])[0]["bounds_ms"] is None


def test_reordered_sequences_and_publisher_restart_remain_distinct():
    events = [
        dict(
            event="transport.received",
            mono_ns=i,
            process="receiver",
            clock_domain="c",
            context=asdict(TraceContext("run", publisher, "camera", sequence, 0, "c")),
        )
        for i, (publisher, sequence) in enumerate(
            [("first", 1), ("first", 4), ("first", 3), ("first", 3), ("restart", 1)]
        )
    ]
    streams = analyze([], events)["delivery_sequences"]
    assert [(s["unobserved_sequences"], s["duplicates"], s["out_of_order"]) for s in streams] == [
        (1, 1, 1),
        (0, 0, 0),
    ]


def test_topology_does_not_invent_a_physical_route_or_join_topics_by_name():
    manifests = [{"process": "sender", "host": "robot"}, {"process": "receiver", "host": "headset"}]
    events = [
        dict(
            event="topology.edge",
            process=p,
            module="arm",
            endpoint="arm/pose",
            channel="pose",
            transport="Zenoh",
            direction=d,
        )
        for p, d in [("sender", "publish"), ("receiver", "subscribe")]
    ]
    graph = topology(manifests, events)
    assert len([n for n in graph["nodes"] if n["kind"] == "channel"]) == 2
    assert not any(e["kind"] == "sample_provenance" for e in graph["edges"])
    context = TraceContext("run", "sender", "pose", 1, 0, "c")
    events.append(dict(event="transport.received", process="receiver", context=asdict(context)))
    delivery_edges = [
        e for e in topology(manifests, events)["edges"] if e["kind"] == "sample_provenance"
    ]
    assert len(delivery_edges) == 1
    assert delivery_edges[0]["physical_route"] == "unresolved"


def test_matched_path_uses_actual_parent_not_a_nearby_identical_payload():
    first = TraceContext("run", "input", "left", 1, 0, "clock")
    second = TraceContext("run", "input", "left", 2, 10_000_000, "clock")
    output = TraceContext("run", "ik", "target", 1, 30_000_000, "clock", (first.sample_id,))
    events = [
        dict(event=stage, mono_ns=t, clock_domain="clock", context=asdict(context))
        for stage, t, context in [
            ("input", 0, first),
            ("input", 10_000_000, second),
            ("dispatch", 30_000_000, output),
        ]
    ]
    paths = matched_paths(events, "input", "dispatch")
    assert [(p["source_id"], p["bounds_ms"]) for p in paths] == [(first.sample_id, [30, 30])]
    profile = {
        "schema": "dimos.budgets/v1",
        "paths": {
            "control": {
                "source": "input",
                "destination": "dispatch",
                "min_samples": 1,
                "max_ms": 40,
                "channels": ["left", "right"],
            }
        },
    }
    result = evaluate([], events, profile)
    assert result["status"] == "inconclusive"
    assert result["checks"][0]["missing_channels"] == ["right"]


def test_missing_one_of_multiple_parents_prevents_a_false_pass():
    first = TraceContext("run", "input", "left", 1, 0, "clock")
    output = TraceContext("run", "ik", "target", 1, 1, "clock", (first.sample_id, "missing"))
    events = [
        dict(event=stage, mono_ns=t, clock_domain="clock", context=asdict(context))
        for stage, t, context in [("input", 0, first), ("dispatch", 1, output)]
    ]
    paths = matched_paths(events, "input", "dispatch")
    assert paths[0]["status"] == "partial_lineage"
    assert paths[0]["missing_parents"] == ["missing"]


def test_reverse_clock_mapping_preserves_asymmetry_and_expires():
    mapping = ClockMapping.exchange("A", "B", 0, 130, 140, 50, valid_for_ns=1000, drift_ppm=0)
    start = {"clock_domain": "B", "mono_ns": 200}
    assert elapsed_bounds(start, {"clock_domain": "A", "mono_ns": 150}, [mapping]) == (40, 80)
    assert elapsed_bounds(start, {"clock_domain": "A", "mono_ns": 2000}, [mapping]) is None


def test_freezes_and_missing_watchdog_coverage_do_not_report_healthy_video():
    profile = {"schema": "dimos.budgets/v1", "freshness": {"video": {"max_ms": 250}}}

    def age(t, age):
        return dict(event="freshness.age", watch="video", clock_domain="c", mono_ns=t, age_ns=age)

    healthy = [age(0, 0), age(100_000_000, 100_000_000)]
    assert evaluate([], healthy, profile)["status"] == "pass"
    assert evaluate([], [*healthy, age(300_000_000, 300_000_000)], profile)["status"] == "fail"
    assert evaluate([], [age(0, 0), age(3_000_000_000, 0)], profile)["status"] == "inconclusive"


def test_one_camera_cannot_supply_another_cameras_required_sample_count():
    events = []
    for channel, count in [("left", 10), ("right", 1)]:
        for sequence in range(count):
            context = TraceContext("run", "camera", channel, sequence, 0, "clock")
            events.extend(
                dict(event=stage, mono_ns=t, clock_domain="clock", context=asdict(context))
                for stage, t in [("acquire", 0), ("display", 1)]
            )
    profile = {
        "schema": "dimos.budgets/v1",
        "paths": {
            "video": {
                "source": "acquire",
                "destination": "display",
                "channels": ["left", "right"],
                "max_ms": 100,
                "min_samples": 5,
            }
        },
    }
    result = evaluate([], events, profile)
    assert result["status"] == "inconclusive"
    assert result["checks"][0]["samples_per_channel"] == {"left": 10, "right": 1}


def test_unrelated_heartbeat_cannot_certify_an_hour_of_missing_camera_observations():
    events = [
        dict(
            event="freshness.age",
            watch="video",
            process="headset",
            clock_domain="c",
            mono_ns=t,
            age_ns=10_000_000,
        )
        for t in (1_000_000_000, 1_100_000_000)
    ]
    events.append(
        dict(event="heartbeat", process="headset", clock_domain="c", mono_ns=3_602_000_000_000)
    )
    profile = {
        "schema": "dimos.budgets/v1",
        "min_duration_s": 3600,
        "freshness": {"video": {"max_ms": 100}},
    }

    result = evaluate([], events, profile)

    assert result["status"] == "inconclusive"
    assert result["checks"][0]["max_observation_gap_ms"] == 3_600_900


def test_fresh_laptop_cannot_hide_headset_observer_disappearing():
    events = [
        dict(
            event="freshness.age",
            watch="video",
            process=process,
            clock_domain="c",
            mono_ns=t,
            age_ns=0,
        )
        for process, times in [
            ("headset", (0, 100_000_000)),
            ("laptop", range(0, 2_100_000_000, 100_000_000)),
        ]
        for t in times
    ]
    events.append(
        dict(event="heartbeat", process="headset", clock_domain="c", mono_ns=2_000_000_000)
    )

    result = evaluate(
        [], events, {"schema": "dimos.budgets/v1", "freshness": {"video": {"max_ms": 100}}}
    )

    assert result["status"] == "inconclusive"
    assert {c["process"]: c["complete"] for c in result["checks"][0]["coverage"]} == {
        "headset": False,
        "laptop": True,
    }


def test_explicit_watch_stop_ends_required_coverage():
    events = [
        dict(event="freshness.age", watch="video", clock_domain="c", mono_ns=t, age_ns=0)
        for t in (0, 100_000_000)
    ]
    events.extend(
        [
            dict(
                event="freshness.observe",
                watch="video",
                active=False,
                clock_domain="c",
                mono_ns=200_000_000,
            ),
            dict(event="heartbeat", clock_domain="c", mono_ns=10_000_000_000),
        ]
    )

    result = evaluate(
        [], events, {"schema": "dimos.budgets/v1", "freshness": {"video": {"max_ms": 100}}}
    )

    assert result["status"] == "pass"


def test_checkpoint_order_is_chronological_but_triggered_incident_loss_is_preserved():
    profile = {"schema": "dimos.budgets/v1", "stages": {"ik": {"p99_ms": 10, "min_samples": 1}}}
    old = dict(
        process="p",
        clock_domain="c",
        reason="checkpoint",
        captured_ns=1,
        health={"capture_window_complete": False},
    )
    new = {**old, "captured_ns": 2, "health": {"capture_window_complete": True}}

    assert evaluate([new, old], spans("ik", 1_000_000), profile)["status"] == "pass"
    assert (
        evaluate([new, {**old, "reason": "camera_stall"}], spans("ik", 1_000_000), profile)[
            "status"
        ]
        == "inconclusive"
    )
