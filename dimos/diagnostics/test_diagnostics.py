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

"""Behavioral regression coverage for identity, timing and bounded capture."""

from asyncio import CancelledError
from concurrent.futures import ThreadPoolExecutor

import pytest

from dimos.core.stream import Out
from dimos.diagnostics.analysis import analyze, load_bundle, perfetto
from dimos.diagnostics.collector import CaptureLimits, Collector, EventQueue
from dimos.diagnostics.envelope import pack, unpack
from dimos.diagnostics.model import ClockMapping, TraceContext, context_scope, current_context
from dimos.diagnostics.runtime import Runtime, publication, span


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    runtime = Runtime("run", tmp_path)
    monkeypatch.setattr("dimos.diagnostics.runtime._runtime", runtime)
    monkeypatch.setattr("dimos.diagnostics.runtime._mode", "trace")
    yield runtime
    runtime.close()


def sample(seq=1):
    return TraceContext("run", "publisher", "hand", seq, 100, "clock")


def test_cached_inputs_preserve_multiple_parents_and_restore_scope(runtime):
    first, second = sample(), sample(2)
    with context_scope(first):
        with publication("target", parents=[first, second]) as output:
            assert output.parents == (first.sample_id, second.sample_id)
            assert output.sample_id not in output.parents
        assert current_context() == first
    assert current_context() is None


def test_identity_survives_envelope_with_identical_payloads():
    first = pack("link", "bytes", sample(), b"unchanged")
    second = pack("link", "bytes", sample(2), b"unchanged")
    assert unpack(first, "link", "bytes") == (sample(), b"unchanged")
    assert unpack(second, "link", "bytes")[0].sample_id != sample().sample_id
    with pytest.raises(ValueError, match="mismatch"):
        unpack(first, "wrong", "bytes")
    with pytest.raises(ValueError):
        unpack(b"legacy payload", "link", "bytes")


def test_same_class_and_port_on_two_instances_have_independent_sequences(runtime):
    first, second = Out(int, "pose", object()), Out(int, "pose", object())
    received = []
    for stream in (first, second):
        unsubscribe = stream.subscribe(lambda _: received.append(current_context()))
        try:
            stream.publish(1)
        finally:
            unsubscribe()
    assert [context.sequence for context in received] == [1, 1]
    assert len({context.sample_id for context in received}) == 2


def test_clock_asymmetry_reports_bounds_and_expiry():
    # A sent=0, B received=130 (100 offset +30 transit), reply=140,
    # A received=50 (10 return transit): estimated offset=110, true=100.
    mapping = ClockMapping.exchange("A", "B", 0, 130, 140, 50, valid_for_ns=100, drift_ppm=0)
    assert (mapping.lower_ns, mapping.upper_ns) == (90, 130)
    assert mapping.latency_bounds(0, 130, 50) == (0, 40)
    assert mapping.latency_bounds(0, 130, 151) is None
    with pytest.raises(ValueError):
        ClockMapping.exchange("A", "B", 0, 100, 200, 10, 100)


def test_error_spans_restore_context_and_trigger_incident(runtime):
    with pytest.raises(RuntimeError), context_scope(sample()), span("ik"):
        raise RuntimeError("blocked")
    runtime.close()
    manifests, events = load_bundle(runtime.collector.directory)
    report = analyze(manifests, events)
    assert report["faults"][0]["reason"] == "RuntimeError"
    assert report["qualification"] == "incomplete"
    assert report["stages"]["ik"]["n"] == 1
    assert current_context() is None


def test_cancelled_handler_is_distinguished_from_processing_failure(runtime):
    with pytest.raises(CancelledError), span("handler"):
        raise CancelledError()
    runtime.close()
    manifests, events = load_bundle(runtime.collector.directory)
    assert analyze(manifests, events)["faults"] == []
    assert [e["outcome"] for e in events if e["event"] == "handler.end"] == ["cancelled"]


def test_ring_eviction_is_explicit_and_storage_is_bounded(tmp_path):
    collector = Collector(
        tmp_path,
        {"schema": "dimos.incident/v1"},
        CaptureLimits(ring_bytes=400, storage_bytes=2000, queue_events=100, post_seconds=0),
    )
    try:
        for i in range(40):
            collector.emit({"event": "frame", "mono_ns": i, "padding": "x" * 100})
    finally:
        collector.close()
    manifests, events = load_bundle(tmp_path)
    assert manifests[0]["health"]["ring_evictions"] > 0
    assert len(events) < 40
    assert sum(p.stat().st_size for p in tmp_path.glob("incident-*.jsonl")) <= 2000


def test_missing_and_cross_clock_spans_never_report_latency():
    events = [
        dict(event="ik.start", process="a", span_id="one", clock_domain="A", mono_ns=100),
        dict(event="ik.end", process="a", span_id="one", clock_domain="B", mono_ns=200),
    ]
    result = analyze([], events)
    assert result["stages"] == {}
    assert result["qualification"] == "incomplete"
    assert result["unmatched_spans"] == 1
    assert analyze([], [])["qualification"] == "incomplete"


def test_injected_queue_delay_is_not_encode_work():
    def pair(stage, begin, end, span_id):
        return [
            dict(
                event=stage + "." + suffix,
                process="p",
                span_id=span_id,
                clock_domain="c",
                mono_ns=t,
            )
            for suffix, t in [("start", begin), ("end", end)]
        ]

    events = pair("video.queue", 0, 20_000_000, "q") + pair(
        "video.encode", 20_000_000, 25_000_000, "e"
    )
    result = analyze([], events)
    assert result["stages"]["video.queue"]["p95_ms"] == 20
    assert result["stages"]["video.encode"]["p95_ms"] == 5
    exported = perfetto(events)
    assert [e["ph"] for e in exported["traceEvents"]] == ["b", "e", "b", "e"]


def test_trace_emission_returns_while_observer_queue_is_locked(runtime):
    collector = runtime.collector
    with ThreadPoolExecutor(max_workers=1) as pool:
        with collector.queue.mutex:
            before = collector.dropped
            emitted = pool.submit(collector.emit, {"event": "robot.callback"})
            emitted.result(timeout=1)
            assert collector.dropped == before + 1


def test_observer_queue_rejects_capacity_before_copying_metadata_and_recovers():
    queue = EventQueue(1)
    event = {"event": "frame", "labels": ["original"]}
    assert queue.offer(event)
    event["labels"].append("changed")
    # Full queues reject even invalid metadata without doing producer-side copying.
    assert not queue.offer({"payload": bytearray(10)})
    assert queue.get_nowait() == {"event": "frame", "labels": ["original"]}
    with pytest.raises(ValueError):
        queue.offer({"payload": bytearray(10)})
    assert queue.offer({"event": "recovered"})
    assert queue.get_nowait() == {"event": "recovered"}
