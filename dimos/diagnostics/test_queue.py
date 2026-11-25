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

"""Provenance must survive the real thread handoffs, not just synchronous callbacks."""

from contextlib import ExitStack
from queue import Queue

from reactivex.scheduler import ThreadPoolScheduler
from reactivex.subject import Subject

from dimos.diagnostics.analysis import load_bundle
from dimos.diagnostics.model import context_scope, current_context
from dimos.diagnostics.queue import capture, delivery, discard
from dimos.utils.reactive import backpressure


def test_scheduler_handoff_preserves_each_sample_context(trace_runtime):
    results = Queue()
    with ExitStack() as cleanup:
        scheduler = ThreadPoolScheduler(max_workers=1)
        cleanup.callback(scheduler.executor.shutdown, wait=True)
        source = Subject()
        cleanup.callback(source.dispose)
        subscription = backpressure(source, scheduler, drop_unprocessed=False).subscribe(
            lambda value: results.put((value, current_context()))
        )
        cleanup.callback(subscription.dispose)
        contexts = [trace_runtime.sample("hand") for _ in range(3)]
        for context in contexts:
            with context_scope(context):
                source.on_next("identical payload")
        assert [results.get(timeout=3) for _ in contexts] == [
            ("identical payload", c) for c in contexts
        ]
        assert current_context() is None
    trace_runtime.close()
    _, events = load_bundle(trace_runtime.collector.directory)
    assert len([event for event in events if event["event"] == "queue.delay"]) == 3


def test_supersession_names_the_discarded_sample_without_corrupting_the_winner(trace_runtime):
    old, new = trace_runtime.sample("hand"), trace_runtime.sample("hand")
    with context_scope(old):
        pending = capture("old")
    with context_scope(new):
        discard(pending, "test.latest")
        pending = capture("new")
    with delivery(pending, "test.latest") as value:
        assert (value, current_context()) == ("new", new)
    trace_runtime.close()
    _, events = load_bundle(trace_runtime.collector.directory)
    dropped = [e for e in events if e["event"] == "queue.discard"]
    assert [(e["context"]["sequence"], e["reason"]) for e in dropped] == [
        (old.sequence, "superseded")
    ]
