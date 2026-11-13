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

from collections.abc import Callable, Iterator
from contextlib import ExitStack

import pytest

from dimos.web.relay_bridge.topic_mailbox import TopicIngressBudget, TopicMailbox, TopicSample


@pytest.fixture
def mailboxes() -> Iterator[Callable[..., TopicMailbox]]:
    with ExitStack() as stack:

        def create(*args, **kwargs):
            mailbox = TopicMailbox(*args, **kwargs)
            stack.callback(mailbox.close)
            return mailbox

        yield create


def sample(key: str, payload: bytes) -> TopicSample:
    return TopicSample(key, "application/octet-stream", payload, ingress_ts=123)


def test_latest_selects_newest_before_rate_gate_and_preserves_quiet_keys(mailboxes):
    budget = TopicIngressBudget()
    wakes = []
    mailbox = mailboxes("latest", 10, budget, lambda: wakes.append(True))
    first = sample("camera", b"first")
    quiet = sample("odom", b"quiet")
    mailbox.offer(first)
    mailbox.offer(quiet)
    batch, delay = mailbox.take(10)
    assert batch == [first, quiet]
    assert delay is None
    for item in batch:
        mailbox.release(item)
    for n in range(100):
        mailbox.offer(sample("camera", str(n).encode()))
    held, delay = mailbox.take(10.05)
    assert held == []
    assert delay == pytest.approx(0.05)
    newest, delay = mailbox.take(10.1)
    assert newest == [sample("camera", b"99")]
    assert newest[0].ingress_ts == 123
    assert mailbox.dropped == 99
    assert len(wakes) == 2  # one scheduled notification per burst, not per sample
    for item in newest:
        mailbox.release(item)
    assert budget.used == 0


def test_reliable_order_and_in_progress_samples_remain_within_aggregate_budget(mailboxes):
    event = sample("event", b"one")
    budget = TopicIngressBudget(limit=3 * event.size)
    first = mailboxes("reliable", 1000, budget, lambda: None)
    second = mailboxes("reliable", 1000, budget, lambda: None)
    first.offer(event)
    first.offer(sample("other", b"two"))
    batch, _ = first.take(0)
    assert [item.payload for item in batch] == [b"one", b"two"]
    second.offer(event)
    assert budget.used == 3 * event.size
    assert budget.peak == 3 * event.size
    second.offer(event)
    assert second.error == "topic_overflow"
    for item in batch:
        first.release(item)
    assert budget.used == event.size
    second.close()
    assert budget.used == 0
    assert budget.peak == 3 * event.size


def test_shared_rate_changes_select_newest_without_reopening_or_losing_quiet_keys(mailboxes):
    budget = TopicIngressBudget()
    mailbox = mailboxes("latest", 5, budget, lambda: None)
    mailbox.offer(sample("camera", b"first"))
    first, _ = mailbox.take(10)
    mailbox.release(first[0])
    mailbox.offer(sample("camera", b"second"))
    mailbox.set_rate(30)
    batch, _ = mailbox.take(10.04)
    assert [item.payload for item in batch] == [b"second"]
    mailbox.release(batch[0])
    mailbox.set_rate(5)
    mailbox.offer(sample("camera", b"obsolete"))
    mailbox.offer(sample("camera", b"last"))
    mailbox.offer(sample("quiet", b"unrelated"))
    batch, delay = mailbox.take(10.08)
    assert [item.payload for item in batch] == [b"unrelated"]
    mailbox.release(batch[0])
    # The gate is anchored to its schedule (due 10 + 1/30 after the rate
    # change, then one more period), not to the late take at 10.04.
    assert delay == pytest.approx(10 + 2 / 30 + 0.2 - 1 / 30 - 10.08)
    batch, _ = mailbox.take(10.25)
    assert [item.payload for item in batch] == [b"last"]
    mailbox.release(batch[0])
    assert budget.used == 0


def test_distinct_wildcard_keys_are_bounded_even_after_prior_frames_drain(mailboxes):
    budget = TopicIngressBudget()
    mailbox = mailboxes("latest", 30, budget, lambda: None)
    for n in range(128):
        mailbox.offer(sample(f"key/{n}", b"x"))
        batch, _ = mailbox.take(0)
        for item in batch:
            mailbox.release(item)
    mailbox.offer(sample("key/overflow", b"x"))
    assert mailbox.error == "topic_key_limit"
    assert budget.used == 0


def test_only_dense_reliable_arrivals_request_coalescing_and_keep_every_event(mailboxes):
    budget = TopicIngressBudget()
    mailbox = mailboxes("reliable", 1000, budget, lambda: None)
    values = [
        TopicSample("event", "application/json", str(index).encode(), ingress_ts=stamp)
        for index, stamp in enumerate([1.0, 1.01, 1.02, 1.021, 1.022, 1.03, 1.2, 1.201, 1.202, 0.1])
    ]
    coalesced = []
    delivered = []
    for item in values:
        mailbox.offer(item)
        coalesced.append(mailbox.coalesce)
        batch, _ = mailbox.take(item.ingress_ts)
        delivered.extend(batch)
        for received in batch:
            mailbox.release(received)

    # A backwards local clock adjustment also ends the old burst window.
    assert coalesced == [False, False, False, False, True, True, False, False, True, False]
    assert delivered == values
    assert budget.used == 0


def test_same_rate_source_loses_no_frames_to_late_wakes(mailboxes):
    """A 30 Hz camera through a 30 Hz gate whose wakes run a millisecond late:
    a gate re-anchored on each take drifts behind the source and drops one
    frame per period of accumulated lateness; one anchored to its schedule
    does not."""
    period = 1 / 30
    lateness = 0.001
    mailbox = mailboxes("latest", 30, TopicIngressBudget(), lambda: None)
    taken = 0
    timer = None
    frame = 0
    while frame < 300:
        arrival = frame * period
        if timer is not None and timer < arrival:
            batch, delay = mailbox.take(timer + lateness)
            taken += len(batch)
            for item in batch:
                mailbox.release(item)
            timer = None if delay is None else timer + lateness + delay
            continue
        mailbox.offer(sample("camera", str(frame).encode()))
        frame += 1
        batch, delay = mailbox.take(arrival + 0.0003)
        taken += len(batch)
        for item in batch:
            mailbox.release(item)
        timer = None if delay is None else arrival + 0.0003 + delay
    batch, _ = mailbox.take(400 * period)
    taken += len(batch)
    assert mailbox.dropped == 0
    assert taken == 300
