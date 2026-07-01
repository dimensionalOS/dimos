#!/usr/bin/env python3
# Unit tests for the PriorityOutbox scheduler in qos.py — the core QoS enforcement claim: weighted
# round-robin drain ratios, the non-starvation floor for the lowest class, conflation (latest-only)
# vs bounded keep_last, per-class topic round-robin, and control-beats-data. Loop-free where possible
# (drive _pick/put directly); one asyncio test covers get() blocking + wake.
#
# Run: uv run pytest dimos/web/dimoscope/gateway/tests/test_outbox.py -q
import asyncio
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))

from qos import PriorityOutbox


def _fill(ob: PriorityOutbox, topic: str, prio: int, n: int, depth: int = 64) -> None:
    for i in range(n):
        ob.put_data(topic, prio, False, depth, f"{topic}:{i}".encode())


def _drain(ob: PriorityOutbox, n: int) -> list:
    out = []
    for _ in range(n):
        item = ob._pick()
        if item is None:
            break
        out.append(item)
    return out


def test_higher_class_served_first():
    ob = PriorityOutbox()
    _fill(ob, "/lo", 0, 1)
    _fill(ob, "/hi", 3, 1)
    assert _drain(ob, 2) == [b"/hi:0", b"/lo:0"]


def test_wrr_ratio_and_low_class_floor():
    # Saturate the top and bottom classes: per credit cycle the drain must be 8×class-3 then
    # 1×class-0 (the non-starving floor), repeating — bulk is throttled but never starved.
    ob = PriorityOutbox()
    _fill(ob, "/hi", 3, 20)
    _fill(ob, "/lo", 0, 20)
    picked = _drain(ob, 18)
    expect = [b"/hi:%d" % i for i in range(8)] + [b"/lo:0"]
    expect += [b"/hi:%d" % i for i in range(8, 16)] + [b"/lo:1"]
    assert picked == expect


def test_conflation_keeps_only_latest():
    ob = PriorityOutbox()
    for payload in (b"a", b"b", b"c"):
        ob.put_data("/img", 0, True, 1, payload)  # best-effort → latest-wins slot
    assert _drain(ob, 5) == [b"c"]


def test_reliable_keep_last_bounds_the_deque():
    ob = PriorityOutbox()
    for i in range(5):
        ob.put_data("/r", 1, False, 3, f"{i}".encode())
    assert _drain(ob, 5) == [b"2", b"3", b"4"]  # keep_last 3: oldest shed, order preserved


def test_round_robin_across_topics_within_a_class():
    ob = PriorityOutbox()
    for i in range(2):
        ob.put_data("/a", 2, False, 8, f"a{i}".encode())
        ob.put_data("/b", 2, False, 8, f"b{i}".encode())
    assert _drain(ob, 4) == [b"a0", b"b0", b"a1", b"b1"]  # no topic monopolizes its class


def test_control_beats_data():
    ob = PriorityOutbox()
    _fill(ob, "/bulk", 0, 3)
    ob.put_control('{"op":"topic"}')
    assert _drain(ob, 1) == ['{"op":"topic"}']


def test_empty_pick_returns_none_and_memory_is_freed():
    ob = PriorityOutbox()
    _fill(ob, "/a", 1, 2)
    _drain(ob, 2)
    assert ob._pick() is None
    assert all(not cls for cls in ob._cls)  # drained topics are deleted, not left empty


def test_get_blocks_until_put_wakes_it():
    async def run():
        ob = PriorityOutbox()
        task = asyncio.create_task(ob.get())
        await asyncio.sleep(0.01)
        assert not task.done()
        ob.put_data("/a", 1, False, 4, b"x")
        assert await asyncio.wait_for(task, 1) == b"x"

    asyncio.run(run())
