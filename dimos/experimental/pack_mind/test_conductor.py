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

"""Unit tests for the PACK MIND conductor.

All tests run on CPU with no hardware and no network: the conductor is driven in
``mock=True`` mode, where ``call_tool`` logs an event instead of issuing HTTP. The
state-machine code path is identical to real hardware, so these tests guard the
exact logic that runs at the venue.
"""

from __future__ import annotations

import argparse
import dataclasses
import time

import pytest

from dimos.experimental.pack_mind.conductor import (
    Conductor,
    Dog,
    MemoryEvent,
    _extract_text,
    _parse_dog,
)


def _mock_conductor() -> Conductor:
    dogs = [
        Dog(id="alpha", name="Alpha", mcp_url="http://mock/alpha/mcp", role="scout"),
        Dog(id="bravo", name="Bravo", mcp_url="http://mock/bravo/mcp", role="guide"),
    ]
    return Conductor(dogs, mock=True)


def _wait_for_mission(c: Conductor, target: str, timeout: float = 3.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if c.snapshot()["mission"] == target:
            return True
        time.sleep(0.02)
    return False


# -- blackboard / immutability ----------------------------------------------


def test_memory_event_is_frozen():
    event = MemoryEvent(id="e1", ts="t", robot="alpha", type="x", text="y")

    with pytest.raises(dataclasses.FrozenInstanceError):
        event.text = "mutated"  # type: ignore[misc]


def test_memory_event_carries_no_coordinate_fields():
    # The whole thesis: dogs share meaning, never geometry.
    forbidden = {"x", "y", "z", "pose", "coordinate", "position"}
    fields = set(MemoryEvent.__dataclass_fields__)

    assert forbidden.isdisjoint(fields)


def test_append_event_appends_and_returns_event():
    c = _mock_conductor()

    event = c.append_event("alpha", "object_found", "found it", object_="bag", zone="zone_a")

    assert event.robot == "alpha"
    assert event.zone == "zone_a"
    assert c.snapshot()["events"][0]["id"] == event.id


# -- semantic recall (the core trick) ---------------------------------------


def test_latest_zone_for_returns_most_recent_match():
    c = _mock_conductor()
    c.append_event("alpha", "object_found", "a", object_="bag", zone="zone_a")
    c.append_event("alpha", "object_found", "b", object_="bag", zone="zone_c")

    assert c._latest_zone_for("bag") == "zone_c"


def test_latest_zone_for_unknown_object_is_none():
    c = _mock_conductor()

    assert c._latest_zone_for("nonexistent") is None


def test_ask_where_answers_from_shared_memory():
    c = _mock_conductor()
    # Alpha records the find; Bravo must answer from the SHARED blackboard.
    c.inject_found("alpha", "red backpack", "zone_b")

    answer = c.ask_where("bravo", "red backpack")

    assert "Alpha" in answer
    assert "zone_b" in answer


def test_ask_where_with_no_memory_admits_ignorance():
    c = _mock_conductor()

    answer = c.ask_where("bravo", "red backpack")

    assert "no pack memory" in answer.lower()


# -- movement lock (only one dog moves) -------------------------------------


def test_send_dog_blocked_when_move_lock_held():
    c = _mock_conductor()
    c.inject_found("alpha", "red backpack", "zone_b")
    c._move_lock.acquire()

    try:
        ok = c.send_dog_to_memory("bravo", "red backpack")
    finally:
        c._move_lock.release()

    assert ok is False
    assert any(e.type == "blocked" for e in c._events)


def test_send_dog_without_memory_refuses():
    c = _mock_conductor()

    ok = c.send_dog_to_memory("bravo", "red backpack")

    assert ok is False
    assert any(e.type == "blocked" for e in c._events)


def test_send_dog_releases_lock_after_navigation():
    c = _mock_conductor()
    c.inject_found("alpha", "red backpack", "zone_b")

    assert c.send_dog_to_memory("bravo", "red backpack") is True
    # async navigation finishes fast in mock; lock must be released for the next dog.
    deadline = time.time() + 3.0
    while c._move_lock.locked() and time.time() < deadline:
        time.sleep(0.02)

    assert not c._move_lock.locked()
    assert any(e.type == "arrived" for e in c._events)


# -- full mission sequence ---------------------------------------------------


def test_full_mock_mission_reaches_done():
    c = _mock_conductor()

    c.start_act1()
    assert c.snapshot()["mission"] == "SCOUT_ALPHA"

    c.inject_found("alpha", "red backpack", "zone_b")
    assert c.snapshot()["mission"] == "FOUND_EVENT_RECORDED"

    c.ask_where("bravo", "red backpack")
    assert c.snapshot()["mission"] == "QUERY_BRAVO"

    assert c.send_dog_to_memory("bravo", "red backpack") is True
    assert c.snapshot()["mission"] == "BRAVO_NAVIGATING"

    c.verify_at_zone("bravo", "red backpack")
    assert _wait_for_mission(c, "DONE")


def test_emergency_stop_resets_to_idle():
    c = _mock_conductor()
    c.start_act1()
    c.inject_found("alpha", "red backpack", "zone_b")

    c.emergency_stop()

    snap = c.snapshot()
    assert snap["mission"] == "IDLE"
    assert all(d["status"] == "idle" for d in snap["roster"])


# -- arg parsing -------------------------------------------------------------


def test_parse_dog_default_port():
    dog = _parse_dog("alpha=10.0.0.10")

    assert dog.mcp_url == "http://10.0.0.10:9990/mcp"
    assert dog.role == "scout"


def test_parse_dog_explicit_port_and_case_insensitive():
    dog = _parse_dog("BRAVO=1.2.3.4:7000")

    assert dog.id == "bravo"
    assert dog.mcp_url == "http://1.2.3.4:7000/mcp"


def test_parse_dog_rejects_unknown_id():
    with pytest.raises(argparse.ArgumentTypeError):
        _parse_dog("zulu=1.2.3.4")


# -- MCP response shape tolerance --------------------------------------------


@pytest.mark.parametrize(
    "data,expected",
    [
        ({"result": {"content": [{"text": "hi"}]}}, "hi"),
        ({"content": [{"text": "a"}, {"text": "b"}]}, "a b"),
        ({"result": {"content": []}}, None),
        ({"result": {}}, None),
        ("not a dict", None),
        ({"result": {"content": [{"no_text": 1}]}}, None),
    ],
)
def test_extract_text_tolerates_shape_variation(data, expected):
    assert _extract_text(data) == expected
