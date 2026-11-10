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

import threading

import pytest

from dimos.web.relay_bridge.topic_guard import GuardedPublishers
from dimos.web.relay_bridge.topic_protocol import (
    MAX_TOPIC_GUARD_BYTES,
    MAX_TOPIC_GUARDS,
    TopicGuardClosed,
    TopicGuardCommandMeta,
    TopicGuardHeartbeat,
    TopicGuardOpenMeta,
    TopicGuardReady,
    TopicGuardResult,
    TopicGuardTicket,
)


@pytest.fixture
def spec():
    return TopicGuardOpenMeta(
        action="open",
        id="@p:1",
        handle="@p:1",
        source="workspace",
        gen=7,
        key="robot/cmd_vel",
        encoding="application/octet-stream",
        group="controller",
        owner="viewer-1",
        timeoutMs=400,
        takeover=False,
    )


@pytest.fixture
def guard(mocker):
    now = [10.0]
    messages = []
    put = mocker.Mock()
    manager = GuardedPublishers(put, messages.append, clock=lambda: now[0])
    try:
        yield manager, now, messages, put
    finally:
        manager.close()


def command(spec, ticket=1):
    return TopicGuardCommandMeta(
        action="command",
        source=spec.source,
        gen=spec.gen,
        handle=spec.handle,
        ticket=ticket,
    )


def test_ready_follows_safe_stop_and_commands_are_checked_puts(guard, spec):
    manager, now, messages, put = guard
    manager.open(spec, b"STOP")
    manager.poll()

    put.assert_called_once_with(spec.key, b"STOP", spec.encoding)
    assert messages == [
        TopicGuardReady(id=spec.id, handle=spec.handle, gen=7, ticket=1, timeoutMs=400)
    ]

    manager.command(command(spec), 1, b"MOVE")
    manager.poll()

    put.assert_called_with(spec.key, b"MOVE", spec.encoding)
    assert (messages[-1].outcome, messages[-1].seq) == ("accepted", 1)


def test_closed_provider_reports_lifecycle_failure_instead_of_invalid_stop(guard, spec):
    manager, _, messages, put = guard
    manager.close()
    manager.open(spec, b"STOP")
    assert messages == [TopicGuardClosed(handle=spec.handle, gen=7, code="provider_closed")]
    put.assert_not_called()


def test_expiry_stops_even_when_heartbeats_continue_without_commands(guard, spec):
    manager, now, messages, put = guard
    manager.open(spec, b"STOP")
    manager.poll()
    manager.command(command(spec), 1, b"MOVE")
    manager.poll()
    now[0] += 0.2
    manager.heartbeat(TopicGuardHeartbeat(handle=spec.handle, gen=7, ticket=1))
    assert isinstance(messages[-1], TopicGuardTicket)
    now[0] += 0.21
    manager.poll()

    assert messages[-1] == TopicGuardClosed(handle=spec.handle, gen=7, code="guard_expired")
    assert [call.args[1] for call in put.call_args_list] == [b"STOP", b"MOVE", b"STOP"]
    manager.heartbeat(TopicGuardHeartbeat(handle=spec.handle, gen=7, ticket=2))
    manager.command(command(spec, ticket=2), 2, b"LATE")
    manager.poll()
    assert put.call_count == 3


def test_delayed_command_cannot_extend_its_provider_ticket_deadline(guard, spec):
    manager, now, messages, put = guard
    manager.open(spec, b"STOP")
    manager.poll()
    now[0] += 0.39
    manager.command(command(spec), 1, b"MOVE")
    manager.poll()
    now[0] += 0.02
    manager.poll()

    assert [call.args[1] for call in put.call_args_list] == [b"STOP", b"MOVE", b"STOP"]
    assert messages[-1].code == "guard_expired"


def test_one_pending_command_per_handle_and_stop_wins_over_it(guard, spec):
    manager, now, messages, put = guard
    manager.open(spec, b"STOP")
    manager.poll()
    manager.command(command(spec), 1, b"OLD")
    manager.command(command(spec), 2, b"NEW")
    assert (messages[-1].seq, messages[-1].code) == (1, "superseded")
    manager.stop(spec.handle, 7)
    manager.stop(spec.handle, 7)
    manager.poll()
    manager.stop(spec.handle, 7)
    manager.poll()

    assert [call.args[1] for call in put.call_args_list] == [b"STOP", b"STOP"]
    assert [m.code for m in messages if isinstance(m, TopicGuardResult)] == [
        "superseded",
        "released",
    ]
    assert messages[-1] == TopicGuardClosed(handle=spec.handle, gen=7, code="released")


def test_stale_generations_sequences_and_expired_tickets_do_not_publish(guard, spec):
    manager, now, messages, put = guard
    manager.open(spec, b"STOP")
    manager.poll()
    manager.command(command(spec), 2, b"MOVE")
    manager.poll()
    manager.command(command(spec), 1, b"OLD")
    manager.command(command(spec).model_copy(update={"gen": 6}), 3, b"OLDGEN")
    manager.stop(spec.handle, 6)
    manager.poll()
    now[0] += 0.41
    manager.command(command(spec), 4, b"EXPIRED")
    manager.poll()

    assert [call.args[1] for call in put.call_args_list] == [b"STOP", b"MOVE", b"STOP"]
    assert [m.code for m in messages if isinstance(m, TopicGuardResult)] == [
        "",
        "stale_guard_command",
        "stale_guard_command",
    ]


def test_competing_controller_requires_takeover_and_stops_entire_group(guard, spec):
    manager, now, messages, put = guard
    manager.open(spec, b"STOP-DRIVE")
    arm = spec.model_copy(update={"id": "@p:2", "handle": "@p:2", "key": "robot/arm"})
    manager.open(arm, b"STOP-ARM")
    manager.poll()
    other = spec.model_copy(update={"id": "@p:3", "handle": "@p:3", "owner": "viewer-2"})
    manager.open(other, b"STOP-NEW")
    assert messages[-1] == TopicGuardClosed(handle="@p:3", gen=7, code="guard_owned")
    manager.open(other.model_copy(update={"takeover": True}), b"STOP-NEW")
    manager.poll()

    assert [call.args[1] for call in put.call_args_list] == [
        b"STOP-DRIVE",
        b"STOP-ARM",
        b"STOP-DRIVE",
        b"STOP-ARM",
        b"STOP-NEW",
    ]
    assert [
        m.handle for m in messages if isinstance(m, TopicGuardClosed) and m.code == "taken_over"
    ] == ["@p:1", "@p:2"]
    assert messages[-1] == TopicGuardReady(id="@p:3", handle="@p:3", gen=7, ticket=1, timeoutMs=400)


def test_unknown_stop_outcome_does_not_arm_a_replacement(guard, spec):
    manager, now, messages, put = guard
    manager.open(spec, b"STOP")
    manager.poll()
    put.side_effect = RuntimeError("Zenoh session closed")
    other = spec.model_copy(
        update={"id": "@p:2", "handle": "@p:2", "owner": "other", "takeover": True}
    )
    manager.open(other, b"STOP")
    manager.poll()

    assert [m.handle for m in messages if isinstance(m, TopicGuardReady)] == ["@p:1"]
    assert messages[-1] == TopicGuardClosed(handle="@p:2", gen=7, code="guard_stop_failed")


def test_a_failed_group_stop_holds_the_other_guard_until_it_is_stopped(guard, spec):
    manager, now, messages, put = guard
    manager.open(spec, b"STOP-DRIVE")
    arm = spec.model_copy(update={"id": "@p:2", "handle": "@p:2", "key": "robot/arm"})
    manager.open(arm, b"STOP-ARM")
    manager.poll()
    manager.command(command(spec), 1, b"DRIVE")

    def arm_stop_fails(key, payload, encoding):
        if key == "robot/arm":
            raise RuntimeError("Zenoh session closed")

    put.side_effect = arm_stop_fails
    manager.stop(arm.handle, arm.gen)
    assert manager.poll() == 0.0
    assert messages[-1] == TopicGuardClosed(handle="@p:2", gen=7, code="guard_stop_failed")
    manager.poll()

    assert [call.args[1] for call in put.call_args_list] == [
        b"STOP-DRIVE",
        b"STOP-ARM",
        b"STOP-ARM",
        b"STOP-DRIVE",
    ]
    result = messages[-2]
    assert isinstance(result, TopicGuardResult)
    assert (result.handle, result.seq, result.outcome, result.code) == (
        "@p:1",
        1,
        "rejected",
        "guard_stop_failed",
    )
    assert messages[-1] == TopicGuardClosed(handle="@p:1", gen=7, code="guard_stop_failed")


def test_handle_and_stop_payload_bounds(guard, spec):
    manager, now, messages, put = guard
    manager.open(spec, b"x" * (MAX_TOPIC_GUARD_BYTES + 1))
    assert messages[-1].code == "invalid_guard_stop"
    for n in range(MAX_TOPIC_GUARDS):
        manager.open(
            spec.model_copy(update={"id": f"@p:{n}", "handle": f"@p:{n}", "key": f"cmd/{n}"}),
            b"STOP",
        )
    manager.open(spec.model_copy(update={"handle": "@p:extra"}), b"STOP")

    assert messages[-1].code == "guard_limit"
    manager.poll()
    assert put.call_count == MAX_TOPIC_GUARDS


def test_watchdog_expires_without_an_event_loop_turn(spec):
    stopped = threading.Event()
    payloads = []

    def emit(message):
        if isinstance(message, TopicGuardClosed):
            stopped.set()

    manager = GuardedPublishers(lambda key, data, encoding: payloads.append(data), emit)
    try:
        manager.start()
        manager.open(spec.model_copy(update={"timeoutMs": 100}), b"STOP")
        # Deliberately block this thread. The independent watchdog still runs.
        assert stopped.wait(timeout=1)
        assert payloads == [b"STOP", b"STOP"]
    finally:
        manager.close()
