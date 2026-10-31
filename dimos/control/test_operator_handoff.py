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

"""OperatorHandoff: the agent's request, the event log, and the restart of autonomy."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import asdict
import json
from pathlib import Path
import time
from typing import cast
from unittest.mock import MagicMock

import pytest

from dimos.control.operator_handoff import OPERATOR_EVENTS_FILE, OperatorHandoff
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    OperatorHoldStatus,
)
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.String import String


def _hold(on: bool, route: str = "manual", reason: str = "arm stuck") -> OperatorHoldStatus:
    if not on:
        return OperatorHoldStatus(on=False, stamp=2.0)
    return OperatorHoldStatus(on=True, route=route, reason=reason, started_at=1.0, stamp=1.0)


@pytest.fixture
def handoff(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[OperatorHandoff]:
    monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(tmp_path))
    module = OperatorHandoff()
    module._coordinator = MagicMock()
    try:
        yield module
    finally:
        module.stop()


def _wait_for(condition: Callable[[], bool]) -> None:
    deadline = time.monotonic() + 5.0
    while not condition() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert condition()


def _actions(module: OperatorHandoff) -> list[tuple[str, str]]:
    return [(event["action"], event["kind"]) for event in module.get_operator_events()]


def _coordinator(module: OperatorHandoff) -> MagicMock:
    return cast("MagicMock", module._coordinator)


def test_request_operator_holds_on_the_agent_route(handoff: OperatorHandoff) -> None:
    _coordinator(handoff).task_invoke.return_value = _hold(True, "agent", "missed the can")

    result = handoff.request_operator("missed the can")

    _coordinator(handoff).task_invoke.assert_called_once_with(
        OPERATOR_HOLD_TASK_NAME, "request", {"route": "agent", "reason": "missed the can"}
    )
    assert "frozen" in result.message and "missed the can" in result.message
    assert result.metadata["route"] == "agent"
    (event,) = handoff.get_operator_events()
    assert (event["action"], event["route"], event["reason"]) == (
        "request",
        "agent",
        "missed the can",
    )


def test_request_operator_says_when_there_is_no_hold(handoff: OperatorHandoff) -> None:
    _coordinator(handoff).task_invoke.side_effect = KeyError("operator_hold")

    result = handoff.request_operator("stuck")

    assert result.message.startswith("Could not request an operator")
    assert handoff.get_operator_events() == []


def _read(module: OperatorHandoff, hold: OperatorHoldStatus, estopped: bool = False) -> None:
    """One read of the coordinator's status, as a stream message or the resync timer starts."""
    _coordinator(module).get_operator_status.return_value = {
        "hold": {**asdict(hold), "unheld": list(hold.unheld)},
        "estopped": estopped,
    }
    module._resync()


def test_hold_and_estop_changes_are_logged_once(handoff: OperatorHandoff) -> None:
    _read(handoff, _hold(True))
    _read(handoff, _hold(True))  # the hold repeats its status once a second
    _read(handoff, _hold(True), estopped=True)
    _read(handoff, _hold(True), estopped=True)
    _read(handoff, _hold(True), estopped=False)
    _read(handoff, _hold(False))

    assert [action for action, _ in _actions(handoff)] == [
        "request",
        "estop",
        "estop_clear",
        "acknowledge",
    ]
    assert all(event["reason"] == "arm stuck" for event in handoff.get_operator_events())


def test_an_estop_event_carries_the_hold_it_caused(handoff: OperatorHandoff) -> None:
    _read(handoff, _hold(True, "failsafe", "e-stop"), estopped=True)

    (request, estop) = handoff.get_operator_events()
    assert (request["action"], request["route"]) == ("request", "failsafe")
    assert (estop["action"], estop["reason"]) == ("estop", "e-stop")


def test_a_stream_message_reads_the_status_at_once(handoff: OperatorHandoff) -> None:
    _coordinator(handoff).get_operator_status.return_value = {"hold": None, "estopped": False}
    handoff.start()
    _wait_for(lambda: _coordinator(handoff).get_operator_status.called)
    _coordinator(handoff).get_operator_status.reset_mock()

    handoff._on_status(Bool(data=True))

    _wait_for(lambda: _coordinator(handoff).get_operator_status.called)
    handoff.stop()


def test_takeover_marks_events_until_autonomy_restarts(handoff: OperatorHandoff) -> None:
    _read(handoff, _hold(True))
    _read(handoff, _hold(False))
    handoff.record_operator_event("takeover")
    handoff.record_operator_event("target", object_id="can-2")

    answer = handoff.restart_autonomy("moved the can by hand")
    handoff.record_operator_event("pick")

    assert answer["ok"] and answer["status"]["mode"] == "autonomous"
    assert _actions(handoff) == [
        ("request", "assist"),
        ("acknowledge", "assist"),
        ("takeover", "takeover"),
        ("target", "takeover"),
        ("restart", "takeover"),
        ("pick", "assist"),
    ]
    assert handoff.get_operator_events()[-1]["object_id"] == "can-2"


def test_restart_is_refused_while_held_or_estopped(handoff: OperatorHandoff) -> None:
    _read(handoff, _hold(True))
    held = handoff.restart_autonomy()
    _read(handoff, _hold(False), estopped=True)
    latched = handoff.restart_autonomy()

    assert not held["ok"] and "acknowledge" in held["error"]
    assert not latched["ok"] and "e-stop" in latched["error"]
    assert "restart" not in [action for action, _ in _actions(handoff)]


def test_restart_tells_the_agent_to_carry_on(handoff: OperatorHandoff) -> None:
    told: list[str] = []
    handoff.human_input.subscribe(told.append)
    handoff.record_operator_event("pick", object_id="ball-1")

    handoff.restart_autonomy("re-picked the ball")

    assert len(told) == 1 and "ball-1" in told[0] and "re-picked the ball" in told[0]


def test_unknown_page_actions_are_refused(handoff: OperatorHandoff) -> None:
    with pytest.raises(ValueError, match="acknowledge"):
        handoff.record_operator_event("acknowledge")


def test_events_go_to_the_run_log(handoff: OperatorHandoff, tmp_path: Path) -> None:
    _read(handoff, _hold(True, "agent", "gone"))
    handoff.record_operator_event("target", object_id="box-3")

    lines = (tmp_path / OPERATOR_EVENTS_FILE).read_text().splitlines()

    assert [json.loads(line)["action"] for line in lines] == ["request", "target"]
    assert set(json.loads(lines[0])) == {
        "timestamp",
        "kind",
        "action",
        "object_id",
        "reason",
        "route",
        "detail",
    }


def test_status_goes_out_as_json(handoff: OperatorHandoff) -> None:
    sent: list[String] = []
    handoff.operator_status.subscribe(sent.append)
    _coordinator(handoff).get_operator_status.return_value = {
        "hold": {"on": True, "route": "failsafe", "reason": "e-stop"},
        "estopped": True,
    }

    handoff.start()
    _wait_for(lambda: bool(sent))
    handoff.stop()

    status = json.loads(sent[0].data)
    assert status["hold"]["on"] and status["estopped"] and status["mode"] == "assist"
    assert [event["action"] for event in status["events"]] == ["request", "estop"]
