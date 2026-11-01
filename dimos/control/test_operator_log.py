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

"""OperatorLog: what each input logs, the file, the topic and the RPCs.

The inputs get in-process transports, so publishing on one calls the module's
handler at once. Time comes from a counter so every event has its own ts.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import asdict
import itertools
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from dimos.control import operator_log
from dimos.control.operator_log import OPERATOR_EVENTS_FILE, OperatorLog
from dimos.control.tasks.operator_hold_task.operator_hold_task import OperatorHoldStatus
from dimos.imitation.collection.episode import EpisodeStatus, RecordingState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.String import String


class LocalTransport:
    """An in-process transport: publishing calls every subscriber at once."""

    def __init__(self) -> None:
        self.subscribers: list[Callable[[Any], Any]] = []

    def broadcast(self, selfstream: Any, value: Any) -> None:
        for cb in list(self.subscribers):
            cb(value)

    def publish(self, value: Any) -> None:
        self.broadcast(None, value)

    def subscribe(self, cb: Callable[[Any], Any], selfstream: Any = None) -> Callable[[], None]:
        self.subscribers.append(cb)
        return lambda: self.subscribers.remove(cb)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


class Rig:
    def __init__(self, **config: Any) -> None:
        self.log = OperatorLog(**config)
        self.inputs = {
            name: LocalTransport() for name in ("operator_hold", "operator_estop_status", "status")
        }
        for name, transport in self.inputs.items():
            self.log.set_transport(name, transport)  # type: ignore[arg-type]
        self.sent: list[dict[str, Any]] = []
        self.log.operator_events.subscribe(lambda msg: self.sent.append(json.loads(msg.data)))
        self.log.start()

    def hold(self, on: bool, route: str = "", reason: str = "") -> None:
        status = OperatorHoldStatus(on=on, route=route, reason=reason, unheld=("arm/j1",))
        self.inputs["operator_hold"].publish(String(json.dumps(asdict(status))))

    def estop(self, on: bool) -> None:
        self.inputs["operator_estop_status"].publish(Bool(data=on))

    def episode(self, state: RecordingState, saved: int, discarded: int, event: str) -> None:
        status = EpisodeStatus(
            ts=1.0,
            state=state,
            episodes_saved=saved,
            episodes_discarded=discarded,
            last_event=event,  # type: ignore[arg-type]
            task_label="stack the cups",
        )
        self.inputs["status"].publish(String(status.to_json()))

    def actions(self) -> list[tuple[str, str]]:
        return [(event["kind"], event["action"]) for event in self.log.get_events()]


@pytest.fixture
def run_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """The run's log folder, not created yet; time ticks 1 s per call."""
    folder = tmp_path / "run"
    monkeypatch.setattr(operator_log, "get_run_log_dir", lambda: folder)
    monkeypatch.delenv("DIMOS_RUN_LOG_DIR", raising=False)
    clock = itertools.count(1000.0)
    monkeypatch.setattr(operator_log, "time", SimpleNamespace(time=lambda: float(next(clock))))
    return folder


@pytest.fixture
def make_rig(run_dir: Path) -> Iterator[Callable[..., Rig]]:
    rigs: list[Rig] = []

    def _make(**config: Any) -> Rig:
        rigs.append(Rig(**config))
        return rigs[-1]

    yield _make
    for rig in rigs:
        rig.log.stop()


@pytest.fixture
def rig(make_rig: Callable[..., Rig]) -> Rig:
    return make_rig()


def test_hold_logs_one_request_and_one_acknowledge_per_change(rig: Rig) -> None:
    for _ in range(3):  # the hold resends its status every second while on
        rig.hold(True, route="agent", reason="box slipped")
    rig.hold(False)
    rig.hold(False)

    events = rig.log.get_events()
    assert rig.actions() == [("hold", "request"), ("hold", "acknowledge")]
    for event in events:  # the acknowledge names the hold it ended
        assert (event["route"], event["reason"]) == ("agent", "box slipped")


def test_estop_logs_each_change_only(rig: Rig) -> None:
    rig.estop(False)  # the coordinator sends the current state at start
    rig.estop(True)
    rig.estop(True)
    rig.estop(False)

    assert rig.actions() == [("safety", "estop"), ("safety", "estop_clear")]


def test_episodes_log_start_save_and_discard(rig: Rig) -> None:
    rig.episode("idle", 0, 0, "init")
    rig.episode("recording", 0, 0, "start")
    rig.episode("idle", 1, 0, "save")
    rig.episode("idle", 1, 0, "save")  # a save while idle changes nothing
    rig.episode("recording", 1, 0, "start")
    rig.episode("idle", 1, 1, "discard")
    rig.episode("recording", 1, 1, "start")
    rig.episode("recording", 2, 1, "start")  # a start while recording saves the take

    assert rig.actions() == [
        ("data", "episode_start"),
        ("data", "episode_saved"),
        ("data", "episode_start"),
        ("data", "episode_discarded"),
        ("data", "episode_start"),
        ("data", "episode_saved"),
        ("data", "episode_start"),
    ]
    assert {event["detail"] for event in rig.log.get_events()} == {"stack the cups"}


def test_invalid_messages_are_ignored(rig: Rig) -> None:
    rig.inputs["operator_hold"].publish(String("not json"))
    rig.inputs["status"].publish(String('{"state": "recording"}'))

    assert rig.log.get_events() == []


def test_record_event_kinds(rig: Rig) -> None:
    takeover = rig.log.record_event("takeover")
    pick = rig.log.record_event("pick", detail="grasped", object_id="cup_1")

    assert takeover["kind"] == "takeover"
    assert pick == {
        "ts": pick["ts"],
        "kind": "assist",
        "action": "pick",
        "route": "",
        "reason": "",
        "object_id": "cup_1",
        "detail": "grasped",
    }
    with pytest.raises(ValueError):
        rig.log.record_event("")


def test_get_events_since(rig: Rig) -> None:
    first = rig.log.record_event("target", object_id="cup_1")
    rig.log.record_event("pick", object_id="cup_1")

    assert [event["action"] for event in rig.log.get_events(since=first["ts"])] == ["pick"]
    assert len(rig.log.get_events()) == 2


def test_each_event_is_one_line_in_the_run_folder(rig: Rig, run_dir: Path) -> None:
    rig.hold(True, route="manual", reason="check the gripper")
    rig.estop(True)
    rig.log.record_event("takeover")

    path = run_dir / OPERATOR_EVENTS_FILE
    lines = path.read_text(encoding="utf-8").splitlines()
    assert [json.loads(line) for line in lines] == rig.log.get_events()
    status = rig.log.get_status()
    assert status["path"] == str(path)
    assert status["count"] == 3
    assert status["last"]["action"] == "takeover"


def test_topic_carries_the_latest_events_and_the_total(make_rig: Callable[..., Rig]) -> None:
    rig = make_rig(status_events=2)
    for action in ("target", "pick", "takeover"):
        rig.log.record_event(action)

    assert [payload["count"] for payload in rig.sent] == [1, 2, 3]
    last = rig.sent[-1]
    assert [event["action"] for event in last["events"]] == ["pick", "takeover"]
    assert last["events"] == rig.log.get_events()[-2:]


def test_worker_process_finds_the_run_folder_in_the_environment(
    rig: Rig, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(operator_log, "get_run_log_dir", lambda: None)
    monkeypatch.setenv("DIMOS_RUN_LOG_DIR", str(tmp_path / "worker"))
    rig.log.record_event("target")

    assert (tmp_path / "worker" / OPERATOR_EVENTS_FILE).exists()


def test_outside_a_run_nothing_is_written(rig: Rig, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(operator_log, "get_run_log_dir", lambda: None)
    rig.log.record_event("target")

    assert rig.log.get_status()["path"] == ""
    assert rig.log.get_status()["count"] == 1


def test_unconnected_inputs_are_skipped(run_dir: Path) -> None:
    log = OperatorLog()
    try:
        log.start()
        assert log.record_event("takeover")["kind"] == "takeover"
    finally:
        log.stop()
