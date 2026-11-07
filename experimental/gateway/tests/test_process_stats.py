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

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from fastapi.testclient import TestClient

from dimos.core import run_registry
from experimental.gateway.process_stats import ProcessStats
from experimental.gateway.state import State


class FakeTransport:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.handlers: list[Callable[[dict[str, Any]], None]] = []
        self.started = False
        self.stopped = False

    def subscribe(self, handler: Callable[[dict[str, Any]], None]) -> None:
        self.handlers.append(handler)

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def send(self, message: dict[str, Any]) -> None:
        for handler in self.handlers:
            handler(message)


def stats_message(cpu: float) -> dict[str, Any]:
    return {
        "coordinator": {"pid": 10, "cpu_percent": cpu, "pss": 1000, "num_threads": 4},
        "workers": [
            {
                "worker_id": 0,
                "pid": 11,
                "alive": True,
                "modules": ["nav", "lidar"],
                "cpu_percent": cpu * 2,
                "children": [{"pid": 12, "name": "ffmpeg", "cpu_percent": 3.0}],
            }
        ],
    }


def test_it_subscribes_on_the_first_ask_and_keeps_cpu_history() -> None:
    made: list[FakeTransport] = []

    def factory(topic: str) -> FakeTransport:
        made.append(FakeTransport(topic))
        return made[-1]

    stats = ProcessStats(factory)
    assert made == []
    assert stats.snapshot()["up"] is False
    assert [publisher.topic for publisher in made] == ["/resource_stats"] and made[0].started
    for cpu in (1.0, 2.0, 3.0):
        made[0].send(stats_message(cpu))
    shown = stats.snapshot()
    assert shown["up"] is True and shown["stale"] is False
    assert shown["coordinator"]["pid"] == 10
    assert shown["workers"][0]["modules"] == ["nav", "lidar"]
    assert shown["cpuHistory"] == {
        "coordinator": [1.0, 2.0, 3.0],
        "worker:0": [2.0, 4.0, 6.0],
        "child:12": [3.0, 3.0, 3.0],
    }
    assert len(made) == 1, "one subscriber, however often it's asked"


def test_the_subscriber_drops_when_nothing_runs(monkeypatch: Any) -> None:
    made: list[FakeTransport] = []
    stats = ProcessStats(lambda topic: made.append(FakeTransport(topic)) or made[-1])
    stats.snapshot()
    made[0].send(stats_message(5.0))
    monkeypatch.setattr(run_registry, "get_most_recent", lambda alive_only: object())
    stats.stop_if_nothing_runs()
    assert not made[0].stopped, "a run is alive"
    monkeypatch.setattr(run_registry, "get_most_recent", lambda alive_only: None)
    stats.stop_if_nothing_runs()
    assert made[0].stopped
    assert stats.snapshot()["up"] is False and len(made) == 2


def test_the_route_answers_the_snapshot(client: TestClient, state: State) -> None:
    made: list[FakeTransport] = []
    state.process_stats = ProcessStats(lambda topic: made.append(FakeTransport(topic)) or made[-1])
    answer = client.get("/active/stats")
    assert answer.status_code == 200, answer.text
    assert answer.json()["up"] is False
    made[0].send(stats_message(7.0))
    shown = client.get("/active/stats").json()
    assert shown["up"] is True
    assert shown["cpuHistory"]["worker:0"] == [14.0]
