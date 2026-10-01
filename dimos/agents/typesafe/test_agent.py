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
"""The generic agent: a trigger stream, a state, questions, and typed answers out."""

from collections.abc import Iterator
import json
from pathlib import Path
import threading
import time

import pytest
from reactivex import Observable
from reactivex.scheduler import ThreadPoolScheduler

from dimos.agents.typesafe.agent import TypeSafeAgent
from dimos.agents.typesafe.types import Answers, Question, noul
from dimos.core.stream import In
from dimos.core.transport import pLCMTransport


class EchoAgent(TypeSafeAgent):
    """Asks one Noul about each text that arrives."""

    text: In[str]

    def trigger(self) -> Observable[object]:
        return self.text.observable()

    def state(self, trigger: object) -> object | None:
        return {"text": trigger}

    def questions(self, state: object) -> dict[str, Question]:
        return {"urgent": noul("Is `text` urgent?")}

    def on_answers(self, state: object, answers: Answers) -> None:
        self.seen.append((state, answers))


@pytest.fixture
def agent(
    monkeypatch: pytest.MonkeyPatch,
) -> Iterator[tuple[EchoAgent, list[Answers], threading.Event]]:
    scheduler = ThreadPoolScheduler(max_workers=2)
    monkeypatch.setattr("dimos.utils.reactive.get_scheduler", lambda: scheduler)
    a = EchoAgent(max_hz=None)
    for name in ("text", "choices", "scores", "nouls"):
        getattr(a, name).transport = pLCMTransport(f"/test_typesafe_agent/{name}")
    a.seen = []  # type: ignore[attr-defined]
    calls = threading.Event()

    def fake_post(state: object, qs: dict[str, Question]) -> Answers:
        calls.set()
        return {"urgent": {"type": "noul", "noul": 0.9 if "ASAP" in str(state) else 0.1}}

    a._post = fake_post  # type: ignore[method-assign]
    published: list[Answers] = []
    unsub = a.nouls.transport.subscribe(published.append)
    a.start()
    yield a, published, calls
    unsub()
    a.stop()
    scheduler.executor.shutdown(wait=True)


def test_trigger_drives_inference_and_answers_publish_by_type(
    agent: tuple[EchoAgent, list[Answers], threading.Event],
) -> None:
    a, published, calls = agent
    a.text.transport.publish("fix this ASAP")
    assert calls.wait(3)
    deadline = time.monotonic() + 3
    while not published and time.monotonic() < deadline:
        time.sleep(0.02)
    assert published[0]["urgent"]["noul"] == 0.9  # type: ignore[typeddict-item]
    assert a.seen[0][0] == {"text": "fix this ASAP"}  # type: ignore[attr-defined]


def test_set_trace_dir_records_each_call(
    agent: tuple[EchoAgent, list[Answers], threading.Event], tmp_path: Path
) -> None:
    a, _published, calls = agent
    a.set_trace_dir(str(tmp_path))
    a.text.transport.publish("hello")
    assert calls.wait(3)
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        try:
            request = json.loads((tmp_path / "1-request.json").read_text())
            response = json.loads((tmp_path / "1-response.json").read_text())
            break
        except (OSError, ValueError):  # not written yet, or written half-way
            time.sleep(0.02)
    assert request["body"]["state"] == {"text": "hello"} and "started_at" in request
    assert response["body"]["answers"]["urgent"]["noul"] == 0.1 and "usage" in response["body"]
    assert "latency_s" in response
