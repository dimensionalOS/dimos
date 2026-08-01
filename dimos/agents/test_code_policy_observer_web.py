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

import base64
from datetime import UTC, datetime
import io
from pathlib import Path
import threading

from fastapi.testclient import TestClient

from dimos.agents.code_policy import CodePolicyObserverDescriptor
from dimos.agents.code_policy_observer import ObservationEvent
from dimos.agents.code_policy_observer_web import (
    CodePolicyWebServer,
    ObservationEventHub,
    WebObserverView,
    WebSessionState,
    create_web_app,
)


def _descriptor() -> CodePolicyObserverDescriptor:
    return CodePolicyObserverDescriptor(
        transport="tcp",
        ip="127.0.0.1",
        iopub_port=54321,
        signature_scheme="hmac-sha256",
        key_base64=base64.b64encode(b"browser-must-not-see-this").decode(),
        code_policy_session_id="code_policy_session_" + "a" * 32,
        jupyter_client_session_id="policy-client",
        kernel_generation=3,
    )


def _event(sequence: int, *, text: str = "hello\n") -> ObservationEvent:
    descriptor = _descriptor()
    return ObservationEvent(
        sequence=sequence,
        observed_at=datetime.now(UTC),
        kind="iopub",
        code_policy_session_id=descriptor.code_policy_session_id,
        kernel_generation=descriptor.kernel_generation,
        parent_message_id="cell-1",
        message_type="stream",
        content={"name": "stdout", "text": text},
    )


def test_event_hub_replays_and_resumes_without_waiting() -> None:
    hub = ObservationEventHub()
    hub.publish(_event(1, text="one"))
    hub.publish(_event(2, text="two"))

    all_events, closed = hub.wait_after(0, 0)
    resumed, _ = hub.wait_after(1, 0)

    assert [event.sequence for event in all_events] == [1, 2]
    assert [event.sequence for event in resumed] == [2]
    assert not closed


def test_event_hub_wakes_waiter_on_publish() -> None:
    hub = ObservationEventHub()
    observed: list[ObservationEvent] = []
    finished = threading.Event()

    def wait_for_event() -> None:
        events, _closed = hub.wait_after(0, 1)
        observed.extend(events)
        finished.set()

    thread = threading.Thread(target=wait_for_event)
    thread.start()
    try:
        hub.publish(_event(1))
        assert finished.wait(1)
        assert [event.sequence for event in observed] == [1]
    finally:
        hub.close()
        thread.join(timeout=1)


def test_web_app_is_read_only_replayable_and_credential_free(tmp_path: Path) -> None:
    hub = ObservationEventHub()
    state = WebSessionState()
    state.set_recording_path(tmp_path)
    state.ready(_descriptor())
    hub.publish(_event(1))
    hub.publish(_event(2, text="world\n"))
    hub.close()

    with TestClient(create_web_app(hub, state)) as client:
        page = client.get("/")
        session = client.get("/api/session")
        replay = client.get("/api/events", headers={"last-event-id": "1"})
        rejected = client.post("/api/session", json={"code": "print('no')"})

    assert page.status_code == 200
    assert "Code Policy Observer" in page.text
    assert "default-src 'self'" in page.headers["content-security-policy"]
    assert session.json() == {
        "status": "ready",
        "recording_path": str(tmp_path.resolve()),
        "code_policy_session_id": _descriptor().code_policy_session_id,
        "jupyter_client_session_id": "policy-client",
        "kernel_generation": 3,
    }
    assert "id: 1" not in replay.text
    assert "id: 2" in replay.text
    assert "observer-close" in replay.text
    assert "browser-must-not-see-this" not in session.text + replay.text + page.text
    assert "54321" not in session.text + replay.text + page.text
    assert rejected.status_code == 405


def test_web_view_opens_browser_only_after_ready(mocker, tmp_path: Path) -> None:
    server = mocker.create_autospec(CodePolicyWebServer, instance=True)
    server.start.return_value = "http://127.0.0.1:8766"
    server.url = "http://127.0.0.1:8766"
    opener = mocker.Mock()
    output = io.StringIO()
    mocker.patch(
        "dimos.agents.code_policy_observer_web.CodePolicyWebServer",
        return_value=server,
    )
    view = WebObserverView(port=8766, browser_opener=opener, stream=output)

    view.start(tmp_path)
    opener.assert_not_called()
    view.connected(_descriptor(), initial=True)
    view.close()

    opener.assert_called_once_with("http://127.0.0.1:8766")
    server.connected.assert_called_once_with(_descriptor())
    server.stop.assert_called_once_with()
    assert "web: http://127.0.0.1:8766" in output.getvalue()
    assert "ready" in output.getvalue()
