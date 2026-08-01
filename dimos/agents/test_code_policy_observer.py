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
from collections.abc import Iterator
from datetime import UTC, datetime
import io
import json
from pathlib import Path
import stat
import threading
import time

import nbformat
import pytest

from dimos.agents.code_policy import (
    CodePolicyExecutionRecord,
    CodePolicyModule,
    CodePolicyObserverDescriptor,
    CodePolicyObserverProbeReceipt,
    CodePolicyObserverState,
)
from dimos.agents.code_policy_observer import (
    IOPubSubscriber,
    ObservationStore,
    ObservedIOPubMessage,
    materialize_notebook,
    select_code_policy_module,
    watch_code_policy,
)


def _descriptor() -> CodePolicyObserverDescriptor:
    return CodePolicyObserverDescriptor(
        transport="tcp",
        ip="127.0.0.1",
        iopub_port=1234,
        signature_scheme="hmac-sha256",
        key_base64=base64.b64encode(b"secret").decode("ascii"),
        code_policy_session_id="code_policy_session_" + "a" * 32,
        jupyter_client_session_id="policy-client",
        kernel_generation=1,
    )


@pytest.fixture
def code_policy(mocker, tmp_path) -> Iterator[CodePolicyModule]:
    mocker.patch("dimos.agents.code_policy._bootstrap_source", return_value="pass")
    module = CodePolicyModule(recording_path=str(tmp_path / "unused.db"))
    module.start()
    try:
        yield module
    finally:
        module.stop()


def test_subscriber_constructs_only_a_read_only_sub_socket(mocker) -> None:
    context = mocker.Mock()
    socket = mocker.Mock()
    context.socket.return_value = socket

    subscriber = IOPubSubscriber(_descriptor(), context=context)
    subscriber.close()

    import zmq

    context.socket.assert_called_once_with(zmq.SUB)
    socket.setsockopt.assert_any_call(zmq.SUBSCRIBE, b"")
    socket.connect.assert_called_once_with("tcp://127.0.0.1:1234")
    socket.close.assert_called_once_with(linger=0)
    forbidden = {
        "execute",
        "interrupt",
        "restart",
        "shutdown",
        "shell_channel",
        "control_channel",
        "stdin_channel",
        "hb_channel",
    }
    assert forbidden.isdisjoint(dir(subscriber))


def test_first_agent_cell_is_captured_after_verified_probe(
    code_policy: CodePolicyModule,
) -> None:
    state = code_policy.prepare_observer()
    assert state.descriptor is not None
    subscriber = IOPubSubscriber(state.descriptor)
    try:
        probe = code_policy.issue_observer_probe(state.kernel_generation)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            message = subscriber.receive(timeout_s=0.1)
            if (
                message is not None
                and message.parent_message_id == probe.message_id
                and message.message_type == "status"
                and message.content["execution_state"] == "idle"
            ):
                break
        else:
            pytest.fail("subscriber did not observe fixed readiness probe")

        thread = threading.Thread(
            target=code_policy.python_exec,
            args=("print('live output')\n6 * 7",),
        )
        thread.start()
        messages = []
        capture_deadline = time.monotonic() + 5
        while time.monotonic() < capture_deadline:
            message = subscriber.receive(timeout_s=0.1)
            if message is not None:
                messages.append(message)
            if not thread.is_alive() and any(
                value.message_type == "status" and value.content.get("execution_state") == "idle"
                for value in messages
            ):
                break
        thread.join(timeout=2)
    finally:
        subscriber.close()

    assert any(
        message.message_type == "execute_input"
        and message.content["code"] == "print('live output')\n6 * 7"
        for message in messages
    )
    assert any(
        message.message_type == "stream" and message.content["text"] == "live output\n"
        for message in messages
    )
    assert {message.parent_session_id for message in messages} == {
        state.descriptor.jupyter_client_session_id
    }


def test_store_reserves_private_non_overwriting_recordings(tmp_path: Path) -> None:
    first = ObservationStore(tmp_path)
    second = ObservationStore(tmp_path)

    assert first.path != second.path
    assert stat.S_IMODE(first.path.stat().st_mode) == 0o700
    assert stat.S_IMODE(first.events_path.stat().st_mode) == 0o600


def test_store_records_bounded_events_and_valid_notebook(tmp_path: Path) -> None:
    store = ObservationStore(tmp_path, event_limit=80)
    message = ObservedIOPubMessage(
        observed_at=datetime.now(UTC),
        message_type="execute_input",
        parent_message_id="cell-1",
        parent_session_id="policy-client",
        execution_count=1,
        content={"code": "x" * 1000},
    )

    event = store.append_iopub(_descriptor(), message)
    notebook_path = store.finalize(complete=False)
    notebook = nbformat.read(notebook_path, as_version=4)

    assert event.truncated
    assert event.artifact_refs
    assert (store.path / event.artifact_refs[0]).stat().st_size == 80
    assert notebook.cells[-1].cell_type == "markdown"
    assert "Incomplete observation" in notebook.cells[-1].source


def test_store_enforces_execution_mime_and_recording_limits(tmp_path: Path) -> None:
    store = ObservationStore(
        tmp_path,
        event_limit=10_000,
        execution_limit=100,
        mime_limit=20,
        recording_limit=10_000,
    )
    rich = ObservedIOPubMessage(
        observed_at=datetime.now(UTC),
        message_type="display_data",
        parent_message_id="cell-1",
        parent_session_id="policy-client",
        execution_count=None,
        content={"data": {"image/png": "a" * 100}, "metadata": {}},
    )
    stream = rich.model_copy(
        update={
            "message_type": "stream",
            "content": {"name": "stdout", "text": "b" * 100},
        }
    )

    rich_event = store.append_iopub(_descriptor(), rich)
    stream_event = store.append_iopub(_descriptor(), stream)

    assert rich_event.truncated
    assert rich_event.artifact_refs
    assert stream_event.truncated
    assert "execution exceeded" in stream_event.content["truncation"]

    tiny = ObservationStore(tmp_path, recording_limit=10)
    with pytest.raises(RuntimeError, match="recording limit"):
        tiny.append_lifecycle(_descriptor(), "too-large")


def test_notebook_recovery_ignores_malformed_jsonl_tail(tmp_path: Path) -> None:
    store = ObservationStore(tmp_path)
    store.append_lifecycle(_descriptor(), "ready")
    with store.events_path.open("a") as stream:
        stream.write('{"partial":')

    recovered = store.path / "recovered.ipynb"
    materialize_notebook(store.events_path, recovered, complete=False)

    notebook = nbformat.read(recovered, as_version=4)
    assert "ready" in notebook.cells[0].source
    assert "Incomplete observation" in notebook.cells[-1].source


def test_notebook_recovery_marks_missing_blob(tmp_path: Path) -> None:
    store = ObservationStore(tmp_path, event_limit=20)
    event = store.append_iopub(
        _descriptor(),
        ObservedIOPubMessage(
            observed_at=datetime.now(UTC),
            message_type="execute_input",
            parent_message_id="cell-1",
            parent_session_id="policy-client",
            execution_count=1,
            content={"code": "x" * 100},
        ),
    )
    (store.path / event.artifact_refs[0]).unlink()

    store.finalize()
    notebook = nbformat.read(store.notebook_path, as_version=4)

    assert any(
        cell.cell_type == "markdown" and "Missing artifacts" in cell.source
        for cell in notebook.cells
    )


@pytest.mark.parametrize("wait", [False, True])
def test_notebook_clear_output_matches_jupyter_semantics(tmp_path: Path, wait: bool) -> None:
    store = ObservationStore(tmp_path)

    def append(message_type: str, content: dict[str, object]) -> None:
        store.append_iopub(
            _descriptor(),
            ObservedIOPubMessage(
                observed_at=datetime.now(UTC),
                message_type=message_type,  # type: ignore[arg-type]
                parent_message_id="cell-1",
                parent_session_id="policy-client",
                execution_count=1 if message_type == "execute_input" else None,
                content=content,
            ),
        )

    append("execute_input", {"code": "display('before'); clear_output(); print('after')"})
    append("stream", {"name": "stdout", "text": "before\n"})
    append("clear_output", {"wait": wait})
    append("stream", {"name": "stdout", "text": "after\n"})
    store.finalize()

    notebook = nbformat.read(store.notebook_path, as_version=4)
    outputs = notebook.cells[0].outputs
    assert [output.text for output in outputs] == ["after\n"]


@pytest.mark.parametrize(
    ("candidates", "requested", "expected"),
    [
        (["CodePolicyModule"], None, "CodePolicyModule"),
        (["Other", "CodePolicyModule"], None, "CodePolicyModule"),
        (["a.CodePolicyModule", "b.CodePolicyModule"], "b.CodePolicyModule", "b.CodePolicyModule"),
    ],
)
def test_select_code_policy_module(
    candidates: list[str], requested: str | None, expected: str
) -> None:
    assert select_code_policy_module(candidates, requested) == expected


@pytest.mark.parametrize(
    ("candidates", "requested"),
    [
        ([], None),
        (["a.CodePolicyModule", "b.CodePolicyModule"], None),
        (["CodePolicyModule"], "missing.CodePolicyModule"),
    ],
)
def test_select_code_policy_module_fails_closed(
    candidates: list[str], requested: str | None
) -> None:
    with pytest.raises(LookupError, match="candidates"):
        select_code_policy_module(candidates, requested)


def test_fake_host_watch_reconnects_reconciles_and_finalizes(mocker, tmp_path: Path) -> None:
    descriptors = [
        _descriptor(),
        _descriptor().model_copy(
            update={
                "kernel_generation": 2,
                "code_policy_session_id": "code_policy_session_" + "c" * 32,
            }
        ),
    ]

    class FakeHost:
        probe_id: str | None = None
        state_calls = 0
        prepare_calls = 0

        def prepare_observer(self) -> CodePolicyObserverState:
            self.prepare_calls += 1
            descriptor = descriptors[min(self.prepare_calls - 1, 1)]
            return CodePolicyObserverState(
                availability="ready",
                code_policy_session_id=descriptor.code_policy_session_id,
                kernel_generation=descriptor.kernel_generation,
                descriptor=descriptor,
            )

        def issue_observer_probe(self, kernel_generation: int) -> CodePolicyObserverProbeReceipt:
            self.probe_id = f"probe-{kernel_generation}"
            return CodePolicyObserverProbeReceipt(
                message_id=self.probe_id,
                code_policy_session_id=descriptors[0].code_policy_session_id,
                kernel_generation=kernel_generation,
            )

        def get_observer_state(
            self, known_generation: int | None = None
        ) -> CodePolicyObserverState:
            self.state_calls += 1
            if self.state_calls == 1:
                return CodePolicyObserverState(
                    availability="unavailable",
                    code_policy_session_id=descriptors[1].code_policy_session_id,
                    kernel_generation=1,
                    descriptor=None,
                )
            return CodePolicyObserverState(
                availability="stopped",
                code_policy_session_id=descriptors[1].code_policy_session_id,
                kernel_generation=2,
                descriptor=None,
            )

        def get_execution_records(
            self, session_id: str | None = None
        ) -> tuple[CodePolicyExecutionRecord, ...]:
            now = datetime.now(UTC)
            return (
                CodePolicyExecutionRecord(
                    execution_id="code_policy_call_" + "b" * 32,
                    session_id=descriptors[0].code_policy_session_id,
                    source="print('hello')",
                    requested_timeout_s=10.0,
                    started_at=now,
                    finished_at=now,
                    monotonic_duration_s=0.1,
                    status="completed",
                    jupyter_message_id="cell-1",
                    jupyter_execution_count=1,
                    output="hello\n",
                    transcript="completed",
                    interrupt_attempted=False,
                    interrupt_recovered=False,
                    kernel_restarted=False,
                    namespace_preserved=True,
                    remote_work_may_continue=False,
                ),
            )

    host = FakeHost()

    class FakeSubscriber:
        def __init__(self, descriptor: CodePolicyObserverDescriptor) -> None:
            self.descriptor = descriptor
            self.sent_probe = False
            self.messages = (
                [
                    ObservedIOPubMessage(
                        observed_at=datetime.now(UTC),
                        message_type="execute_input",
                        parent_message_id="cell-1",
                        parent_session_id=descriptor.jupyter_client_session_id,
                        execution_count=1,
                        content={"code": "print('hello')"},
                    ),
                    ObservedIOPubMessage(
                        observed_at=datetime.now(UTC),
                        message_type="stream",
                        parent_message_id="cell-1",
                        parent_session_id=descriptor.jupyter_client_session_id,
                        execution_count=None,
                        content={"name": "stdout", "text": "hello\n"},
                    ),
                    ObservedIOPubMessage(
                        observed_at=datetime.now(UTC),
                        message_type="status",
                        parent_message_id="cell-1",
                        parent_session_id=descriptor.jupyter_client_session_id,
                        execution_count=None,
                        content={"execution_state": "idle"},
                    ),
                ]
                if descriptor.kernel_generation == 1
                else []
            )

        def receive(self, timeout_s: float = 0.1) -> ObservedIOPubMessage | None:
            del timeout_s
            if host.probe_id is not None and not self.sent_probe:
                self.sent_probe = True
                return ObservedIOPubMessage(
                    observed_at=datetime.now(UTC),
                    message_type="status",
                    parent_message_id=host.probe_id,
                    parent_session_id=self.descriptor.jupyter_client_session_id,
                    execution_count=None,
                    content={"execution_state": "idle"},
                )
            if self.messages:
                return self.messages.pop(0)
            return None

        def close(self) -> None:
            return None

    mocker.patch(
        "dimos.agents.code_policy_observer.IOPubSubscriber",
        side_effect=FakeSubscriber,
    )
    output = io.StringIO()

    path = watch_code_policy(
        host,
        output_parent=tmp_path,
        startup_timeout_s=1,
        poll_interval_s=0.001,
        stream=output,
    )

    events = [json.loads(line) for line in (path / "iopub-events.jsonl").read_text().splitlines()]
    notebook = nbformat.read(path / "code-policy-transcript.ipynb", as_version=4)
    assert "ready" in output.getvalue()
    assert "hello" in output.getvalue()
    assert "observation-gap" in output.getvalue()
    assert "reconnected" in output.getvalue()
    assert any(event["kind"] == "record" for event in events)
    assert any(event["content"].get("event") == "observation-gap" for event in events)
    assert any(cell.cell_type == "code" for cell in notebook.cells)
