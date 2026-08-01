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

import asyncio
import base64
from collections.abc import Iterator

import pytest

from dimos.agents.code_policy import (
    CodePolicyModule,
    CodePolicyObserverDescriptor,
    CodePolicyObserverState,
    _BoundedTextOutput,
)
from dimos.agents.mcp.mcp_server import handle_request
from dimos.benchmark.agent_eval.pi_adapter import inspect_python_exec_inventory


@pytest.fixture
def code_policy(mocker, tmp_path) -> Iterator[CodePolicyModule]:
    mocker.patch("dimos.agents.code_policy._bootstrap_source", return_value="pass")
    module = CodePolicyModule(
        recording_path=str(tmp_path / "unused.db"),
        interrupt_grace_s=1.0,
    )
    module.start()
    try:
        yield module
    finally:
        module.stop()


def test_output_collector_keeps_only_bounded_plain_text() -> None:
    output = _BoundedTextOutput(limit=20)

    output(
        {
            "header": {"msg_type": "stream"},
            "content": {"name": "stdout", "text": "hello\n"},
        }
    )
    output(
        {
            "header": {"msg_type": "execute_result"},
            "content": {"data": {"text/plain": "42", "image/png": "ignored"}},
        }
    )
    output(
        {
            "header": {"msg_type": "display_data"},
            "content": {"data": {"text/html": "<b>ignored</b>"}},
        }
    )

    assert output.text() == "hello\n42"


def test_output_collector_truncates_combined_output() -> None:
    output = _BoundedTextOutput(limit=30)

    output(
        {
            "header": {"msg_type": "stream"},
            "content": {"name": "stdout", "text": "x" * 100},
        }
    )
    output(
        {
            "header": {"msg_type": "stream"},
            "content": {"name": "stdout", "text": "not collected"},
        }
    )

    assert output.text() == "x" * 7 + "\n... [output truncated]"


def test_output_collector_honors_limits_smaller_than_marker() -> None:
    output = _BoundedTextOutput(limit=5)

    output(
        {
            "header": {"msg_type": "stream"},
            "content": {"name": "stdout", "text": "x" * 100},
        }
    )

    assert output.text() == "\n... "
    assert len(output.text()) == 5


def test_code_policy_exposes_only_python_exec_skill(
    code_policy: CodePolicyModule,
) -> None:
    assert [skill.func_name for skill in code_policy.get_skills()] == ["python_exec"]


def test_mcp_lists_python_exec(code_policy: CodePolicyModule) -> None:
    skills = code_policy.get_skills()

    response = asyncio.run(
        handle_request(
            {"method": "tools/list", "id": 1},
            skills,
            {"python_exec": code_policy.python_exec},
        )
    )

    assert response is not None
    assert [tool["name"] for tool in response["result"]["tools"]] == ["python_exec"]


def test_observation_operations_are_rpc_only() -> None:
    for method_name in (
        "prepare_observer",
        "get_observer_state",
        "issue_observer_probe",
    ):
        method = getattr(CodePolicyModule, method_name)
        assert method.__rpc__ is True
        assert not getattr(method, "__skill__", False)


def test_prepare_observer_returns_minimized_descriptor(
    code_policy: CodePolicyModule,
) -> None:
    state = code_policy.prepare_observer()

    assert state.availability == "ready"
    assert state.descriptor is not None
    descriptor = state.descriptor
    assert descriptor.kernel_generation == 1
    assert descriptor.code_policy_session_id == code_policy.get_session_receipt().session_id
    assert descriptor.jupyter_client_session_id
    assert base64.b64decode(descriptor.key_base64)

    serialized = descriptor.model_dump()
    assert set(serialized) == {
        "transport",
        "ip",
        "iopub_port",
        "signature_scheme",
        "key_base64",
        "code_policy_session_id",
        "jupyter_client_session_id",
        "kernel_generation",
    }
    serialized_text = descriptor.model_dump_json()
    for forbidden in (
        "connection_file",
        "shell",
        "control",
        "stdin",
        "heartbeat",
        "hb_port",
    ):
        assert forbidden not in serialized_text


def test_observer_descriptor_and_state_are_strict_and_immutable() -> None:
    descriptor = CodePolicyObserverDescriptor(
        transport="tcp",
        ip="127.0.0.1",
        iopub_port=1234,
        signature_scheme="hmac-sha256",
        key_base64=base64.b64encode(b"secret").decode("ascii"),
        code_policy_session_id="code_policy_session_" + "a" * 32,
        jupyter_client_session_id="jupyter-client",
        kernel_generation=1,
    )
    state = CodePolicyObserverState(
        availability="ready",
        code_policy_session_id=descriptor.code_policy_session_id,
        kernel_generation=1,
        descriptor=descriptor,
    )

    with pytest.raises(ValueError):
        descriptor.kernel_generation = 2
    with pytest.raises(ValueError):
        CodePolicyObserverDescriptor.model_validate({**descriptor.model_dump(), "shell_port": 1235})
    assert state.descriptor is descriptor


def test_observer_state_is_unavailable_before_prepare_and_stopped_after_stop(
    code_policy: CodePolicyModule,
) -> None:
    unavailable = code_policy.get_observer_state()
    assert unavailable.availability == "unavailable"
    assert unavailable.descriptor is None
    assert unavailable.kernel_generation == 0

    code_policy.prepare_observer()
    code_policy.stop()

    stopped = code_policy.get_observer_state()
    assert stopped.availability == "stopped"
    assert stopped.descriptor is None


def test_observer_probe_is_fixed_silent_and_history_free(
    mocker, code_policy: CodePolicyModule
) -> None:
    client = mocker.Mock()
    client.session.session = "policy-client"
    client.execute.return_value = "probe-message"
    manager = mocker.Mock()
    manager.is_alive.return_value = True
    manager.get_connection_info.return_value = {
        "transport": "tcp",
        "ip": "127.0.0.1",
        "iopub_port": 1234,
        "signature_scheme": "hmac-sha256",
        "key": b"secret",
    }
    code_policy._kernel_manager = manager
    code_policy._kernel_client = client
    code_policy._kernel_generation = 7

    receipt = code_policy.issue_observer_probe(7)

    assert receipt.message_id == "probe-message"
    assert receipt.kernel_generation == 7
    client.execute.assert_called_once_with(
        "None",
        silent=True,
        store_history=False,
        allow_stdin=False,
    )
    with pytest.raises(TypeError):
        code_policy.issue_observer_probe(7, "print('not allowed')")  # type: ignore[call-arg]


def test_python_exec_schema_is_compatible_with_pi_facade(
    code_policy: CodePolicyModule,
) -> None:
    response = asyncio.run(
        handle_request(
            {"method": "tools/list", "id": 1},
            code_policy.get_skills(),
            {"python_exec": code_policy.python_exec},
        )
    )

    assert response is not None
    receipt = inspect_python_exec_inventory(
        "http://localhost:9990/mcp",
        response["result"]["tools"],
    )
    assert receipt.python_exec_schema_sha256


def test_python_exec_persists_namespace_and_records_evidence(
    code_policy: CodePolicyModule,
) -> None:
    receipt = code_policy.get_session_receipt()
    first = code_policy.python_exec(
        "import math\n"
        "items = []\n"
        "def remember(value): items.append(value)\n"
        "remember(math.factorial(4))\n"
        "print('hello from kernel')\n"
        "items"
    )
    second = code_policy.python_exec("remember(5)\nitems")
    records = code_policy.get_execution_records(receipt.session_id)

    assert "hello from kernel\n[24]" in first
    assert "[24, 5]" in second
    assert len(records) == 2
    assert {record.session_id for record in records} == {receipt.session_id}
    assert [record.source for record in records] == [
        "import math\n"
        "items = []\n"
        "def remember(value): items.append(value)\n"
        "remember(math.factorial(4))\n"
        "print('hello from kernel')\n"
        "items",
        "remember(5)\nitems",
    ]
    assert all(record.status == "completed" for record in records)
    assert all(record.jupyter_message_id for record in records)
    assert all(record.jupyter_execution_count is not None for record in records)
    assert all(record.monotonic_duration_s >= 0 for record in records)


def test_reset_session_clears_namespace_and_changes_identity(
    code_policy: CodePolicyModule,
) -> None:
    before = code_policy.get_session_receipt()
    code_policy.python_exec("marker = 'old namespace'")

    after = code_policy.reset_session()
    lookup = code_policy.python_exec("'marker' in globals()")

    assert after.previous_session_id == before.session_id
    assert after.session_id != before.session_id
    assert "False" in lookup
    assert len(code_policy.get_execution_records(before.session_id)) == 1
    assert len(code_policy.get_execution_records(after.session_id)) == 1


def test_kernel_generation_advances_across_reset_and_recreation(
    code_policy: CodePolicyModule,
) -> None:
    first = code_policy.prepare_observer()
    code_policy.reset_session()
    unavailable = code_policy.get_observer_state(first.kernel_generation)
    second = code_policy.prepare_observer()

    assert first.kernel_generation == 1
    assert unavailable.availability == "unavailable"
    assert unavailable.kernel_generation == 1
    assert second.kernel_generation == 2
    assert second.code_policy_session_id != first.code_policy_session_id


def test_observer_reports_replaced_generation(
    code_policy: CodePolicyModule,
) -> None:
    first = code_policy.prepare_observer()

    state = code_policy.get_observer_state(first.kernel_generation - 1)

    assert state.availability == "replaced"
    assert state.descriptor is not None
    assert state.descriptor.kernel_generation == first.kernel_generation


def test_interrupt_active_is_false_when_no_cell_is_running(
    code_policy: CodePolicyModule,
) -> None:
    assert code_policy.interrupt_active() is False


def test_python_error_preserves_namespace_and_is_recorded(
    code_policy: CodePolicyModule,
) -> None:
    failed = code_policy.python_exec(
        "items = ['before error']\nraise ValueError('expected failure')"
    )
    recovered = code_policy.python_exec("items")

    assert "failed" in failed
    assert "ValueError" in failed
    assert "expected failure" in failed
    assert "['before error']" in recovered
    assert code_policy.get_execution_records()[-2].status == "python-error"
    assert code_policy.get_execution_records()[-2].namespace_preserved


def test_timeout_interrupts_and_preserves_responsive_kernel(
    code_policy: CodePolicyModule,
) -> None:
    timed_out = code_policy.python_exec(
        "import time\nmarker = 'survived interrupt'\ntime.sleep(10)",
        timeout_s=0.1,
    )
    recovered = code_policy.python_exec("marker")
    record = code_policy.get_execution_records()[-2]

    assert "was interrupted" in timed_out
    assert "namespace was preserved" in timed_out
    assert "'survived interrupt'" in recovered
    assert record.status == "timed-out"
    assert record.interrupt_attempted
    assert record.interrupt_recovered
    assert not record.kernel_restarted
    assert record.namespace_preserved
    assert record.remote_work_may_continue


def test_timeout_restarts_kernel_when_interrupt_does_not_recover(
    mocker, code_policy: CodePolicyModule
) -> None:
    manager = mocker.Mock()
    manager.is_alive.return_value = True
    client = mocker.Mock()
    client.execute_interactive.side_effect = TimeoutError
    client.wait_for_ready.side_effect = [TimeoutError, None]
    code_policy._kernel_manager = manager
    code_policy._kernel_client = client
    mocker.patch.object(code_policy, "_bootstrap")

    result = code_policy.python_exec("while True: pass", timeout_s=0.1)
    record = code_policy.get_execution_records()[-1]

    manager.interrupt_kernel.assert_called_once_with()
    manager.restart_kernel.assert_called_once_with(now=True)
    assert "was restarted" in result
    assert "namespace was reset" in result
    assert record.status == "timed-out"
    assert record.kernel_restarted
    assert not record.namespace_preserved
    assert code_policy._kernel_generation == 1


def test_failed_kernel_restart_does_not_advance_generation(
    mocker, code_policy: CodePolicyModule
) -> None:
    manager = mocker.Mock()
    manager.is_alive.return_value = True
    manager.restart_kernel.side_effect = RuntimeError("restart failed")
    client = mocker.Mock()
    client.execute_interactive.side_effect = TimeoutError
    client.wait_for_ready.side_effect = TimeoutError
    code_policy._kernel_manager = manager
    code_policy._kernel_client = client
    code_policy._kernel_generation = 5

    code_policy.python_exec("while True: pass", timeout_s=0.1)

    assert code_policy.get_observer_state().kernel_generation == 5


def test_python_exec_rejects_invalid_timeout_without_starting_kernel(
    mocker, code_policy: CodePolicyModule
) -> None:
    ensure_kernel = mocker.patch.object(code_policy, "_ensure_kernel")

    result = code_policy.python_exec("pass", timeout_s=111.0)

    assert "Invalid timeout" in result
    ensure_kernel.assert_not_called()
    assert code_policy.get_execution_records()[-1].status == "invalid-request"


def test_python_exec_rejects_concurrent_call(code_policy: CodePolicyModule) -> None:
    assert code_policy._execution_lock is not None
    code_policy._execution_lock.acquire()
    try:
        result = code_policy.python_exec("pass")
    finally:
        code_policy._execution_lock.release()

    assert result == "Code Policy Module busy: another python_exec call is active"
    assert code_policy.get_execution_records()[-1].status == "busy"


def test_reset_session_rejects_active_execution(
    code_policy: CodePolicyModule,
) -> None:
    assert code_policy._execution_lock is not None
    code_policy._execution_lock.acquire()
    try:
        with pytest.raises(RuntimeError, match="python_exec is active"):
            code_policy.reset_session()
    finally:
        code_policy._execution_lock.release()


def test_module_stop_prevents_new_execution_and_records_rejection(
    code_policy: CodePolicyModule,
) -> None:
    code_policy.python_exec("marker = 1")
    code_policy.stop()

    result = code_policy.python_exec("marker")

    assert result == "Code Policy Module stopped"
    assert code_policy.get_execution_records()[-1].status == "module-stopped"
