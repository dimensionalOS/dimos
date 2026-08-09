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

from pathlib import Path

from dimos.agents.code_policy_core import (
    CodeExecutionRecord,
    CodePolicySession,
    CodePolicySessionConfig,
    FrozenMemoryEnvironment,
    LiveDimosEnvironment,
    _kernel_environment,
    latest_policy_source,
    validate_policy_source,
)
from dimos.memory2.store.sqlite import SqliteStore


def test_session_persists_python_namespace(mocker, tmp_path: Path) -> None:
    mocker.patch("dimos.agents.code_policy_core._bootstrap_source", return_value="pass")
    session = CodePolicySession(
        CodePolicySessionConfig(
            environment=LiveDimosEnvironment(recording_path=str(tmp_path / "unused.db"))
        )
    )
    session.start()
    try:
        assert "[1]" in session.python_exec("items = [1]\nitems")
        assert "[1, 2]" in session.python_exec("items.append(2)\nitems")
        assert session.execution_count == 2
    finally:
        session.stop()


def test_new_session_starts_with_a_fresh_namespace(mocker, tmp_path: Path) -> None:
    mocker.patch("dimos.agents.code_policy_core._bootstrap_source", return_value="pass")
    config = CodePolicySessionConfig(
        environment=LiveDimosEnvironment(recording_path=str(tmp_path / "unused.db"))
    )
    first = CodePolicySession(config)
    first.start()
    try:
        first.python_exec("session_only = 1")
    finally:
        first.stop()

    second = CodePolicySession(config)
    second.start()
    try:
        assert "False" in second.python_exec("'session_only' in globals()")
    finally:
        second.stop()


def test_frozen_session_exposes_bounded_memory_without_app(tmp_path: Path) -> None:
    source_path = tmp_path / "source.db"
    derived_path = tmp_path / "derived.db"
    with SqliteStore(path=str(source_path)) as source:
        source.stream("messages", str).append("before", ts=1.0)
        source.stream("messages", str).append("after", ts=3.0)
    with SqliteStore(path=str(derived_path)) as derived:
        derived.stream("global_map", str).append("map", ts=2.0)

    session = CodePolicySession(
        CodePolicySessionConfig(
            environment=FrozenMemoryEnvironment(
                recording_path=str(source_path),
                derived_recording_path=str(derived_path),
                memory_cutoff_timestamp=2.0,
            )
        )
    )
    session.start()
    try:
        result = session.python_exec(
            "([item.data for item in memory.streams.messages], "
            "memory.streams.global_map.last().data, 'app' in globals())"
        )
        assert "(['before'], 'map', False)" in result
        assert "PermissionError" in session.python_exec("memory.streams.messages.append('blocked')")
    finally:
        session.stop()


def test_kernel_environment_does_not_forward_credentials(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "secret")
    monkeypatch.setenv("SOME_AUTH_TOKEN", "secret")
    monkeypatch.setenv("CODE_POLICY_TEST_VALUE", "safe")
    environment = LiveDimosEnvironment(recording_path=str(tmp_path / "memory.db"))
    result = _kernel_environment(environment)
    assert "OPENAI_API_KEY" not in result
    assert "SOME_AUTH_TOKEN" not in result
    assert result["CODE_POLICY_TEST_VALUE"] == "safe"


def test_timeout_interrupts_kernel_and_keeps_session_usable(mocker, tmp_path: Path) -> None:
    mocker.patch("dimos.agents.code_policy_core._bootstrap_source", return_value="pass")
    session = CodePolicySession(
        CodePolicySessionConfig(
            environment=LiveDimosEnvironment(recording_path=str(tmp_path / "unused.db")),
            interrupt_grace_s=2,
        )
    )
    session.start()
    try:
        assert "timed out" in session.python_exec("while True: pass", timeout_s=0.1)
        assert "2" in session.python_exec("1 + 1")
    finally:
        session.stop()


def test_latest_policy_source_selects_newest_successful_zero_argument_definition() -> None:
    records = [
        CodeExecutionRecord("def policy():\n    return 1", "completed", "", 0.1),
        CodeExecutionRecord("def policy(value):\n    return value", "completed", "", 0.1),
        CodeExecutionRecord("async def policy():\n    return 2", "completed", "", 0.1),
        CodeExecutionRecord("def policy():\n    return 3", "failed", "", 0.1),
        CodeExecutionRecord(
            "import math\n\ndef helper():\n    return math.floor(4.5)\n\n"
            "def policy():\n    return helper()",
            "completed",
            "",
            0.1,
        ),
    ]

    source = latest_policy_source(records)

    assert source == records[-1].source


def test_latest_policy_source_returns_none_without_conforming_definition() -> None:
    records = [
        CodeExecutionRecord("value = 1", "completed", "", 0.1),
        CodeExecutionRecord("def policy(value):\n    return value", "completed", "", 0.1),
    ]

    assert latest_policy_source(records) is None


def test_validate_policy_source_replays_with_frozen_memory(tmp_path: Path) -> None:
    source_path = tmp_path / "source.db"
    derived_path = tmp_path / "derived.db"
    with SqliteStore(path=str(source_path)) as source:
        source.stream("messages", int).append(7, ts=1.0)
    with SqliteStore(path=str(derived_path)):
        pass
    config = CodePolicySessionConfig(
        environment=FrozenMemoryEnvironment(
            recording_path=str(source_path),
            derived_recording_path=str(derived_path),
            memory_cutoff_timestamp=2.0,
        )
    )

    result = validate_policy_source(
        config,
        "def policy():\n    return memory.streams.messages.last().data",
    )

    assert result.valid is True
    assert result.result == 7
    assert result.error is None


def test_validate_policy_source_reports_missing_authoring_state(mocker, tmp_path: Path) -> None:
    mocker.patch("dimos.agents.code_policy_core._bootstrap_source", return_value="pass")
    config = CodePolicySessionConfig(
        environment=LiveDimosEnvironment(recording_path=str(tmp_path / "unused.db"))
    )

    result = validate_policy_source(config, "def policy():\n    return earlier_value")

    assert result.valid is False
    assert result.error is not None
    assert "NameError" in result.error


def test_validate_policy_source_reports_non_json_result(mocker, tmp_path: Path) -> None:
    mocker.patch("dimos.agents.code_policy_core._bootstrap_source", return_value="pass")
    config = CodePolicySessionConfig(
        environment=LiveDimosEnvironment(recording_path=str(tmp_path / "unused.db"))
    )

    result = validate_policy_source(config, "def policy():\n    return object()")

    assert result.valid is False
    assert result.error is not None
    assert "JSON serializable" in result.error


def test_validate_policy_source_reports_policy_exception(mocker, tmp_path: Path) -> None:
    mocker.patch("dimos.agents.code_policy_core._bootstrap_source", return_value="pass")
    config = CodePolicySessionConfig(
        environment=LiveDimosEnvironment(recording_path=str(tmp_path / "unused.db"))
    )

    result = validate_policy_source(
        config,
        "def policy():\n    raise RuntimeError('broken policy')",
    )

    assert result.valid is False
    assert result.error is not None
    assert "RuntimeError" in result.error
    assert "broken policy" in result.error


def test_validate_policy_source_reports_timeout(mocker, tmp_path: Path) -> None:
    mocker.patch("dimos.agents.code_policy_core._bootstrap_source", return_value="pass")
    config = CodePolicySessionConfig(
        environment=LiveDimosEnvironment(recording_path=str(tmp_path / "unused.db"))
    )

    result = validate_policy_source(
        config,
        "def policy():\n    while True:\n        pass",
        timeout_s=0.1,
    )

    assert result.valid is False
    assert result.error is not None
    assert "timed out" in result.error
