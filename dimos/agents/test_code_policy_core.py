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
    CodePolicySession,
    CodePolicySessionConfig,
    EmptyEnvironment,
    FrozenMemoryEnvironment,
    LiveDimosEnvironment,
    _kernel_environment,
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


def test_empty_session_runs_python_without_a_memory_object() -> None:
    session = CodePolicySession(CodePolicySessionConfig(environment=EmptyEnvironment()))
    session.start()
    try:
        assert "3" in session.python_exec("1 + 2")
        assert "NameError" in session.python_exec("memory")
    finally:
        session.stop()


def test_empty_environment_hides_inherited_recording_paths(monkeypatch) -> None:
    monkeypatch.setenv("DIMOS_CODE_POLICY_RECORDING_PATH", "/host/recording.db")
    monkeypatch.setenv("DIMOS_CODE_POLICY_MEMORY_CUTOFF", "1.0")
    result = _kernel_environment(EmptyEnvironment())
    assert "DIMOS_CODE_POLICY_RECORDING_PATH" not in result
    assert "DIMOS_CODE_POLICY_MEMORY_CUTOFF" not in result
    assert result["DIMOS_CODE_POLICY_CONNECT_APP"] == "0"


def test_empty_environment_hides_the_variables_its_caller_names(monkeypatch) -> None:
    """A benchmark case must not be able to read where its own answers are kept."""
    monkeypatch.setenv("DIMOS_TEST_ANSWER_DIR", "/runs/SAtt_text/20260809")
    monkeypatch.setenv("DIMOS_TEST_KEPT", "kept")

    result = _kernel_environment(EmptyEnvironment(hidden_env=("DIMOS_TEST_ANSWER_DIR",)))

    assert "DIMOS_TEST_ANSWER_DIR" not in result
    assert result["DIMOS_TEST_KEPT"] == "kept"


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
