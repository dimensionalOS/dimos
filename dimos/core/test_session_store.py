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

"""Tests for session_store save/load/restore/find_latest."""

from __future__ import annotations

import pytest
from langchain_core.messages import AIMessage, HumanMessage

import dimos.core.session_store as session_store_mod
from dimos.core.session_store import (
    find_latest_session,
    load_session,
    restore_session,
    save_session,
)


@pytest.fixture(autouse=True)
def _isolated_state_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(session_store_mod, "STATE_DIR", tmp_path)


class TestSaveAndLoadSession:
    def test_round_trip(self) -> None:
        run_id = "20260503-120000-demo-agent"
        history = [HumanMessage(content="hello"), AIMessage(content="hi")]
        save_session(
            run_id=run_id,
            blueprint="demo-agent",
            model="gpt-4",
            started_at="2026-05-03T12:00:00+00:00",
            original_argv=["dimos", "run", "demo-agent"],
            history=history,
        )
        messages, metadata = load_session(run_id)
        assert len(messages) == 2
        assert messages[0].content == "hello"
        assert messages[1].content == "hi"
        assert metadata["run_id"] == run_id
        assert metadata["blueprint"] == "demo-agent"


class TestFindLatestSession:
    def test_returns_none_when_no_sessions(self) -> None:
        assert find_latest_session("demo-agent") is None

    def test_returns_latest_run_id(self) -> None:
        for ts in ["20260503-100000", "20260503-120000"]:
            save_session(
                run_id=f"{ts}-demo-agent",
                blueprint="demo-agent",
                model="gpt-4",
                started_at=None,
                original_argv=[],
                history=[HumanMessage(content="hi")],
            )
        assert find_latest_session("demo-agent") == "20260503-120000-demo-agent"

    def test_returns_none_on_corrupt_file(self, tmp_path) -> None:
        bp_dir = tmp_path / "sessions" / "demo-agent"
        bp_dir.mkdir(parents=True)
        (bp_dir / "20260503-120000-demo-agent.json").write_text("not json")
        assert find_latest_session("demo-agent") is None


class TestRestoreSession:
    def test_returns_empty_when_no_restore_flag(self) -> None:
        history, parent = restore_session(
            blueprint="demo-agent", restore_session_id=None, no_restore=True
        )
        assert history == []
        assert parent is None

    def test_restores_latest_session(self) -> None:
        run_id = "20260503-120000-demo-agent"
        save_session(
            run_id=run_id,
            blueprint="demo-agent",
            model="gpt-4",
            started_at=None,
            original_argv=[],
            history=[HumanMessage(content="hello")],
        )
        history, parent = restore_session(
            blueprint="demo-agent", restore_session_id=None, no_restore=False
        )
        assert len(history) == 1
        assert history[0].content == "hello"
        assert parent == run_id

    def test_returns_empty_on_corrupt_session(self, tmp_path) -> None:
        bp_dir = tmp_path / "sessions" / "demo-agent"
        bp_dir.mkdir(parents=True)
        (bp_dir / "20260503-120000-demo-agent.json").write_text("not json")
        history, parent = restore_session(
            blueprint="demo-agent",
            restore_session_id="20260503-120000-demo-agent",
            no_restore=False,
        )
        assert history == []
        assert parent is None
