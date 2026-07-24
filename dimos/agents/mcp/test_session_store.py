# Copyright 2025-2026 Dimensional Inc.
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

import json
from pathlib import Path

from langchain_core.messages import AIMessage, HumanMessage
import pytest

from dimos.agents.mcp import session_store


@pytest.fixture()
def sessions_root(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    root = tmp_path / "agent_sessions"
    monkeypatch.setattr(session_store, "sessions_dir", lambda: root)
    root.mkdir(parents=True, exist_ok=True)
    return root


def test_save_load_roundtrip(sessions_root: Path) -> None:
    messages = [
        HumanMessage(content="add 3 and 5 with the tool"),
        AIMessage(content="I'll call sum_numbers"),
    ]
    session_store.save_session("sess-1", messages, model="gpt-4o")
    loaded = session_store.load_session("sess-1")
    assert len(loaded) == 2
    assert loaded[0].content == "add 3 and 5 with the tool"
    assert loaded[1].content == "I'll call sum_numbers"


def test_heuristic_summary_uses_first_human_message() -> None:
    messages = [
        AIMessage(content="ignore me"),
        HumanMessage(content="  go to the kitchen please  "),
        HumanMessage(content="second"),
    ]
    assert session_store.heuristic_summary(messages) == "go to the kitchen please"


def test_heuristic_summary_truncates_long_text() -> None:
    long = "x" * 200
    summary = session_store.heuristic_summary([HumanMessage(content=long)])
    assert summary.endswith("…")
    assert len(summary) == session_store._SUMMARY_MAX_LEN


def test_images_stripped_on_save(sessions_root: Path) -> None:
    messages = [
        HumanMessage(
            content=[
                {"type": "text", "text": "see this"},
                {
                    "type": "image_url",
                    "image_url": {"url": "data:image/jpeg;base64,AAAA"},
                },
            ]
        )
    ]
    session_store.save_session("sess-img", messages)
    loaded = session_store.load_session("sess-img")
    assert isinstance(loaded[0].content, list)
    types = [b.get("type") for b in loaded[0].content if isinstance(b, dict)]
    assert "image_url" not in types
    assert any(
        isinstance(b, dict) and "omitted" in str(b.get("text", "")) for b in loaded[0].content
    )


def test_list_sessions_includes_summary(sessions_root: Path) -> None:
    session_store.save_session(
        "aaa",
        [HumanMessage(content="first chat")],
        model="gpt-4o",
    )
    session_store.save_session(
        "bbb",
        [HumanMessage(content="second chat")],
        model="gpt-4o",
    )
    listed = session_store.list_sessions()
    assert [s.session_id for s in listed] == ["bbb", "aaa"] or {s.session_id for s in listed} == {
        "aaa",
        "bbb",
    }
    by_id = {s.session_id: s for s in listed}
    assert by_id["aaa"].summary == "first chat"
    assert by_id["bbb"].n_messages == 1


def test_load_missing_session_raises(sessions_root: Path) -> None:
    with pytest.raises(FileNotFoundError):
        session_store.load_session("does-not-exist")


def test_update_session_summary_uses_user_summary_field(sessions_root: Path) -> None:
    session_store.save_session("sess-name", [HumanMessage(content="original first line")])
    path = session_store.session_path("sess-name")
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["summary"] == "original first line"
    assert payload.get("user_summary", "") == ""

    session_store.update_session_summary("sess-name", "厨房导航实验")
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["summary"] == "original first line"
    assert payload["user_summary"] == "厨房导航实验"
    assert session_store.list_sessions()[0].summary == "厨房导航实验"

    # Auto summary still refreshes; display keeps user_summary.
    session_store.save_session(
        "sess-name",
        [
            HumanMessage(content="original first line"),
            AIMessage(content="ok"),
            HumanMessage(content="another user line"),
        ],
    )
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["summary"] == "original first line"
    assert payload["user_summary"] == "厨房导航实验"
    assert session_store.list_sessions()[0].summary == "厨房导航实验"

    # Empty user_summary clears the custom name → fall back to auto summary.
    session_store.update_session_summary("sess-name", "")
    assert session_store.list_sessions()[0].summary == "original first line"


def test_delete_session_removes_file(sessions_root: Path) -> None:
    session_store.save_session("sess-del", [HumanMessage(content="bye")])
    session_store.delete_session("sess-del")
    assert session_store.list_sessions() == []
    with pytest.raises(FileNotFoundError):
        session_store.delete_session("sess-del")


def test_display_summary_prefers_user_summary() -> None:
    assert (
        session_store.display_summary(
            {"summary": "auto", "user_summary": "custom"}
        )
        == "custom"
    )
    assert session_store.display_summary({"summary": "auto", "user_summary": ""}) == "auto"
    assert session_store.display_summary({"summary": "", "user_summary": ""}) == "(no summary)"


def test_update_session_summary_missing_raises(sessions_root: Path) -> None:
    with pytest.raises(FileNotFoundError):
        session_store.update_session_summary("missing", "x")


def test_invalid_session_id_rejected() -> None:
    with pytest.raises(ValueError):
        session_store.session_path("../escape")
    with pytest.raises(ValueError):
        session_store.session_path("")


def test_load_rejects_bad_schema_version(sessions_root: Path) -> None:
    session_store.save_session("sess-ver", [HumanMessage(content="hi")])
    path = session_store.session_path("sess-ver")
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["version"] = 999
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="Unsupported"):
        session_store.load_session("sess-ver")


def test_load_rejects_corrupt_json(sessions_root: Path) -> None:
    path = sessions_root / "sess-bad.json"
    path.write_text("{not-json", encoding="utf-8")
    with pytest.raises(json.JSONDecodeError):
        session_store.load_session("sess-bad")


def test_list_skips_corrupt_files(sessions_root: Path) -> None:
    session_store.save_session("good", [HumanMessage(content="ok")])
    (sessions_root / "junk.json").write_text("{broken", encoding="utf-8")
    listed = session_store.list_sessions()
    assert [s.session_id for s in listed] == ["good"]
