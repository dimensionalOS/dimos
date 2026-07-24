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
"""Persist McpClient conversation history to disk (issue #1898).

JSON fields:
- ``summary``: auto title from the first human message (refreshed on every save)
- ``user_summary``: optional CLI-set display name (empty by default)

``dimos agent sessions`` shows ``user_summary`` if non-empty, else ``summary``.

Follow-up: optional LLM-generated one-line summaries (ChatGPT-style).
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

from langchain_core.load import dumpd, load
from langchain_core.messages import BaseMessage

from dimos.constants import STATE_DIR

SCHEMA_VERSION = 1
_IMAGE_PLACEHOLDER = "[image omitted from session restore]"
_SUMMARY_MAX_LEN = 72


def sessions_dir() -> Path:
    path = STATE_DIR / "agent_sessions"
    path.mkdir(parents=True, exist_ok=True)
    return path


def session_path(session_id: str) -> Path:
    # Keep session files strictly under sessions_dir (no path traversal).
    safe_id = Path(session_id).name
    if safe_id != session_id or not safe_id:
        raise ValueError(f"Invalid session_id: {session_id!r}")
    return sessions_dir() / f"{safe_id}.json"


def _text_from_content(content: Any) -> str:
    if isinstance(content, str):
        return content.strip()
    if isinstance(content, list):
        parts: list[str] = []
        for block in content:
            if isinstance(block, str):
                parts.append(block)
            elif isinstance(block, dict) and block.get("type") == "text":
                parts.append(str(block.get("text", "")))
        return " ".join(p for p in parts if p).strip()
    return ""


def heuristic_summary(messages: list[BaseMessage]) -> str:
    """First human text, truncated — no LLM call."""
    for msg in messages:
        if msg.type != "human":
            continue
        text = _text_from_content(msg.content)
        if not text:
            continue
        text = " ".join(text.split())
        if len(text) > _SUMMARY_MAX_LEN:
            return text[: _SUMMARY_MAX_LEN - 1] + "…"
        return text
    return "(no user messages yet)"


def _strip_images_in_content(content: Any) -> Any:
    if not isinstance(content, list):
        return content
    cleaned: list[Any] = []
    for block in content:
        if isinstance(block, dict) and block.get("type") in ("image_url", "image"):
            cleaned.append({"type": "text", "text": _IMAGE_PLACEHOLDER})
        else:
            cleaned.append(block)
    return cleaned


def _messages_for_disk(messages: list[BaseMessage]) -> list[dict[str, Any]]:
    """Serialize messages, dropping bulky image payloads."""
    out: list[dict[str, Any]] = []
    for msg in messages:
        content = _strip_images_in_content(msg.content)
        if content is msg.content:
            dumped = dumpd(msg)
        else:
            dumped = dumpd(msg.model_copy(update={"content": content}))
        if not isinstance(dumped, dict):
            raise TypeError(f"Expected dict from dumpd, got {type(dumped)}")
        out.append(dumped)
    return out


def _messages_from_disk(raw: list[Any]) -> list[BaseMessage]:
    messages: list[BaseMessage] = []
    for item in raw:
        # allowed_objects="messages": session files only store chat messages.
        loaded = load(item, allowed_objects="messages")
        if not isinstance(loaded, BaseMessage):
            raise TypeError(f"Expected BaseMessage, got {type(loaded)}")
        messages.append(loaded)
    return messages


@dataclass(frozen=True)
class SessionInfo:
    session_id: str
    summary: str  # display summary: user_summary if set, else auto summary
    updated_at: str
    n_messages: int
    model: str | None


def _read_payload(session_id: str) -> dict[str, Any]:
    path = session_path(session_id)
    if not path.is_file():
        raise FileNotFoundError(f"No agent session saved at {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise TypeError(f"Session file must be a JSON object: {path}")
    return payload


def _write_payload(session_id: str, payload: dict[str, Any]) -> Path:
    path = session_path(session_id)
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    tmp.replace(path)
    return path


def display_summary(payload: dict[str, Any]) -> str:
    """Prefer user_summary; fall back to auto summary from first human message."""
    user = str(payload.get("user_summary") or "").strip()
    if user:
        return user
    auto = str(payload.get("summary") or "").strip()
    return auto or "(no summary)"


def save_session(
    session_id: str,
    messages: list[BaseMessage],
    *,
    model: str | None = None,
) -> Path:
    path = session_path(session_id)
    user_summary = ""
    if path.is_file():
        try:
            old = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            old = {}
        if isinstance(old, dict):
            user_summary = str(old.get("user_summary") or "")

    payload = {
        "version": SCHEMA_VERSION,
        "session_id": session_id,
        "model": model,
        "updated_at": datetime.now(timezone.utc).isoformat(),
        # Auto title from first human message — always refreshed on save.
        "summary": heuristic_summary(messages),
        # Optional user-facing name; preserved across saves unless CLI updates it.
        "user_summary": user_summary,
        "messages": _messages_for_disk(messages),
    }
    return _write_payload(session_id, payload)


def load_session(session_id: str) -> list[BaseMessage]:
    payload = _read_payload(session_id)
    version = payload.get("version", SCHEMA_VERSION)
    if version != SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported agent session schema version {version!r} "
            f"(expected {SCHEMA_VERSION}) at {session_path(session_id)}"
        )
    raw = payload.get("messages") or []
    if not isinstance(raw, list):
        raise TypeError("session messages must be a list")
    return _messages_from_disk(raw)


def update_session_summary(session_id: str, summary: str) -> Path:
    """Set ``user_summary`` (list display name). Empty string clears it."""
    text = " ".join(summary.split()).strip()
    payload = _read_payload(session_id)
    payload["user_summary"] = text
    payload["updated_at"] = datetime.now(timezone.utc).isoformat()
    return _write_payload(session_id, payload)


def delete_session(session_id: str) -> None:
    """Delete a saved session file."""
    path = session_path(session_id)
    if not path.is_file():
        raise FileNotFoundError(f"No agent session saved at {path}")
    path.unlink()


def list_sessions() -> list[SessionInfo]:
    infos: list[SessionInfo] = []
    for path in sessions_dir().glob("*.json"):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if not isinstance(payload, dict):
            continue
        session_id = str(payload.get("session_id") or path.stem)
        messages = payload.get("messages") or []
        infos.append(
            SessionInfo(
                session_id=session_id,
                summary=display_summary(payload),
                updated_at=str(payload.get("updated_at") or ""),
                n_messages=len(messages) if isinstance(messages, list) else 0,
                model=payload.get("model"),
            )
        )
    infos.sort(key=lambda s: s.updated_at, reverse=True)
    return infos
