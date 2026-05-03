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

"""Persistent storage for agent conversation history across dimos runs.

Layout on disk:
    ~/.local/state/dimos/sessions/
        <blueprint>/
            <timestamp>.json
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
import re
from pathlib import Path

from langchain_core.messages import messages_from_dict, messages_to_dict
from langchain_core.messages.base import BaseMessage

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _get_sessions_dir() -> Path:
    xdg = os.environ.get("XDG_STATE_HOME")
    base = Path(xdg) / "dimos" if xdg else Path.home() / ".local" / "state" / "dimos"
    return base / "sessions"


SESSIONS_DIR = _get_sessions_dir()

# run_id format: "20260502-143022-<blueprint>"
_RUN_ID_RE = re.compile(r"^(\d{8}-\d{6})-(.*)")


def _parse_run_id(run_id: str) -> tuple[str, str]:
    """Return (timestamp, blueprint) from a run_id."""
    m = _RUN_ID_RE.match(run_id)
    if not m:
        raise ValueError(f"Cannot parse run_id: {run_id!r}")
    return m.group(1), m.group(2)


def _session_path(run_id: str) -> Path:
    timestamp, blueprint = _parse_run_id(run_id)
    return SESSIONS_DIR / blueprint / f"{timestamp}.json"


def save_session(
    run_id: str,
    blueprint: str,
    model: str,
    started_at: str | None,
    original_argv: list[str],
    history: list[BaseMessage],
    parent_session_id: str | None = None,
) -> None:
    """Persist agent conversation history and session metadata to disk."""
    path = _session_path(run_id)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "run_id": run_id,
        "blueprint": blueprint,
        "model": model,
        "started_at": started_at,
        "ended_at": datetime.now(timezone.utc).isoformat(),
        "original_argv": original_argv,
        "parent_session_id": parent_session_id,
        "messages": messages_to_dict(history),
    }
    path.write_text(json.dumps(data, indent=2))
    logger.info("Saved agent session.", run_id=run_id, n_messages=len(history))


def load_session(run_id: str) -> tuple[list[BaseMessage], dict[str, object]]:
    """Load agent history and metadata by run_id.

    Returns (messages, metadata) where metadata contains all non-message fields.
    Raises FileNotFoundError if the session does not exist.
    """
    path = _session_path(run_id)
    data = json.loads(path.read_text())
    messages = messages_from_dict(data["messages"])
    metadata = {k: v for k, v in data.items() if k != "messages"}
    logger.info("Restored agent session.", run_id=run_id, n_messages=len(messages))
    return messages, metadata


def restore_session(
    blueprint: str,
    restore_session_id: str | None,
    no_restore: bool,
) -> tuple[list[BaseMessage], str | None]:
    """Restore history from a previous session.

    Returns (history, parent_session_id). Returns ([], None) if nothing to restore.
    """
    if no_restore or not blueprint:
        return [], None
    session_id = restore_session_id or find_latest_session(blueprint)
    if not session_id:
        return [], None
    try:
        history, metadata = load_session(session_id)
        return history, str(metadata["run_id"])
    except FileNotFoundError:
        return [], None


def find_latest_session(blueprint: str) -> str | None:
    """Return the run_id of the most recent saved session for a blueprint, or None."""
    bp_dir = SESSIONS_DIR / blueprint
    if not bp_dir.exists():
        return None
    files = sorted(bp_dir.glob("*.json"))
    if not files:
        return None
    try:
        data = json.loads(files[-1].read_text())
        return str(data["run_id"])
    except Exception:
        return None


