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

import json
import os
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Any, Literal
from uuid import UUID, uuid4

from filelock import FileLock, Timeout
from langchain_core.messages import BaseMessage, messages_from_dict, messages_to_dict
from pydantic import BaseModel, ConfigDict, ValidationError


class _SessionData(BaseModel):
    model_config = ConfigDict(extra="forbid")

    version: Literal[1]
    session_id: UUID
    messages: list[dict[str, Any]]


class AgentSession:
    """An exclusively owned, atomically updated conversation checkpoint."""

    def __init__(self, directory: Path, restore_session: str | None = None) -> None:
        self.session_id = (
            str(UUID(restore_session)) if restore_session is not None else str(uuid4())
        )
        self.path = directory / f"{self.session_id}.json"
        self._restoring = restore_session is not None
        # The module starts on an RPC thread but closes on its agent thread.
        self._lease = FileLock(self.path.with_suffix(".lock"), thread_local=False)

    def start(self) -> list[BaseMessage]:
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        try:
            self._lease.acquire(timeout=0)
        except Timeout as exc:
            raise RuntimeError(f"Agent session {self.session_id} is already in use") from exc
        try:
            if self._restoring:
                return self._load()
            self.save([])
            return []
        except BaseException:
            self.close()
            raise

    def _load(self) -> list[BaseMessage]:
        try:
            data = _SessionData.model_validate_json(self.path.read_text(encoding="utf-8"))
            if str(data.session_id) != self.session_id:
                raise ValueError("session ID does not match filename")
            return messages_from_dict(data.messages)
        except (ValidationError, ValueError, KeyError, TypeError) as exc:
            raise ValueError(f"Invalid agent session {self.session_id}: {exc}") from exc

    def save(self, messages: list[BaseMessage]) -> None:
        if not self._lease.is_locked:
            raise RuntimeError("Cannot save an agent session without its lease")
        data = _SessionData(
            version=1, session_id=UUID(self.session_id), messages=messages_to_dict(messages)
        )
        temporary: Path | None = None
        try:
            with NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=self.path.parent, suffix=".tmp", delete=False
            ) as output:
                temporary = Path(output.name)
                json.dump(data.model_dump(mode="json"), output, ensure_ascii=False)
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def close(self) -> None:
        self._lease.release()
