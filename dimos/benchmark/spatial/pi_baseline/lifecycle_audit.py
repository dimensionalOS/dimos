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

# Copyright 2026 Dimensional Inc.
"""Bounded, typed lifecycle facts for the native adapter controller."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from typing import Literal

AuditDirection = Literal["in", "out"]


@dataclass(frozen=True)
class LifecycleAuditRecord:
    """The deliberately small public vocabulary of the private audit."""

    schema_version: str
    sequence: int
    direction: AuditDirection
    frame_type: str
    run_id: str | None = None
    tool: str | None = None
    call_id: str | None = None
    reply_ok: bool | None = None
    terminal_category: str | None = None
    reason_code: str | None = None
    session_state: str | None = None
    session_persisted: bool | None = None
    session_byte_count: int | None = None
    system_prompt_byte_count: int | None = None
    initial_prompt_byte_count: int | None = None
    system_prompt_sha256: str | None = None
    initial_prompt_sha256: str | None = None

    def encoded(self) -> bytes:
        values = {key: value for key, value in asdict(self).items() if value is not None}
        return json.dumps(values, separators=(",", ":"), sort_keys=True).encode() + b"\n"


def bounded_audit(records: list[LifecycleAuditRecord], limit: int) -> bytes:
    """Encode records only after enforcing a record-count and byte bound."""
    if len(records) > limit:
        raise ValueError("adapter lifecycle audit exceeds record bound")
    payload = b"".join(record.encoded() for record in records)
    if len(payload) > limit * 2048:
        raise ValueError("adapter lifecycle audit exceeds byte bound")
    return payload
