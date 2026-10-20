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

import json
from typing import Any, Literal, TypeAlias

from pydantic import BaseModel, FiniteFloat

EpisodeEvent: TypeAlias = Literal["start", "save", "discard", "init"]
RecordingState: TypeAlias = Literal["idle", "recording"]


class EpisodeStatus(BaseModel):
    """Internal episode state and the version-1 JSON recording document."""

    ts: FiniteFloat
    state: RecordingState
    episodes_saved: int
    episodes_discarded: int
    last_event: EpisodeEvent = "init"
    task_label: str | None = None

    def to_json(self) -> str:
        """Serialize a recording document, independently of its transport."""
        return json.dumps({"schema_version": 1, **self.model_dump(mode="json")}, allow_nan=False)

    @classmethod
    def from_json(cls, data: str) -> EpisodeStatus:
        """Validate a recorded event or a live String payload."""
        payload = json.loads(data)
        if not isinstance(payload, dict) or "schema_version" not in payload:
            raise ValueError("EpisodeStatus JSON requires schema_version")
        version = payload.pop("schema_version")
        if type(version) is not int or version != 1:
            raise ValueError("Unsupported EpisodeStatus schema_version")
        return cls.model_validate(payload)

    @classmethod
    def json_schema(cls) -> dict[str, Any]:
        """Describe the stored JSON document for MCAP readers."""
        schema = cls.model_json_schema()
        schema["$id"] = "imitation.episode-status.v1"
        schema["properties"]["schema_version"] = {"type": "integer", "const": 1}
        schema["required"].append("schema_version")
        return schema
