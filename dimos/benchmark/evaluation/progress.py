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

"""Presentation-only progress contracts for interactive evaluation runs."""

from __future__ import annotations

from collections.abc import Callable
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field


class ProgressModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class StatusProgress(ProgressModel):
    kind: Literal["status"] = "status"
    channel: Literal["eval", "pi"]
    message: str = Field(min_length=1)


class CaseHeaderProgress(ProgressModel):
    kind: Literal["case_header"] = "case_header"
    case_id: str = Field(min_length=1)
    source: str = Field(min_length=1)
    progress: float | None = Field(default=None, ge=0, le=1)
    question: str = Field(min_length=1)


class AssistantTextProgress(ProgressModel):
    kind: Literal["assistant_text"] = "assistant_text"
    delta: str = Field(min_length=1)


class ToolStartProgress(ProgressModel):
    kind: Literal["tool_start"] = "tool_start"
    code: str = Field(min_length=1)


class ToolEndProgress(ProgressModel):
    kind: Literal["tool_end"] = "tool_end"
    ok: bool
    result: str
    duration_seconds: float = Field(ge=0)


class FinalResponseProgress(ProgressModel):
    kind: Literal["final_response"] = "final_response"
    text: str


EvalProgress = Annotated[
    StatusProgress
    | CaseHeaderProgress
    | AssistantTextProgress
    | ToolStartProgress
    | ToolEndProgress
    | FinalResponseProgress,
    Field(discriminator="kind"),
]
ProgressSink = Callable[[EvalProgress], None]


def emit_progress(sink: ProgressSink | None, event: EvalProgress) -> None:
    """Notify a presentation observer without allowing it to affect evaluation."""
    if sink is None:
        return
    try:
        sink(event)
    except Exception:
        return
