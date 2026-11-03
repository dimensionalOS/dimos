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

"""Typed run paths and the configuration exchanged with Pi."""

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, Field

from dimos.constants import CACHE_DIR, CONFIG_DIR

Provider = Literal["openai", "anthropic", "google"]
Thinking = Literal["off", "minimal", "low", "medium", "high", "xhigh", "max"]


@dataclass(frozen=True)
class RunPaths:
    workspace: Path
    config: Path
    cache: Path

    @classmethod
    def for_run(cls, workspace: Path) -> "RunPaths":
        identity = hashlib.sha256(str(workspace.resolve()).encode()).hexdigest()[:20]
        return cls(workspace, CONFIG_DIR / "evals" / identity, CACHE_DIR / "evals")


class RuntimeConfig(BaseModel):
    provider: Provider
    base_url: str
    key_env: str
    allowed_tools: tuple[str, ...] | None = None
    max_output_tokens: int | None = Field(default=None, ge=1)
    excluded_keywords: tuple[str, ...] = ()
    ignored_paths: tuple[str, ...] = ()
    max_tool_seconds: float | None = Field(default=None, gt=0)


class ToolPolicyState(BaseModel):
    tools: tuple[str, ...]
    unknown: tuple[str, ...]
    blocked: int = 0
