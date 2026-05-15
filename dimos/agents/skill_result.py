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

"""Structured return type for ``@skill`` methods.

Skills historically returned free-form strings, which forced agents to parse
prose to tell success from failure. ``SkillResult`` carries a typed
``error_code`` that any caller (LLM agent, RPC client, tests) can branch on.

The MCP server's ``agent_encode`` hook (``dimos/agents/mcp/mcp_server.py``)
auto-detects the method on this class and forwards its output as the JSON-RPC
``content`` field, so no MCP changes are required.

Domain-specific failure codes live with their domain
(e.g. ``dimos.manipulation.skill_errors.ManipulationError``); only codes any
domain might emit live in ``CommonSkillError`` here.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
from enum import Enum, auto
import json
import time
from typing import Any

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class CommonSkillError(Enum):
    """Failure modes any skill domain might emit."""

    ROBOT_NOT_FOUND = auto()
    INVALID_INPUT = auto()
    INVALID_STATE = auto()
    NOT_CONFIGURED = auto()
    EXECUTION_FAILED = auto()
    EXECUTION_TIMEOUT = auto()


@dataclass
class SkillResult:
    """Structured outcome of a ``@skill`` call.

    ``error_code`` accepts any ``Enum`` so each domain can supply its own
    code namespace (manipulation, locomotion, perception, ...). The wire
    representation uses ``Enum.name``, which is supported by all enums.
    """

    success: bool
    message: str = ""
    error_code: Enum | None = None
    duration_ms: float = 0.0
    metadata: dict[str, Any] = field(default_factory=dict)

    def is_success(self) -> bool:
        return self.success

    @classmethod
    def ok(cls, message: str = "", **metadata: Any) -> SkillResult:
        return cls(success=True, message=message, metadata=dict(metadata))

    @classmethod
    def fail(cls, error_code: Enum, message: str = "") -> SkillResult:
        return cls(success=False, error_code=error_code, message=message)

    def agent_encode(self) -> list[dict[str, Any]]:
        payload: dict[str, Any] = {
            "success": self.success,
            "message": self.message,
            "error_code": self.error_code.name if self.error_code is not None else None,
            "duration_ms": round(self.duration_ms, 1),
        }
        if self.metadata:
            payload["metadata"] = self.metadata
        return [{"type": "text", "text": json.dumps(payload)}]

    def __str__(self) -> str:
        if self.success:
            return f"OK: {self.message}" if self.message else "OK"
        code = self.error_code.name if self.error_code is not None else "ERROR"
        return f"{code}: {self.message}" if self.message else code


@contextmanager
def skill_timing(name: str | None = None) -> Iterator[Callable[[SkillResult], SkillResult]]:
    """Stamp a ``SkillResult`` with the elapsed wall time at exit, and log it.


    Usage::

        with skill_timing("set_gripper") as stamp:
            ...
            return stamp(SkillResult.ok("done"))
    """
    t0 = time.monotonic()

    def stamp(result: SkillResult) -> SkillResult:
        result.duration_ms = (time.monotonic() - t0) * 1000.0
        if name is not None:
            code = result.error_code.name if result.error_code is not None else "OK"
            logger.info(f"SKILL {name} result={code} duration_ms={result.duration_ms:.1f}")
        return result

    try:
        yield stamp
    except BaseException:
        if name is not None:
            duration_ms = (time.monotonic() - t0) * 1000.0
            logger.info(f"SKILL {name} result=EXCEPTION duration_ms={duration_ms:.1f}")
        raise
