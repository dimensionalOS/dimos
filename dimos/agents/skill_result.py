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

Error codes are plain strings constrained by ``Literal`` types — each domain
declares its own alias (see e.g. ``dimos.manipulation.skill_errors``) and a
skill annotates ``SkillResult[DomainError]`` so the type checker enforces that
only that domain's codes are emitted. ``CommonSkillError`` holds codes that
any domain might emit.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field, replace
import json
import time
from typing import Any, Generic, Literal

# typing_extensions for PEP 696 TypeVar default support on Python < 3.13.
from typing_extensions import TypeVar

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


CommonSkillError = Literal[
    "ROBOT_NOT_FOUND",
    "INVALID_INPUT",
    "INVALID_STATE",
    "NOT_CONFIGURED",
    "EXECUTION_FAILED",
    "EXECUTION_TIMEOUT",
]


E = TypeVar("E", bound=str, default=str)


@dataclass
class SkillResult(Generic[E]):
    """Structured outcome of a ``@skill`` call.

    Parameterize the class with the domain's error-code alias to constrain
    ``error_code`` (e.g. ``SkillResult[ManipulationError]``). Unparameterized,
    any string is accepted.
    """

    success: bool
    message: str = ""
    error_code: E | None = None
    duration_ms: float = 0.0
    metadata: dict[str, Any] = field(default_factory=dict)

    def is_success(self) -> bool:
        return self.success

    @classmethod
    def ok(cls, message: str = "", **metadata: Any) -> SkillResult[E]:
        return cls(success=True, message=message, metadata=dict(metadata))

    @classmethod
    def fail(cls, error_code: E, message: str = "") -> SkillResult[E]:
        return cls(success=False, error_code=error_code, message=message)

    def agent_encode(self) -> list[dict[str, Any]]:
        payload: dict[str, Any] = {
            "success": self.success,
            "message": self.message,
            "error_code": self.error_code,
            "duration_ms": round(self.duration_ms, 1),
        }
        if self.metadata:
            payload["metadata"] = self.metadata
        return [{"type": "text", "text": json.dumps(payload)}]

    def __str__(self) -> str:
        if self.success:
            return f"OK: {self.message}" if self.message else "OK"
        code = self.error_code if self.error_code is not None else "ERROR"
        return f"{code}: {self.message}" if self.message else code


@contextmanager
def skill_timing(name: str | None = None) -> Iterator[Callable[[SkillResult[E]], SkillResult[E]]]:
    """Stamp a ``SkillResult`` with the elapsed wall time at exit, and log it.

    Usage::

        with skill_timing("set_gripper") as stamp:
            ...
            return stamp(SkillResult.ok("done"))
    """
    t0 = time.monotonic()

    def stamp(result: SkillResult[E]) -> SkillResult[E]:
        stamped = replace(result, duration_ms=(time.monotonic() - t0) * 1000.0)
        if name is not None:
            code = stamped.error_code if stamped.error_code is not None else "OK"
            logger.info(f"SKILL {name} result={code} duration_ms={stamped.duration_ms:.1f}")
        return stamped

    try:
        yield stamp
    except BaseException:
        # BaseException covers KeyboardInterrupt / SystemExit too. We re-raise;
        # the caller (typically the MCP server) handles the traceback. This
        # log line just makes every skill entry produce one structured record.
        if name is not None:
            duration_ms = (time.monotonic() - t0) * 1000.0
            logger.info(f"SKILL {name} result=EXCEPTION duration_ms={duration_ms:.1f}")
        raise
