#!/usr/bin/env python3
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

from collections import deque
from dataclasses import dataclass
import time
from typing import Any, Protocol

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.spec.utils import Spec

RiskLevel = str


class SkillOutcomeStoreSpec(Spec, Protocol):
    """RPC surface used by Layer 3 modules that need recent skill history.

    The store itself is Go2-specific for now, but consumers depend on this
    small Spec instead of the concrete class. That keeps ContextProvider and
    SkillOutcomePredictor decoupled from the storage implementation.
    """

    def get_recent_outcomes(
        self, limit: int = 5, skill_name: str = "", domain: str = ""
    ) -> list[dict[str, Any]]: ...


@dataclass(frozen=True)
class _SkillOutcome:
    """One recorded skill result.

    This is intentionally a small, JSON-shaped event. We store only the fields
    needed by Layer 3 routing/prediction, not raw robot messages or large
    artifacts. The event is converted to a dict before crossing RPC/MCP
    boundaries.
    """

    timestamp: float
    skill_name: str
    success: bool
    domain: str
    error_code: str
    message: str
    risk: RiskLevel
    recovery: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": round(self.timestamp, 3),
            "skill_name": self.skill_name,
            "success": self.success,
            "domain": self.domain,
            "error_code": self.error_code,
            "message": self.message,
            "risk": self.risk,
            "recovery": self.recovery,
        }


class _Go2SkillOutcomeStore(Module):
    """In-memory Layer 3 store for recent skill outcomes.

    Where data lives:
        Outcomes are stored in ``self._outcomes``, a bounded in-memory deque
        inside this module instance. In the normal DimOS runtime this module
        runs in a worker process, so the data is process-local.

    Persistence:
        V2 first pass is deliberately non-persistent. Restarting the blueprint
        clears the history. This avoids introducing database/schema decisions
        before the outcome payload stabilizes.

    How data is written:
        McpClient automatically calls ``record_skill_outcome(...)`` after
        non-internal tool calls when this store is present in the MCP tool
        registry. The skill remains available for manual recording of external
        events or outcomes that did not flow through the agent's MCP tool path.
    """

    _max_outcomes = 100

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Bounded memory: when the 101st event is appended, the oldest one is
        # dropped automatically. This keeps the store useful for recent context
        # without becoming an unbounded log.
        self._outcomes: deque[_SkillOutcome] = deque(maxlen=self._max_outcomes)

    @skill
    def record_skill_outcome(
        self,
        skill_name: str,
        success: bool,
        domain: str = "",
        error_code: str = "",
        message: str = "",
        risk: str = "unknown",
        recovery: str = "",
    ) -> SkillResult:
        """Record the outcome of a skill call for later routing and prediction.

        Use this after a robot skill returns, especially when the result failed
        or contains useful recovery information. This tool only records the
        outcome; it does not execute or retry the skill.

        Args:
            skill_name: Name of the tool or skill that just ran.
            success: Whether the skill succeeded.
            domain: Optional expert domain, such as navigation or perception.
            error_code: Optional structured failure code.
            message: Short result message from the skill.
            risk: Optional observed risk level.
            recovery: Optional recovery suggestion.
        """
        skill_name = skill_name.strip()
        domain = domain.strip()
        error_code = error_code.strip()
        message = message.strip()
        recovery = recovery.strip()

        # Keep the MCP-facing API simple and defensive. A missing skill name
        # would make later filtering/prediction meaningless.
        if not skill_name:
            return SkillResult.fail("INVALID_INPUT", "skill_name is required")

        # ``risk`` is a string instead of Literal[...] because MCP/tool schema
        # generation is more reliable with primitive annotations. Validate the
        # accepted vocabulary manually here.
        if risk not in ("low", "medium", "high", "unknown"):
            return SkillResult.fail("INVALID_INPUT", f"invalid risk level: {risk}")

        # Timestamp at record time, not at skill start. This is enough for
        # "recent outcome" reasoning; CausalWorldModel can add richer timing
        # later if it needs before/after transitions.
        outcome = _SkillOutcome(
            timestamp=time.time(),
            skill_name=skill_name,
            success=success,
            domain=domain,
            error_code=error_code,
            message=message,
            risk=risk,
            recovery=recovery,
        )

        # The only write path in V2 first pass: append to the bounded deque.
        # There is no file/db write here.
        self._outcomes.append(outcome)
        return SkillResult.ok(
            f"Recorded outcome for {skill_name}",
            outcome=outcome.to_dict(),
            total_outcomes=len(self._outcomes),
        )

    @skill
    def summarize_skill_outcomes(
        self, limit: int = 5, skill_name: str = "", domain: str = ""
    ) -> SkillResult:
        """Summarize recent skill outcomes.

        Args:
            limit: Maximum number of recent outcomes to include.
            skill_name: Optional skill name filter.
            domain: Optional expert domain filter.
        """
        outcomes = self.get_recent_outcomes(limit=limit, skill_name=skill_name, domain=domain)
        if not outcomes:
            return SkillResult.ok("No recorded skill outcomes", outcomes=[])

        failures = [outcome for outcome in outcomes if not outcome["success"]]
        message = (
            f"{len(outcomes)} recent outcome(s), "
            f"{len(failures)} failure(s), "
            f"filters skill_name={skill_name or '*'} domain={domain or '*'}"
        )
        return SkillResult.ok(message, outcomes=outcomes)

    @rpc
    def get_recent_outcomes(
        self, limit: int = 5, skill_name: str = "", domain: str = ""
    ) -> list[dict[str, Any]]:
        """Return newest outcomes first, optionally filtered.

        This method is RPC-only, not a skill. Other Layer 3 modules call it via
        Spec injection. Human/LLM users should normally use
        ``summarize_skill_outcomes`` instead.
        """
        limit = max(0, min(limit, self._max_outcomes))
        skill_name = skill_name.strip()
        domain = domain.strip()

        # Iterate newest-to-oldest so predictors see the most relevant failures
        # first. Filters are exact-match to avoid surprising fuzzy behavior.
        outcomes = [
            outcome
            for outcome in reversed(self._outcomes)
            if (not skill_name or outcome.skill_name == skill_name)
            and (not domain or outcome.domain == domain)
        ]
        return [outcome.to_dict() for outcome in outcomes[:limit]]


__all__ = ["SkillOutcomeStoreSpec", "_Go2SkillOutcomeStore"]
