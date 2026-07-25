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

from dataclasses import asdict, dataclass
from enum import StrEnum
import json
import math


class RecoveryCause(StrEnum):
    """Why normal path following asked the global planner for recovery."""

    PROGRESS_TIMEOUT = "progress_timeout"
    OBSTACLE = "obstacle"
    PATH_DEVIATION = "path_deviation"
    PLANNER_ERROR = "planner_error"


class RecoveryAction(StrEnum):
    """One atomic recovery action selected by the supervisor."""

    REPLAN = "replan"
    ROTATE_RESCAN = "rotate_rescan"
    CLEAR_STALE_LOCAL = "clear_stale_local"
    BACK_UP = "back_up"
    ALTERNATE_APPROACH = "alternate_approach"
    FAIL = "fail"


class RecoveryOutcome(StrEnum):
    """Observable outcome of dispatching or completing a recovery action."""

    DISPATCHED = "dispatched"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class RecoveryCapabilities:
    """Safety prerequisites supplied by the runtime, never inferred here."""

    can_clear_stale_local_observations: bool = False
    rear_clearance_verified: bool = False
    alternate_approach_available: bool = False


@dataclass(frozen=True)
class RecoveryDecision:
    action: RecoveryAction
    reason: str
    heading_offset_radians: float | None = None
    skipped_actions: tuple[RecoveryAction, ...] = ()
    failure_reason: str | None = None


@dataclass(frozen=True)
class RecoveryEvent:
    """Transport-neutral recovery diagnostic."""

    attempt: int
    cause: RecoveryCause
    action: RecoveryAction
    outcome: RecoveryOutcome
    reason: str
    timestamp: float

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(",", ":"), sort_keys=True)


class RecoverySupervisor:
    """Pure, bounded recovery policy.

    The supervisor only chooses an action. It cannot publish velocity, clear a
    map, or claim that a safety prerequisite is true.
    """

    def __init__(self, max_attempts: int = 8) -> None:
        if max_attempts < 1:
            raise ValueError("max_attempts must be at least one")
        self._max_attempts = max_attempts

    def decide(
        self,
        *,
        attempt: int,
        cause: RecoveryCause,
        capabilities: RecoveryCapabilities,
    ) -> RecoveryDecision:
        del cause  # Cause is recorded in the event; the first policy is shared.

        if attempt < 0:
            raise ValueError("attempt cannot be negative")

        if attempt >= self._max_attempts:
            return RecoveryDecision(
                action=RecoveryAction.FAIL,
                reason="same-area recovery attempt limit reached",
                failure_reason="attempts_exhausted",
            )

        if attempt == 0:
            return RecoveryDecision(RecoveryAction.REPLAN, "request a fresh path")
        if attempt == 1:
            return self._scan(90.0)
        if attempt == 2:
            return RecoveryDecision(RecoveryAction.REPLAN, "retry after refreshed perception")
        if attempt == 3:
            return self._scan(-90.0)
        if attempt == 4:
            if capabilities.can_clear_stale_local_observations:
                return RecoveryDecision(
                    RecoveryAction.CLEAR_STALE_LOCAL,
                    "clear only observations proven stale in the local layer",
                )
            return RecoveryDecision(
                RecoveryAction.REPLAN,
                "stale-local clearing unavailable; retain map and replan",
                skipped_actions=(RecoveryAction.CLEAR_STALE_LOCAL,),
            )
        if attempt == 5:
            if capabilities.rear_clearance_verified:
                return RecoveryDecision(
                    RecoveryAction.BACK_UP,
                    "rear clearance explicitly verified",
                )
            return self._scan(
                135.0,
                skipped_actions=(RecoveryAction.BACK_UP,),
                reason="rear clearance unverified; rotate instead of reversing",
            )
        if attempt == 6:
            if capabilities.alternate_approach_available:
                return RecoveryDecision(
                    RecoveryAction.ALTERNATE_APPROACH,
                    "alternate approach provider is available",
                )
            return RecoveryDecision(
                RecoveryAction.REPLAN,
                "alternate approach unavailable; retain goal and replan",
                skipped_actions=(RecoveryAction.ALTERNATE_APPROACH,),
            )

        return self._scan(-135.0, reason="final bounded perception refresh")

    @staticmethod
    def _scan(
        degrees: float,
        *,
        skipped_actions: tuple[RecoveryAction, ...] = (),
        reason: str = "rotate in place to refresh local perception",
    ) -> RecoveryDecision:
        return RecoveryDecision(
            RecoveryAction.ROTATE_RESCAN,
            reason,
            heading_offset_radians=math.radians(degrees),
            skipped_actions=skipped_actions,
        )
