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

"""Belief queries an agent can call.

The default agentic blueprint ships no way to consult the past, so an agent can
look at the current frame, move and talk, and nothing else. This is the read half
of that gap.

A failure carries an :class:`~dimos.experimental.memory_belief.answer.UnknownReason` and the
capability that would fix it, so an agent that gets "unknown" back can act rather
than parse prose. Read-only by construction: the skill declares no capabilities.
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Any

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.experimental.memory_belief.answer import UnknownReason
from dimos.experimental.memory_belief.api import QueryError, execute
from dimos.experimental.memory_belief.remedy import RemedyResolver
from dimos.memory.module import MemoryModule, MemoryModuleConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Mapping


logger = setup_logger()


class BeliefQueryConfig(MemoryModuleConfig):
    """Thresholds a caller may need to tune per deployment."""

    belief_stream_name: str = "belief_observation"
    #: Recording start, so ``as_of`` can default to a time base the data shares.
    #: Left None on a live robot, where wall clock and stream clock agree.
    time_origin_ts: float | None = None


class BeliefQuerySkills(MemoryModule):
    """Read-only access to what the robot believes.

    A :class:`~dimos.memory.module.MemoryModule` so a blueprint can point it at
    the same ``db_path`` a recorder is writing, rather than at a second copy of
    the belief.

    **Absence here means "no record", not "looked and saw nothing".** A query
    answers NEVER_COVERED when nothing matched, which is honest but weak: no
    ray-marched visibility backs it. A coverage grid that did give the stronger
    answer was written and removed -- nothing joined it to a query, so it stated
    a capability the layer could not actually reach.
    """

    config: BeliefQueryConfig

    def __init__(
        self,
        store: Any = None,
        remedies: Mapping[str, Any] | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        if store is not None:  # tests and scripts hand one in directly
            self._store = store
        # No providers registered here: which skill supplies `sweep_place` is a
        # deployment fact, not a property of the query. An empty resolver still
        # reports a capability as unprovided, which is the honest answer on a
        # robot that cannot act on it.
        self._remedies = RemedyResolver(remedies)

    @rpc
    def start(self) -> None:
        super().start()

    def _now(self) -> float:
        """The asking time, in the same base as the records.

        Wall clock is only correct when the stream is live. Against a replayed
        recording it produced staleness values of several days, which is how a
        fresh observation came back marked stale in every recorded trace.
        """
        if self.config.time_origin_ts is not None:
            return self.config.time_origin_ts
        if self.config.belief_stream_name not in self.store.streams:
            return time.time()
        last: list[Any] = (
            self.store.stream(self.config.belief_stream_name)
            .order_by("ts", desc=True)
            .limit(1)
            .to_list()
        )
        return last[0].ts if last else time.time()

    @skill
    def query(self, query: dict[str, Any]) -> SkillResult:
        """Ask the belief store one composed question.

        Filters compose inside one call, so ask the whole question at once
        rather than splitting it across several calls.

        Args:
            query: ``{"select": ..., "as_of": ..., "where": [...], "project": ...}``.
                ``select``: ``entities`` -- the only one. A thing, not a
                sighting of one.
                ``where``: a list of clauses, ANDed. Two operators:
                ``{"op": "label", "value": "chair"}`` and
                ``{"op": "time_range", "t1": 0, "t2": 200}``.
                ``project``: ``locate`` -- the only one. Position, label,
                support and dispersion per thing.
                ``as_of``: defaults to the latest record.
                ``limit``: how many rows to read, 50 by default.
        """
        payload = dict(query or {})
        # Clamped, not defaulted. Asking about an earlier moment is a real
        # question -- "where was it a minute ago" -- but a model that picks a
        # later one answers from observations that have not happened, and the
        # answer reads exactly as well as a correct one.
        now = self._now()
        try:
            asked = float(payload.get("as_of", now))
        except (TypeError, ValueError):
            asked = now
        payload["as_of"] = min(asked, now) if math.isfinite(asked) else now
        try:
            envelope = execute(self.store, payload)
        except QueryError as exc:
            return SkillResult(success=False, error_code="INVALID_ARGUMENT", message=str(exc))
        if envelope["status"] == "ok":
            return SkillResult(
                success=True,
                message=str(envelope["result"]),
                metadata={k: envelope[k] for k in ("result", "quality", "time_base")},
            )
        # An unknown is not a failure to be retried blindly: the diagnostic says
        # which clause emptied the result, so the next attempt can be informed.
        #
        # ``unknown_reason`` is set on the result rather than left in metadata
        # because that is the field the agent framework reads to derive
        # ``retryable`` and the suggested capability. Reporting the reason only
        # inside ``metadata`` makes an honest "I have not looked there" arrive
        # as an unadorned failure, and a model that cannot see the difference
        # answers through it -- which is the one behaviour this layer exists to
        # prevent.
        reason = envelope.get("reason")
        unknown = UnknownReason[reason] if reason else None
        # The remedy loop's whole point is that an agent can act on a refusal.
        # Resolving it here, against the providers this deployment registered,
        # is what separates "go and look, something can" from "nobody here can
        # fix this" -- and those lead to opposite next moves.
        remedy = self._remedies.plan_for(unknown)
        return SkillResult(
            success=False,
            error_code="INVALID_STATE",
            message=f"{envelope['status']}: {reason}",
            unknown_reason=unknown,
            metadata={
                **{k: envelope.get(k) for k in ("reason", "quality", "time_base", "diagnostic")},
                "remedy": None
                if remedy is None
                else {
                    "capability": remedy.capability,
                    "provider": remedy.provider,
                    "actionable": remedy.actionable,
                    "needs_revisit": remedy.needs_revisit,
                    "note": remedy.note,
                },
            },
        )
