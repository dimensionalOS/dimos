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

"""Why the robot is not moving, said out loud by whichever stage is stuck.

The motion stack idles legitimately in several places — the planner waits for a
map, the follower waits for a plan, the plan may be a deliberate refusal — and
all of them look identical from outside: a robot standing still. This makes
each stage name the input it is missing, so the question "planner or follower?"
is answered by reading the log instead of by bisecting the graph.

Two rules, both about not drowning the useful line:

* EDGE TRIGGERED. A stage that is fine says so once, when it becomes fine. A
  stage that is blocked says so once when it blocks, then repeats on a slow
  heartbeat, because a message you scrolled past ten minutes ago is not an
  answer to "why is it stuck NOW".
* THE FIRST MISSING INPUT IS THE ANSWER. These are ordered dependencies: with
  no odometry the planner cannot use a map it does have, so reporting both is
  reporting one cause and one symptom.
"""

from __future__ import annotations

import time

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class StallReporter:
    """Edge-triggered "what am I waiting on" logging for one stage."""

    def __init__(self, stage: str, heartbeat_s: float = 3.0) -> None:
        self.stage = stage
        self.heartbeat_s = heartbeat_s
        self._blocked_on: str | None = None
        self._since: float | None = None
        self._last_said = 0.0
        self._healthy: str | None = None

    def check(self, inputs: dict[str, bool]) -> bool:
        """Report the first absent input. True when nothing is missing.

        ``inputs`` is ordered: the first False is the cause and the rest are
        very likely its symptoms.
        """
        missing = next((name for name, present in inputs.items() if not present), None)
        if missing is None:
            self._clear()
            return True
        self._blocked(missing)
        return False

    def ok(self, detail: str) -> None:
        """The stage is doing its job; say so once per transition."""
        self._clear()
        if self._healthy != detail:
            self._healthy = detail
            logger.info("running", stage=self.stage, detail=detail)

    def blocked(self, reason: str) -> None:
        """Blocked for a reason that is not a missing input."""
        self._blocked(reason)

    def _blocked(self, reason: str) -> None:
        now = time.monotonic()
        self._healthy = None
        if reason != self._blocked_on:
            self._blocked_on, self._since, self._last_said = reason, now, now
            logger.warning("waiting", stage=self.stage, on=reason)
            return
        if now - self._last_said >= self.heartbeat_s:
            self._last_said = now
            waited = now - (self._since or now)
            logger.warning("still waiting", stage=self.stage, on=reason, waited_s=round(waited, 1))

    def _clear(self) -> None:
        if self._blocked_on is not None:
            waited = time.monotonic() - (self._since or 0.0)
            logger.info(
                "resuming", stage=self.stage, arrived=self._blocked_on, blocked_s=round(waited, 1)
            )
            self._blocked_on, self._since = None, None
