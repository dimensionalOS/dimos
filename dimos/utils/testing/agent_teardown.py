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

"""Best-effort teardown for the ``agent_setup`` test fixtures.

When an agent test fails mid-run — e.g. the LLM is slow under heavy parallel
load and ``finished_event.wait`` times out — the fixture still has to release
everything it created: the coordinator's worker processes, the LCM transports,
and their subscriptions. If one cleanup step raises, the remaining steps must
still run; otherwise threads leak and the autouse ``monitor_threads`` fixture
turns a plain test failure into a failure *and* a teardown error. So each step
is isolated and its failure logged rather than propagated.
"""

from collections.abc import Callable, Iterable
from typing import Any

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def teardown_agent_setup(
    coordinator: Any | None,
    transports: Iterable[Any],
    unsubs: Iterable[Callable[[], Any]],
) -> None:
    """Tear down an ``agent_setup`` run, attempting every step regardless of failures."""

    def _safe(action: Callable[[], Any], step: str) -> None:
        try:
            action()
        except Exception:
            # Cleanup is best-effort: log and keep going so a single failing
            # step never leaks the rest or masks the test's own result.
            logger.error("agent_setup teardown step failed", step=step, exc_info=True)

    if coordinator is not None:
        _safe(coordinator.stop, "coordinator.stop")

    for transport in transports:
        _safe(transport.stop, f"{type(transport).__name__}.stop")

    for i, unsub in enumerate(unsubs):
        _safe(unsub, f"unsubscribe[{i}]")
