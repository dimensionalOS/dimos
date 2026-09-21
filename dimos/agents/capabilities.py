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

"""Capability registry for skill-level mutual exclusion.

A skill declares the capabilities it occupies via `@skill(uses=[...])`. The MCP
server consults a process-wide `CapabilityRegistry` before dispatching each
`tools/call`. When a required capability is held by another skill, the server
either waits briefly for it to clear (for short, self-completing holders) or
refuses with a plain text "Cannot start X: capability Y is held by Z" result, in
which case the LLM decides what to do (typically: call the appropriate stop tool,
then retry).

Capabilities are plain strings. Today the only declared capability is
`CAP_MOVEMENT`. New capabilities should be added as constants here so they are
discoverable from one place.
"""

from __future__ import annotations

from collections.abc import Callable
import threading
import time
from typing import NamedTuple

CAP_MOVEMENT = "movement"


class _Hold(NamedTuple):
    """Who currently holds a capability.

    `tool_name` is the human-meaningful skill name (used for conflict messages
    and the same-tool takeover decision). `token` is unique per invocation and
    scopes release, so a stale invocation can't free a live hold.
    """

    tool_name: str
    token: str


class CapabilityRegistry:
    """In-memory map of capability -> holding invocation.

    All methods are thread-safe. The registry is intentionally simple: every
    capability is exclusive (no shared/exclusive distinction yet). On conflict
    `acquire` either refuses immediately (the default try-lock) or, when given a
    `timeout`, blocks until the conflicting hold clears or the timeout expires.

    A hold is identified per-invocation by an opaque `token`, not by the tool
    name. Two invocations of the *same* tool don't conflict -- the later one
    takes over the hold -- but release is scoped to the token, so the earlier
    invocation's teardown can't release the capability out from under the later
    one. Different tools sharing a capability still conflict.
    """

    def __init__(self) -> None:
        self._holders: dict[str, _Hold] = {}
        self._cond = threading.Condition()

    def acquire(
        self,
        caps: list[str],
        tool_name: str,
        token: str,
        *,
        timeout: float = 0.0,
        can_wait: Callable[[str], bool] | None = None,
    ) -> tuple[str, str] | None:
        """Atomic all-or-nothing acquire of `caps` for one invocation.

        Conflicts only with a *different* `tool_name`; a same-tool re-acquire is
        a takeover that overwrites the holder with the new `token`.

        With the default `timeout=0.0` this is a non-blocking try-lock: it returns
        `None` on success or, on conflict, `(cap, current_tool)` for the first
        conflicting capability (no caps are acquired in that case).

        With `timeout > 0` it blocks up to `timeout` seconds for a conflicting hold
        to clear, re-attempting the all-or-nothing acquire whenever a hold is
        released. `can_wait(holder_tool_name)` decides whether to wait on a given
        holder; if it returns `False` the conflict is returned immediately instead
        of waiting (used to refuse, rather than block on, holders that won't release
        on their own).
        """
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                conflict = self._acquire_locked(caps, tool_name, token)
                if conflict is None:
                    return None
                if can_wait is not None and not can_wait(conflict[1]):
                    return conflict
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return conflict
                self._cond.wait(remaining)

    def _acquire_locked(
        self, caps: list[str], tool_name: str, token: str
    ) -> tuple[str, str] | None:
        """Atomic check-and-set of `caps`; the caller must hold `self._cond`."""
        for cap in caps:
            current = self._holders.get(cap)
            if current is not None and current.tool_name != tool_name:
                return (cap, current.tool_name)
        for cap in caps:
            self._holders[cap] = _Hold(tool_name, token)
        return None

    def release_by_token(self, token: str) -> list[str]:
        """Release every capability whose current holder matches `token`.

        A stale token (one already taken over by a newer invocation of the same
        tool) matches nothing and releases nothing. Wakes any callers blocked in
        `acquire` so they can re-attempt.
        """
        with self._cond:
            released = [cap for cap, h in self._holders.items() if h.token == token]
            for cap in released:
                del self._holders[cap]
            if released:
                self._cond.notify_all()
        return released

    def snapshot(self) -> dict[str, str]:
        """Map of capability -> holding tool name (for logging/diagnostics)."""
        with self._cond:
            return {cap: h.tool_name for cap, h in self._holders.items()}
