"""Passive auxiliary-call bookkeeping for the execution runtime."""

from dataclasses import dataclass
from typing import Any

from dimos.manipulation.execution_effects import AuxiliaryDone


@dataclass(frozen=True)
class AuxiliaryTicket:
    action_id: str
    setter: bool


class AuxiliaryCallBook:
    """Store auxiliary correlations and results without lifecycle decisions."""

    def __init__(self) -> None:
        self._pending: dict[str, float] = {}
        self._inflight: set[str] = set()
        self._results: dict[str, tuple[bool, Any, str]] = {}

    def register(self, action_id: str, deadline: float, setter: bool) -> AuxiliaryTicket:
        self._inflight.add(action_id)
        self._pending[action_id] = deadline
        return AuxiliaryTicket(action_id, setter)

    def complete(self, done: AuxiliaryDone) -> bool:
        pending = done.action_id in self._pending
        inflight = done.action_id in self._inflight
        self._pending.pop(done.action_id, None)
        self._inflight.discard(done.action_id)
        if not pending and not inflight:
            return False
        if done.action_id in self._results:
            return True
        if done.error is None:
            result = (True, done.value, "")
        else:
            result = (False, None, str(done.error))
        self._results[done.action_id] = result
        if len(self._results) > 16:
            self._results.pop(next(iter(self._results)))
        return True

    def take_result(self, action_id: str) -> tuple[bool, Any, str] | None:
        return self._results.pop(action_id, None)

    def has_pending(self) -> bool:
        return bool(self._pending)

    def has_inflight(self) -> bool:
        return bool(self._inflight)

    def deadlines(self) -> tuple[float, ...]:
        return tuple(self._pending.values())

    def has_unsettled(self) -> bool:
        return bool(self._pending or self._inflight)

    def expire(self, now: float, diagnostic: str) -> bool:
        changed = False
        for action_id, deadline in tuple(self._pending.items()):
            if deadline <= now and action_id not in self._results:
                self._pending.pop(action_id, None)
                self._results[action_id] = (False, None, diagnostic)
                changed = True
                if len(self._results) > 16:
                    self._results.pop(next(iter(self._results)))
        return changed
