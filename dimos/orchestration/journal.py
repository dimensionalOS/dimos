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

"""What the engine did, step by step, as plain JSON for tools that watch an orchestrator."""

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Any

from dimos.orchestration.definition import OrchestratorDefinition
from dimos.orchestration.engine import (
    AfterTimer,
    Call,
    CancelDeadline,
    ClearTimer,
    Effect,
    FaultReport,
    QueueItem,
    ReturnItem,
    Runtime,
    SendCancel,
    SendStart,
    SetTimer,
    StreamItem,
    Timer,
    TimerItem,
    TransitionRecord,
)

Json = dict[str, Any]

_MISSING = object()


@dataclass(frozen=True)
class EditItem(StreamItem):
    """A blackboard value written through the `set_value` RPC rather than read from a stream."""

    @staticmethod
    def of(key: str, value: Any) -> "EditItem":
        return EditItem(f"set_value.{key}", {key: value})


def definition_to_plain(definition: OrchestratorDefinition) -> Json:
    """The definition with keys as the file writes them (`if`, not `if_`)."""
    plain: Json = definition.model_dump(mode="json")
    for node in plain["nodes"]:
        for transition in node["transitions"]:
            transition["if"] = transition.pop("if_")
    return plain


def _call(call: Call) -> Json:
    return {
        "id": call.id,
        "node": call.node,
        "kind": call.kind,
        "phase": call.phase,
        "rpc": call.rpc,
        "args": dict(call.args),
        "uses": sorted(call.uses),
    }


def _timer(timer: Timer, wall_offset: float) -> Json:
    """A timer, with its due time moved from the monotonic clock to the wall clock."""
    match timer:
        case AfterTimer(node, due, transition):
            return {
                "kind": "after",
                "node": node,
                "due": due + wall_offset,
                "transition": transition,
            }
        case CancelDeadline(node, due, call_id):
            return {
                "kind": "cancel_deadline",
                "node": node,
                "due": due + wall_offset,
                "call": call_id,
            }


def _item(item: QueueItem | None) -> Json:
    match item:
        case None:
            return {"kind": "startup"}
        case EditItem(_, values):
            ((key, value),) = values.items()
            return {"kind": "edit", "key": key, "value": value}
        case StreamItem(stream, values):
            return {"kind": "stream", "stream": stream, "values": dict(values)}
        case ReturnItem(call, value, raised):
            return {"kind": "return", "call": _call(call), "value": value, "raised": raised}
        case TimerItem(timer):
            return {"kind": "timer", "node": timer.node}


def _effect(effect: Effect, wall_offset: float) -> Json:
    match effect:
        case SendStart(call):
            return {"kind": "start", "call": _call(call)}
        case SendCancel(call):
            return {"kind": "cancel", "call": _call(call)}
        case SetTimer(timer):
            return {"kind": "set_timer", "timer": _timer(timer, wall_offset)}
        case ClearTimer(timer):
            return {"kind": "clear_timer", "timer": _timer(timer, wall_offset)}
        case TransitionRecord(node, cause, goto):
            return {"kind": "transition", "node": node, "cause": cause, "goto": goto}
        case FaultReport(rule, node, event, _):
            return {"kind": "fault", "rule": rule, "node": node, "event": event}


class Journal:
    """The runtime after each engine step and what led to it, the newest `size` steps kept."""

    def __init__(self, size: int) -> None:
        self._lock = threading.Lock()
        self._entries: deque[Json] = deque(maxlen=size)
        self._seq = 0
        self._runtime: Runtime | None = None
        self._faulted = False
        # Added to a monotonic time to give the wall clock time it stands for.
        self._wall_offset = time.time() - time.monotonic()

    def record(self, item: QueueItem | None, runtime: Runtime, effects: list[Effect]) -> None:
        """Add one step: the item handled (None for startup), the runtime after it, its effects."""
        with self._lock:
            before = {} if self._runtime is None else self._runtime.blackboard
            after = runtime.blackboard
            changed = {
                key: value for key, value in after.items() if before.get(key, _MISSING) != value
            }
            self._seq += 1
            self._faulted = self._faulted or any(isinstance(e, FaultReport) for e in effects)
            self._entries.append(
                {
                    "seq": self._seq,
                    "time": time.time(),
                    "item": _item(item),
                    "effects": [_effect(effect, self._wall_offset) for effect in effects],
                    "changed": changed,
                    "removed": sorted(before.keys() - after.keys()),
                    **self._runtime_plain(runtime),
                }
            )
            self._runtime = runtime

    def snapshot(self) -> Json:
        """The runtime now, with the sequence number of the step that made it."""
        with self._lock:
            if self._runtime is None:
                return {"seq": self._seq, "started": False, "faulted": self._faulted}
            return {
                "seq": self._seq,
                "started": True,
                "faulted": self._faulted,
                "time": time.time(),
                "blackboard": dict(self._runtime.blackboard),
                **self._runtime_plain(self._runtime),
            }

    def since(self, after: int, limit: int) -> Json:
        """Up to `limit` steps after sequence number `after`, and the oldest one still kept."""
        with self._lock:
            entries = [entry for entry in self._entries if entry["seq"] > after][:limit]
            oldest = self._entries[0]["seq"] if self._entries else self._seq + 1
            return {"seq": self._seq, "oldest": oldest, "entries": entries}

    def _runtime_plain(self, runtime: Runtime) -> Json:
        return {
            "active": sorted(runtime.active),
            "calls": [_call(call) for call in runtime.calls],
            "timers": [_timer(timer, self._wall_offset) for timer in runtime.timers],
        }
