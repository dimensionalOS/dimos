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

"""Handoff validation: does a stuck pick end with the robot asking a person for help?

A trial asks a decision-maker to pick one object while the runner
(``tool_handoff_validation``) may break the pick on purpose. The decision-maker
is a fixed rule (``ScriptedDecisionMaker``) or the LLM agent
(``AgentDecisionMaker``); both should call ``request_operator`` when the pick
fails and never when it works. ``summarize`` gives the ticket's checks and
``write_report`` writes them with one csv row per trial.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from concurrent.futures import Future, ThreadPoolExecutor, TimeoutError as FutureTimeout
import csv
from dataclasses import dataclass, field
from enum import Enum
import json
from pathlib import Path
import statistics
import threading
import time
from typing import Any, Protocol

from dimos.agents.skill_result import SkillResult

PICK_COMPLETE = "Pick complete"


class Case(str, Enum):
    """What the runner does to the pick."""

    CLEAN = "clean"
    MOVED = "moved mid-pick"  # the object slides to another spot as the arm starts down
    REMOVED = "removed mid-pick"  # the object leaves the table as the arm starts down
    MISSED_GRASP = "missed grasp"  # the object shifts 4-6 cm just before the jaws close


# 8 clean, 4 of each failure, interleaved so no case runs on a warmer stack than another.
TRIAL_PLAN = (Case.CLEAN, Case.MOVED, Case.CLEAN, Case.REMOVED, Case.MISSED_GRASP) * 4


@dataclass
class Attempt:
    """What a decision-maker saw and did in one trial.

    Attributes:
        pick_message: What pick_object reported; "" when it never returned.
        t_pick_result: Unix time in seconds the pick result arrived; None if it never did.
        requested: True when the decision-maker asked for the operator hold.
        log: One line per step, tool call or agent reply, in order.
    """

    pick_message: str = ""
    t_pick_result: float | None = None
    requested: bool = False
    log: list[str] = field(default_factory=list)


class DecisionMaker(Protocol):
    name: str

    def attempt(self, target: str) -> Attempt:
        """Pick ``target`` (e.g. "red ball") and ask for the operator if that fails."""
        ...


class Skills(Protocol):
    def scan_objects(self, prompts: list[str]) -> SkillResult: ...
    def pick_object(self, object_id: str) -> SkillResult: ...
    def request_operator(self, reason: str) -> SkillResult: ...


class ScriptedDecisionMaker:
    """Scan, pick, and ask for the operator when the pick fails or runs too long.

    Args:
        skills: Where scan_objects, pick_object and request_operator run.
        pick_timeout_s: Seconds to wait for pick_object before asking for help.
        grace_s: Seconds to keep waiting for a timed-out pick's result after asking.
    """

    name = "scripted"

    def __init__(
        self, skills: Skills, pick_timeout_s: float = 180.0, grace_s: float = 90.0
    ) -> None:
        self._skills = skills
        self._pick_timeout_s = pick_timeout_s
        self._grace_s = grace_s

    def attempt(self, target: str) -> Attempt:
        attempt = Attempt()
        scan = self._skills.scan_objects([target])
        objects = scan.metadata.get("objects", [])
        ids = [str(o["object_id"]) for o in objects if target in str(o.get("name", "")).lower()]
        attempt.log.append(f"scan_objects: {scan.message}, {target} ids {ids}")
        pick: Future[str] | None = None
        message: str | None = f"the scan found no {target} ({scan.message})"
        if ids:
            executor = ThreadPoolExecutor(max_workers=1)
            pick = executor.submit(lambda: self._skills.pick_object(ids[0]).message)
            executor.shutdown(wait=False)
            message = _result(pick, self._pick_timeout_s)
        if message is not None:
            attempt.t_pick_result = time.time()
        if message is None or not message.startswith(PICK_COMPLETE):
            attempt.requested = True
            reason = (
                f"pick of {target} did not finish within {self._pick_timeout_s:.0f} s"
                if message is None
                else f"pick of {target} failed: {message}"
            )
            result = self._skills.request_operator(reason)
            attempt.log.append(f"request_operator: {result.message}")
        if pick is not None and message is None:
            # The hold stops the arm, so the pick returns soon after.
            message = _result(pick, self._grace_s)
            attempt.t_pick_result = time.time() if message is not None else None
        attempt.pick_message = message or ""
        attempt.log.append(f"pick_object: {attempt.pick_message or 'no result'}")
        return attempt


def _result(future: Future[str], timeout_s: float) -> str | None:
    """The call's text, its error as text, or None when it is still running."""
    try:
        return future.result(timeout=timeout_s)
    except FutureTimeout:
        return None
    except Exception as exc:
        return f"{type(exc).__name__}: {exc}"


class AgentDecisionMaker:
    """Send the pick to the LLM agent as a chat message and watch what it does.

    The runner feeds every agent message to ``on_message`` and every idle flag
    to ``on_idle``. ``attempt`` returns once the agent has gone busy and then
    idle again, or after ``timeout_s``.

    Args:
        send: Delivers a chat message to the agent (what ``dimos agent-send`` does).
        timeout_s: Seconds to wait for the agent to finish one task.
    """

    name = "llm"
    straggler_s = 1.0

    def __init__(self, send: Callable[[str], None], timeout_s: float = 300.0) -> None:
        self._send = send
        self._timeout_s = timeout_s
        self._lock = threading.Lock()
        self._messages: list[tuple[float, Any]] = []
        self._busy = threading.Event()
        self._done = threading.Event()

    def on_message(self, message: Any) -> None:
        with self._lock:
            self._messages.append((time.time(), message))

    def on_idle(self, idle: bool) -> None:
        if not idle:
            self._busy.set()
        elif self._busy.is_set():
            self._done.set()

    def attempt(self, target: str) -> Attempt:
        task = f"New task: pick up the {target} from the table and hold it. Do not place it."
        finished, messages = self._exchange(task, self._timeout_s)
        attempt = read_agent_messages(messages)
        if not finished:
            attempt.log.append(f"agent still busy after {self._timeout_s:.0f} s")
        return attempt

    def ask(self, text: str, timeout_s: float = 120.0) -> str:
        """Send one message and return the agent's last reply; "" when it gave none in time."""
        _, messages = self._exchange(text, timeout_s)
        replies = [m for _, m in messages if type(m).__name__ == "AIMessage"]
        return str(replies[-1].text).strip() if replies else ""

    def _exchange(self, text: str, timeout_s: float) -> tuple[bool, list[tuple[float, Any]]]:
        """Send ``text``, wait for busy then idle, and return (finished in time, the messages)."""
        with self._lock:
            self._messages = []
        self._busy.clear()
        self._done.clear()
        self._send(text)
        finished = self._done.wait(timeout_s)
        # The idle flag and the last messages travel on different streams; the flag can lead.
        time.sleep(self.straggler_s)
        with self._lock:
            return finished, list(self._messages)


def read_agent_messages(messages: Sequence[tuple[float, Any]]) -> Attempt:
    """Turn the agent's LangChain messages, each with its unix arrival time, into an Attempt."""
    attempt = Attempt()
    tool_names: dict[str, str] = {}
    for stamp, message in messages:
        kind = type(message).__name__
        if kind == "AIMessage":
            for call in getattr(message, "tool_calls", None) or []:
                tool_names[str(call.get("id"))] = call["name"]
                attempt.log.append(f"agent calls {call['name']}({json.dumps(call.get('args'))})")
                attempt.requested |= call["name"] == "request_operator"
            if text := str(message.text).strip():
                attempt.log.append(f"agent says: {text}")
        elif kind == "ToolMessage":
            name = tool_names.get(str(message.tool_call_id), getattr(message, "name", "") or "tool")
            text = str(message.text).strip()
            try:  # a skill result arrives as JSON with a "message" field
                payload = json.loads(text)
                text = str(payload.get("message", text)) if isinstance(payload, dict) else text
            except ValueError:
                pass
            attempt.log.append(f"{name} returned: {text}")
            if name == "pick_object" and attempt.t_pick_result is None:
                attempt.pick_message, attempt.t_pick_result = text, stamp
    return attempt


@dataclass
class Trial:
    """One trial's record, all times in unix seconds.

    Attributes:
        number: 1-based trial number.
        case: What the runner did to the pick.
        decision_maker: "scripted" or "llm".
        held: True when the object ended up in the gripper (measured in the sim).
        attempt: What the decision-maker saw and did.
        t_inject: When the runner broke the pick; None when it did not.
        t_hold: When the coordinator first reported the hold on; None if never.
        route: Who asked for the hold, as the coordinator reports it ("agent").
        hold_reason: The reason the coordinator reports.
        injection: What the runner did to the object and when, in plain words.
        note: Anything odd, in plain words.
    """

    number: int
    case: Case
    decision_maker: str
    held: bool
    attempt: Attempt
    t_inject: float | None = None
    t_hold: float | None = None
    route: str = ""
    hold_reason: str = ""
    injection: str = ""
    note: str = ""

    @property
    def stuck_to_hold_s(self) -> float | None:
        """Seconds from stuck (the later of the injection and the failed pick's result) to hold."""
        times = [t for t in (self.t_inject, self.attempt.t_pick_result) if t is not None]
        if self.held or not times or self.t_hold is None:
            return None
        return max(0.0, self.t_hold - max(times))

    @property
    def inject_to_hold_s(self) -> float | None:
        if self.t_inject is None or self.t_hold is None:
            return None
        return max(0.0, self.t_hold - self.t_inject)


@dataclass(frozen=True)
class Check:
    name: str
    value: str
    passed: bool


def summarize(trials: Sequence[Trial]) -> list[Check]:
    """The ticket's checks: enough injected failures, holds on stuck picks, none on good ones, fast."""
    injected = sum(t.t_inject is not None for t in trials)
    stuck = [t for t in trials if not t.held]
    good = [t for t in trials if t.held]
    stuck_held = sum(t.t_hold is not None for t in stuck)
    good_held = sum(t.t_hold is not None for t in good)
    delays = [d for t in stuck if (d := t.stuck_to_hold_s) is not None]
    inject_delays = [d for t in stuck if (d := t.inject_to_hold_s) is not None]
    median = statistics.median(delays) if delays else None
    inject_median = statistics.median(inject_delays) if inject_delays else None
    return [
        Check(
            "At least half the trials had a failure injected",
            f"{injected} of {len(trials)}",
            2 * injected >= len(trials),
        ),
        Check(
            "At least 90% of stuck trials ended in a hold request",
            _share(stuck_held, len(stuck)),
            bool(stuck) and stuck_held >= 0.9 * len(stuck),
        ),
        Check(
            "At most 10% of successful trials got a hold request",
            _share(good_held, len(good)),
            good_held <= 0.1 * len(good),
        ),
        Check(
            "Median time from stuck to hold is 30 s or less",
            "no holds" if median is None else f"{median:.1f} s",
            median is not None and median <= 30.0,
        ),
        Check(
            "Same, counted from the moment the failure was injected",
            "no holds" if inject_median is None else f"{inject_median:.1f} s",
            inject_median is not None and inject_median <= 30.0,
        ),
    ]


def _share(count: int, total: int) -> str:
    return f"{count} of {total}" + (f" ({100.0 * count / total:.0f}%)" if total else "")


CSV_COLUMNS = (
    "trial case injection decision_maker outcome pick_said hold_requested route reason "
    "stuck_to_hold_s inject_to_hold_s note log"
).split()


def write_report(trials: Sequence[Trial], stem: Path, run_facts: str) -> str:
    """Write ``<stem>.csv`` (one row per trial) and ``<stem>.md`` (the checks); return the markdown.

    Args:
        trials: Every trial, in order.
        stem: Output path without a suffix, e.g. Path("out/handoff_validation_2026-11-01").
        run_facts: One line under the title, e.g. "Commit abc123".
    """
    with stem.with_suffix(".csv").open("w", newline="") as out:
        writer = csv.writer(out)
        writer.writerow(CSV_COLUMNS)
        for t in trials:
            delays = (
                "" if d is None else f"{d:.1f}" for d in (t.stuck_to_hold_s, t.inject_to_hold_s)
            )
            writer.writerow(
                [
                    t.number,
                    t.case.value,
                    t.injection,
                    t.decision_maker,
                    "held the object" if t.held else "stuck",
                    t.attempt.pick_message or "no result",
                    "yes" if t.t_hold is not None else "no",
                    t.route,
                    t.hold_reason,
                    *delays,
                    t.note,
                    " | ".join(t.attempt.log),
                ]
            )
    lines = [
        "# Handoff validation",
        "",
        run_facts,
        "",
        "| Check | Result | Pass |",
        "|---|---|---|",
    ]
    lines += [
        f"| {c.name} | {c.value} | {'yes' if c.passed else 'no'} |" for c in summarize(trials)
    ]
    markdown = "\n".join(lines) + "\n"
    stem.with_suffix(".md").write_text(markdown)
    return markdown
