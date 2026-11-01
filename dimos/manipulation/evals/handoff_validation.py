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

A trial asks a decision-maker to pick one object while the trial runner may
break the pick on purpose. The decision-maker is either a fixed rule
(``ScriptedDecisionMaker``) or the LLM agent (``AgentDecisionMaker``); both
should call ``request_operator`` when the pick fails and never when it works.
``summarize`` turns the trials into the four numbers the handoff ticket asks
for, and ``render_markdown`` / ``render_csv`` write the report.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from concurrent.futures import Future, ThreadPoolExecutor, TimeoutError as FutureTimeout
import csv
from dataclasses import dataclass, field
from enum import Enum
import io
import json
import statistics
import threading
import time
from typing import Any, Protocol

from dimos.agents.skill_result import SkillResult

PICK_COMPLETE = "Pick complete"


class Case(str, Enum):
    """What the trial runner does to the pick."""

    CLEAN = "clean"
    MOVED = "moved mid-pick"  # the object slides to another spot as the arm starts down
    REMOVED = "removed mid-pick"  # the object leaves the table as the arm starts down
    MISSED_GRASP = "missed grasp"  # the object shifts 4-6 cm just before the jaws close


# 8 clean, 4 of each failure, interleaved so no case runs on a warmer stack than another.
TRIAL_PLAN: tuple[Case, ...] = (
    Case.CLEAN,
    Case.MOVED,
    Case.CLEAN,
    Case.REMOVED,
    Case.MISSED_GRASP,
) * 4


class PickSkills(Protocol):
    def scan_objects(self, prompts: list[str]) -> SkillResult: ...
    def pick_object(self, object_id: str) -> SkillResult: ...


class OperatorSkills(Protocol):
    def request_operator(self, reason: str) -> SkillResult: ...


@dataclass
class Attempt:
    """What a decision-maker saw and did in one trial.

    Attributes:
        pick_message: What pick_object reported; "" when it never returned.
        t_pick_result: Unix time in seconds the pick result arrived; None if it never did.
        requested: True when the decision-maker asked for the operator hold.
        reason: The reason it gave; "" when it did not ask.
        log: One line per step, tool call or agent reply, in order.
    """

    pick_message: str = ""
    t_pick_result: float | None = None
    requested: bool = False
    reason: str = ""
    log: list[str] = field(default_factory=list)


class DecisionMaker(Protocol):
    name: str

    def attempt(self, target: str) -> Attempt:
        """Pick ``target`` (e.g. "apple") and ask for the operator if that fails."""
        ...


def hold_reason(target: str, pick_message: str | None, timeout_s: float) -> str | None:
    """The reason to ask a person for help after a pick, or None when the pick worked.

    Args:
        target: The object being picked, e.g. "apple".
        pick_message: What pick_object reported; None when it did not return in time.
        timeout_s: How long the pick was given, seconds, for the reason text.
    """
    if pick_message is None:
        return f"pick of {target} did not finish within {timeout_s:.0f} s"
    if pick_message.startswith(PICK_COMPLETE):
        return None
    return f"pick of {target} failed: {pick_message}"


class ScriptedDecisionMaker:
    """Scan, pick, and ask for the operator when the pick fails or runs too long.

    Args:
        skills: Where scan_objects and pick_object run.
        operator: Where request_operator runs.
        pick_timeout_s: Seconds to wait for pick_object before asking for help.
        grace_s: Seconds to keep waiting for a timed-out pick's result after asking.
    """

    name = "scripted"

    def __init__(
        self,
        skills: PickSkills,
        operator: OperatorSkills,
        pick_timeout_s: float = 180.0,
        grace_s: float = 90.0,
    ) -> None:
        self._skills = skills
        self._operator = operator
        self._pick_timeout_s = pick_timeout_s
        self._grace_s = grace_s

    def attempt(self, target: str) -> Attempt:
        attempt = Attempt()
        scan = self._skills.scan_objects([target])
        ids = [
            str(obj["object_id"])
            for obj in scan.metadata.get("objects", [])
            if target in str(obj.get("name", "")).lower()
        ]
        attempt.log.append(f"scan_objects: {scan.message}, {target} ids {ids}")
        pick: Future[str] | None = None
        if not ids:
            message: str | None = f"the scan found no {target} ({scan.message})"
        else:
            pick = _start(lambda: self._skills.pick_object(ids[0]).message)
            message = _result(pick, self._pick_timeout_s)
        if message is not None:
            attempt.t_pick_result = time.time()
        reason = hold_reason(target, message, self._pick_timeout_s)
        if reason is not None:
            attempt.requested = True
            attempt.reason = reason
            result = self._operator.request_operator(reason)
            attempt.log.append(f"request_operator: {result.message}")
        if pick is not None and message is None:
            # The hold stops the arm, so the pick returns soon after.
            message = _result(pick, self._grace_s)
            attempt.t_pick_result = time.time() if message is not None else None
        attempt.pick_message = message or ""
        attempt.log.append(f"pick_object: {attempt.pick_message or 'no result'}")
        return attempt


def _start(call: Callable[[], str]) -> Future[str]:
    executor = ThreadPoolExecutor(max_workers=1)
    future = executor.submit(call)
    executor.shutdown(wait=False)
    return future


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

    The trial runner feeds every agent message to ``on_message`` and every
    idle flag to ``on_idle``. ``attempt`` returns once the agent has gone
    busy and then idle again, or after ``timeout_s``.

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
        finished = self._exchange(task_message(target), self._timeout_s)
        with self._lock:
            attempt = read_agent_messages(self._messages)
        if not finished:
            attempt.log.append(f"agent still busy after {self._timeout_s:.0f} s")
        return attempt

    def ask(self, text: str, timeout_s: float = 120.0) -> str:
        """Send one message and return the agent's last reply; "" when it gave none in time."""
        self._exchange(text, timeout_s)
        with self._lock:
            replies = [m for _, m in self._messages if type(m).__name__ == "AIMessage"]
        return _text(replies[-1].content) if replies else ""

    def _exchange(self, text: str, timeout_s: float) -> bool:
        """Send ``text`` and wait for the agent to go busy and then idle; False on timeout."""
        with self._lock:
            self._messages = []
        self._busy.clear()
        self._done.clear()
        self._send(text)
        finished = self._done.wait(timeout_s)
        # The idle flag and the agent's last messages travel on different
        # streams, so the flag can arrive first.
        time.sleep(self.straggler_s)
        return finished


def task_message(target: str) -> str:
    return f"New task: pick up the {target} from the table and hold it. Do not place it."


def read_agent_messages(messages: Sequence[tuple[float, Any]]) -> Attempt:
    """Turn the agent's messages (LangChain AI and tool messages, with arrival times) into an Attempt."""
    attempt = Attempt()
    tool_names: dict[str, str] = {}
    for stamp, message in messages:
        kind = type(message).__name__
        if kind == "AIMessage":
            for call in getattr(message, "tool_calls", None) or []:
                tool_names[str(call.get("id"))] = call["name"]
                attempt.log.append(f"agent calls {call['name']}({json.dumps(call.get('args'))})")
                if call["name"] == "request_operator":
                    attempt.requested = True
                    attempt.reason = str((call.get("args") or {}).get("reason", ""))
            if text := _text(message.content):
                attempt.log.append(f"agent says: {text}")
        elif kind == "ToolMessage":
            name = tool_names.get(str(message.tool_call_id), getattr(message, "name", "") or "tool")
            text = _tool_text(message.content)
            attempt.log.append(f"{name} returned: {text}")
            if name == "pick_object" and attempt.t_pick_result is None:
                attempt.pick_message = text
                attempt.t_pick_result = stamp
    return attempt


def _text(content: Any) -> str:
    if isinstance(content, str):
        return content.strip()
    if isinstance(content, list):
        return " ".join(
            str(part.get("text", "")) if isinstance(part, dict) else str(part) for part in content
        ).strip()
    return str(content)


def _tool_text(content: Any) -> str:
    """A skill result's message from its JSON text, or the text itself."""
    text = _text(content)
    try:
        payload = json.loads(text)
    except ValueError:
        return text
    return str(payload.get("message", text)) if isinstance(payload, dict) else text


@dataclass
class Trial:
    """One trial's record, all times in unix seconds.

    Attributes:
        number: 1-based trial number.
        case: What the runner did to the pick.
        decision_maker: "scripted" or "llm".
        held: True when the object ended up in the gripper (measured in the sim).
        attempt: What the decision-maker saw and did.
        t_start: When the trial started.
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
    t_start: float
    t_inject: float | None = None
    t_hold: float | None = None
    route: str = ""
    hold_reason: str = ""
    injection: str = ""
    note: str = ""

    @property
    def stuck(self) -> bool:
        return not self.held

    @property
    def stuck_at(self) -> float | None:
        """When the trial was stuck: the later of the injection and the failed pick's result."""
        if not self.stuck:
            return None
        times = [t for t in (self.t_inject, self.attempt.t_pick_result) if t is not None]
        return max(times) if times else None

    @property
    def stuck_to_hold_s(self) -> float | None:
        stuck_at = self.stuck_at
        if stuck_at is None or self.t_hold is None:
            return None
        return max(0.0, self.t_hold - stuck_at)

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
    stuck = [t for t in trials if t.stuck]
    good = [t for t in trials if not t.stuck]
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


def _seconds(value: float | None) -> str:
    return "" if value is None else f"{value:.1f}"


CSV_COLUMNS = (
    "trial",
    "case",
    "injection",
    "decision_maker",
    "outcome",
    "pick_said",
    "hold_requested",
    "route",
    "reason",
    "stuck_to_hold_s",
    "inject_to_hold_s",
    "note",
)


def _row(trial: Trial) -> dict[str, str]:
    return {
        "trial": str(trial.number),
        "case": trial.case.value,
        "injection": trial.injection,
        "decision_maker": trial.decision_maker,
        "outcome": "stuck" if trial.stuck else "held the object",
        "pick_said": trial.attempt.pick_message or "no result",
        "hold_requested": "yes" if trial.t_hold is not None else "no",
        "route": trial.route,
        "reason": trial.hold_reason,
        "stuck_to_hold_s": _seconds(trial.stuck_to_hold_s),
        "inject_to_hold_s": _seconds(trial.inject_to_hold_s),
        "note": trial.note,
    }


def render_csv(trials: Sequence[Trial]) -> str:
    out = io.StringIO()
    writer = csv.DictWriter(out, fieldnames=CSV_COLUMNS)
    writer.writeheader()
    for trial in trials:
        writer.writerow(_row(trial))
    return out.getvalue()


def render_markdown(trials: Sequence[Trial], header: dict[str, str]) -> str:
    """The report: run facts, the ticket's checks, one row per trial, and the odd ones' logs.

    Args:
        trials: Every trial, in order.
        header: Run facts shown first, e.g. {"Commit": "abc123", "Blueprint": "hybrid-teleop-sim"}.
    """
    lines = ["# Handoff validation", ""]
    lines += [f"- {key}: {value}" for key, value in header.items()]
    lines += ["", "## Ticket checks", "", "| Check | Result | Pass |", "|---|---|---|"]
    lines += [
        f"| {c.name} | {c.value} | {'yes' if c.passed else 'no'} |" for c in summarize(trials)
    ]
    lines += [
        "",
        "Stuck means the object was not in the gripper when the pick ended (read from the "
        "sim). Time to hold counts from the later of the injection and the failed pick's "
        "report to the moment the coordinator showed the hold on.",
        "",
        "## Trials",
        "",
        "| # | Case | Outcome | Pick said | Hold | Route | Reason | Stuck to hold (s) "
        "| Injection to hold (s) |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for trial in trials:
        row = _row(trial)
        cells = [
            row["trial"],
            row["case"],
            row["outcome"],
            _cell(row["pick_said"]),
            row["hold_requested"],
            row["route"],
            _cell(row["reason"]),
            row["stuck_to_hold_s"],
            row["inject_to_hold_s"],
        ]
        lines.append("| " + " | ".join(cells) + " |")
    injected = [t for t in trials if t.injection]
    if injected:
        lines += ["", "## What the runner did to the object", ""]
        lines += [f"- Trial {t.number}: {t.injection}" for t in injected]
    odd = [t for t in trials if t.note or (t.stuck and t.t_hold is None)]
    if odd:
        lines += ["", "## Trials that behaved oddly", ""]
        for trial in odd:
            lines += [f"### Trial {trial.number} ({trial.case.value})", "", trial.note or "", ""]
            lines += ["```", *trial.attempt.log, "```", ""]
    agent_logs = [t for t in trials if t.decision_maker == "llm" and t.stuck]
    if agent_logs:
        lines += ["", "## What the agent did on stuck trials", ""]
        for trial in agent_logs:
            lines += [f"### Trial {trial.number} ({trial.case.value})", "", "```"]
            lines += [*trial.attempt.log, "```", ""]
    return "\n".join(lines) + "\n"


def _cell(text: str, limit: int = 160) -> str:
    text = " ".join(text.split()).replace("|", "/")
    return text if len(text) <= limit else text[: limit - 3] + "..."
