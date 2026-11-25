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

"""``cal.run()`` in ``dimos shell`` against a scripted module: what it prints and answers."""

from __future__ import annotations

from collections.abc import Iterator
from typing import Any

import pytest

from dimos.control.calibration import shell
from dimos.control.calibration.module import CalibrationStatus
from dimos.control.calibration.shell import CalibrationShell


def frame(
    state: str, log: list[str], question: str | None = None, **fields: Any
) -> CalibrationStatus:
    answers = ["go", "skip"] if question else []
    return CalibrationStatus.model_validate(
        {
            "state": state,
            "arm": "toy",
            "steps": [],
            "log": log,
            "logged": len(log) + 10,
            "question": question,
            "answers": answers,
            **fields,
        }
    )


EARLIER = ["the last run's summary"]
STARTED = [*EARLIER, "Calibrating toy: check, gravity_poses"]
FIRST = [*STARTED, "? Move to pose_01?"]
SECOND = [*FIRST, "> go", "? Move to pose_02?"]
DONE = [*SECOND, "> skip", "Calibration done:\n  gravity_poses: 1 of 2 poses"]
WHERE = {"step": "gravity_poses", "progress": "step 2/2 · pose 1/2 · 00:03"}


class ScriptedModule:
    """Each status in turn; a question's status stays until it is answered."""

    def __init__(self, frames: list[CalibrationStatus], reply: str = "started") -> None:
        self.frames, self.reply, self.aborted = frames, reply, False
        self.answers: list[str] = []

    def run(self, steps: list[str] | None) -> str:
        return self.reply

    def status(self) -> CalibrationStatus:
        status = self.frames[0]
        if status.question is None and len(self.frames) > 1:
            self.frames.pop(0)
        return status

    def answer(self, text: str) -> str:
        self.answers.append(text)
        if text.strip().lower() not in self.frames[0].answers:
            return f"unknown answer {text!r}; answer go or skip"
        self.frames.pop(0)
        return "ok"

    def abort(self) -> str:
        self.aborted = True
        return "aborting"


@pytest.fixture(autouse=True)
def no_wait(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(shell, "POLL_S", 0.0)


def typing(monkeypatch: pytest.MonkeyPatch, *lines: str | type[BaseException]) -> None:
    typed: Iterator[str | type[BaseException]] = iter(lines)

    def read(prompt: str) -> str:
        line = next(typed)
        if not isinstance(line, str):
            raise line
        return line

    monkeypatch.setattr("builtins.input", read)


def test_run_prints_each_new_message_and_answers_each_question_enter_first(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    module = ScriptedModule(
        [
            frame("done", EARLIER),
            frame("running", STARTED),
            frame("waiting", FIRST, "Move to pose_01?", **WHERE),
            frame("waiting", SECOND, "Move to pose_02?", **WHERE),
            frame("done", DONE),
        ]
    )
    typing(monkeypatch, "", "dance", "Skip")

    assert CalibrationShell(module).run() is None

    assert module.answers == ["go", "dance", "Skip"]
    assert capsys.readouterr().out.splitlines() == [
        "started",
        "Calibrating toy: check, gravity_poses",
        "toy: waiting at gravity_poses · step 2/2 · pose 1/2 · 00:03",
        "? Move to pose_01?",
        "  answer: go | skip",
        "toy: waiting at gravity_poses · step 2/2 · pose 1/2 · 00:03",
        "? Move to pose_02?",
        "  answer: go | skip",
        "unknown answer 'dance'; answer go or skip",
        "Calibration done:",
        "  gravity_poses: 1 of 2 poses",
    ]


def test_ctrl_c_leaves_the_run_going_and_run_follows_it_again(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    asking = frame("waiting", FIRST, "Move to pose_01?", **WHERE)
    module = ScriptedModule([frame("running", STARTED), asking, frame("done", DONE)])
    cal = CalibrationShell(module)
    typing(monkeypatch, KeyboardInterrupt, "")

    cal.run()
    assert module.answers == [] and not module.aborted
    assert capsys.readouterr().out.endswith(shell.LEFT + "\n")

    module.reply = "already running"
    cal.run()
    assert module.answers == ["go"]
    assert capsys.readouterr().out.splitlines()[:3] == [
        "already running",
        "toy: waiting at gravity_poses · step 2/2 · pose 1/2 · 00:03",
        "? Move to pose_01?",
    ]


def test_an_answer_to_a_question_that_changed_meanwhile_is_not_sent(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    module = ScriptedModule(
        [
            frame("running", STARTED),
            frame("waiting", FIRST, "Move to pose_01?"),
            frame("done", DONE),
        ]
    )

    def elsewhere(prompt: str) -> str:
        module.frames.pop(0)
        return ""

    monkeypatch.setattr("builtins.input", elsewhere)
    CalibrationShell(module).run()

    assert module.answers == []
    assert "The question changed before this answer; not sent." in capsys.readouterr().out


def test_status_answer_and_abort_are_the_rpcs() -> None:
    asking = frame("waiting", FIRST, "Move to pose_01?")
    module = ScriptedModule([asking, frame("done", DONE)])
    cal = CalibrationShell(module)

    assert cal.status() is asking and cal.arm == "toy"
    assert cal.answer("go") == "ok" and cal.abort() == "aborting" and module.aborted
