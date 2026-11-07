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

from pathlib import Path
import stat

import pytest

from dimos.autoresearch.session import Budget, Session, Split, TrialRecord

PROGRAM = "import math\n\ndef run(robot, task):\n    robot.arm.home()\n"


@pytest.fixture
def session(tmp_path: Path) -> Session:
    budget = Budget(
        iterations=3, candidates_per_round=2, dev_trials=2, heldout_trials=4, threshold=0.8
    )
    return Session.create(tmp_path / "lift", task="lift the cube", budget=budget, model="test")


def record(split: Split, index: int, success: bool) -> TrialRecord:
    return TrialRecord(
        split=split,
        index=index,
        success=success,
        stage="done" if success else "reach",
        note="",
        status="completed",
        duration_s=1.0,
        trace=Path("/nowhere/trace.jsonl"),
        episode=None,
    )


def test_candidates_are_immutable_scored_and_frozen_once(session: Session) -> None:
    first = session.submit(PROGRAM, "go straight down")
    second = session.submit(PROGRAM.replace("home()", "home()\n    robot.gripper.open()"), "open")
    assert (first, second) == (1, 2)
    assert session.program_path(first).read_text() == PROGRAM
    assert stat.S_IMODE(session.program_path(first).stat().st_mode) == 0o444
    assert session.hypothesis(second) == "open"

    session.record_trial(first, record("dev", 0, True))
    session.record_trial(first, record("dev", 1, False))
    session.record_trial(second, record("dev", 0, True))
    session.trial_dir(first, "dev", 0)
    with pytest.raises(FileExistsError):
        session.trial_dir(first, "dev", 0)

    other = Session(session.dir)
    board = [(score.candidate, score.dev_passed, score.dev_trials) for score in other.leaderboard()]
    assert board == [(2, 1, 1), (1, 1, 2)]
    assert other.results(first) == [record("dev", 0, True), record("dev", 1, False)]

    other.freeze(second)
    assert session.frozen() == 2
    assert session.info().status == "frozen"
    with pytest.raises(RuntimeError, match="frozen"):
        session.submit(PROGRAM, "too late")
    with pytest.raises(RuntimeError, match="frozen"):
        session.freeze(first)


def test_submit_refuses_programs_outside_the_contract(session: Session) -> None:
    with pytest.raises(ValueError, match="only math"):
        session.submit("import os\n\ndef run(robot, task):\n    pass\n", "uses os")
    with pytest.raises(ValueError, match="only math"):
        session.submit("from numpy import array\n\ndef run(robot, task):\n    pass\n", "numpy")
    with pytest.raises(ValueError, match=r"run\(robot, task\)"):
        session.submit("import math\n\ndef main():\n    pass\n", "no run")
    with pytest.raises(ValueError, match="parse"):
        session.submit("def run(robot, task)\n    pass\n", "syntax error")
    assert session.candidates() == []


def test_library_notes_are_markdown_files(session: Session) -> None:
    path = session.note("Grasp from above", "Approach 10 cm over the cube, then descend.")
    assert path == session.dir / "library" / "grasp-from-above.md"
    assert path.read_text() == "# Grasp from above\n\nApproach 10 cm over the cube, then descend.\n"
    assert Session(session.dir).library() == [path]
