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

import pytest

from dimos.autoresearch.session import Budget, Session, Split, TrialRecord
from dimos.autoresearch.skill import Validation, export_skill, skill_name, validate_skill

PROGRAM = "import math\n\ndef run(robot, task):\n    robot.arm.home()\n"


def _session(directory: Path, task: str) -> Session:
    budget = Budget(dev_trials=2, heldout_trials=3)
    return Session.create(directory, task, budget, "test-model", blueprint="autoresearch-metal-sim")


def _record(session: Session, candidate: int, split: Split, index: int, success: bool) -> None:
    trial = session.trial_dir(candidate, split, index)
    (trial / "keyframes").mkdir()
    for frame in range(6):
        (trial / "keyframes" / f"{frame:03d}.png").write_bytes(b"png")
    session.record_trial(
        candidate,
        TrialRecord(
            split=split,
            index=index,
            success=success,
            stage="done" if success else "grasp",
            note="" if success else "cube rose only 0.01 m",
            status="completed",
            duration_s=2.0,
            trace=trial / "trace.jsonl",
            start_ts=0.0,
            episode=None,
            backend="fake",
        ),
    )


def test_export_writes_a_valid_skill_from_the_frozen_candidate(tmp_path: Path) -> None:
    session = _session(tmp_path / "lift", "Lift the cube!")
    session.submit(PROGRAM, "first try")
    frozen = session.submit(PROGRAM.replace("home()", "home()\n    robot.gripper.open()"), "open")
    _record(session, 1, "dev", 0, False)
    _record(session, frozen, "dev", 0, True)
    session.note("Grasp from above", "Descend 8 cm before closing.")
    session.freeze(frozen)
    for index, success in enumerate((True, False, True)):
        _record(session, frozen, "heldout", index, success)
    session.set_status("scored", heldout_rate=2 / 3)

    skill = export_skill(session, tmp_path / "skills")

    front = validate_skill(skill)
    assert skill == tmp_path / "skills" / "lift-the-cube"
    assert front["name"] == "lift-the-cube"
    assert front["metadata"]["heldout-pass-rate"] == "0.67"
    assert front["metadata"]["dimos-commit"] == "unknown"
    assert (skill / "scripts" / "program.py").read_bytes() == session.program_path(
        frozen
    ).read_bytes()
    assert "## Grasp from above" in (skill / "references" / "findings.md").read_text()
    validation = Validation.model_validate_json((skill / "assets" / "validation.json").read_text())
    assert (validation.heldout.passed, validation.heldout.trials) == (2, 3)
    assert validation.keyframes_from == "heldout-00"
    assert len(list((skill / "assets" / "keyframes").iterdir())) == 4
    assert [path.name for path in (tmp_path / "skills").iterdir()] == ["lift-the-cube"]


def test_export_refuses_before_a_candidate_is_frozen(tmp_path: Path) -> None:
    session = _session(tmp_path / "lift", "lift the cube")
    session.submit(PROGRAM, "first try")
    with pytest.raises(ValueError, match="no frozen candidate"):
        export_skill(session, tmp_path / "skills")


@pytest.mark.parametrize("task", ["Pick UP the cube -- and   put it IN the bowl, " * 5, "?!", ""])
def test_any_task_text_gives_a_valid_name(tmp_path: Path, task: str) -> None:
    session = _session(tmp_path / "s", task or "x")
    session.submit(PROGRAM, "h")
    session.freeze(1)
    skill = export_skill(session, tmp_path / "skills", skill_name(task))
    assert len(validate_skill(skill)["name"]) <= 64
