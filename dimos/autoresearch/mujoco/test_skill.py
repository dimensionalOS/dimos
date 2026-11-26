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

import json
from pathlib import Path

import pytest

from dimos.autoresearch.mujoco.skill import Validation, export_skill, validate_skill
from dimos.autoresearch.session import Budget, Evaluation, Session, SessionInfo, Trial
from dimos.autoresearch.skill import skill_name
from dimos.manipulation.code_policy.api import API_REFERENCE, API_VERSION

PROGRAM = "import math\n\ndef run(robot, task):\n    robot.arm.home()\n"


def _session(directory: Path, task: str) -> Session:
    budget = Budget(dev_trials=2, heldout_trials=3)
    return Session.create(
        directory,
        SessionInfo(
            task=task,
            budget=budget,
            experiment="mujoco",
            metric="pass rate",
            settings={"blueprint": "autoresearch-metal-sim"},
        ),
    )


def _measure(trial: Trial, success: bool) -> Evaluation:
    (trial.directory / "keyframes").mkdir()
    for frame in range(6):
        (trial.directory / "keyframes" / f"{frame:03d}.png").write_bytes(b"png")
    return Evaluation(
        score=float(success),
        note="" if success else "cube rose only 0.01 m",
        evidence=dict(
            stage="done" if success else "grasp",
            status="completed",
            duration_s=2.0,
            trace=str(trial.directory / "trace.jsonl"),
            start_ts=0.0,
            episode=None,
            backend="fake",
        ),
    )


def test_export_writes_a_valid_skill_from_the_frozen_candidate(tmp_path: Path) -> None:
    session = _session(tmp_path / "lift", "Lift the cube!")
    session.set_model("test-model")
    session.submit("program.py", PROGRAM.encode(), "first try")
    frozen = session.submit(
        "program.py", PROGRAM.replace("home()", "home()\n    robot.gripper.open()").encode(), "open"
    )
    session.evaluate(1, "dev", lambda trial: _measure(trial, False))
    session.evaluate(frozen, "dev", lambda trial: _measure(trial, True))
    session.note("Grasp from above", "Descend 8 cm before closing.")
    session.freeze(frozen)
    session.evaluate(frozen, "heldout", lambda trial: _measure(trial, trial.index != 1))

    skill = export_skill(session, tmp_path / "skills")

    front = validate_skill(skill)
    assert skill == tmp_path / "skills" / "lift-the-cube"
    assert front["name"] == "lift-the-cube"
    assert front["metadata"]["heldout-pass-rate"] == "0.67"
    assert front["metadata"]["dimos-commit"] == "unknown"
    assert front["metadata"]["model"] == "test-model"
    assert front["metadata"]["code-policy-api"] == API_VERSION
    assert (skill / "references" / "api.md").read_text().count(API_REFERENCE) == 1
    assert (skill / "scripts" / "program.py").read_bytes() == session.artifact(frozen).read_bytes()
    assert "## Grasp from above" in (skill / "references" / "findings.md").read_text()
    validation = Validation.model_validate_json((skill / "assets" / "validation.json").read_text())
    assert (validation.heldout.passed, validation.heldout.trials) == (2, 3)
    assert validation.keyframes_from == "heldout-00"
    assert len(list((skill / "assets" / "keyframes").iterdir())) == 4
    assert [path.name for path in (tmp_path / "skills").iterdir()] == ["lift-the-cube"]


def test_export_refuses_before_a_candidate_is_frozen(tmp_path: Path) -> None:
    session = _session(tmp_path / "lift", "lift the cube")
    session.submit("program.py", PROGRAM.encode(), "first try")
    with pytest.raises(ValueError, match="no frozen candidate"):
        export_skill(session, tmp_path / "skills")


def test_export_refuses_a_program_without_a_passing_heldout_score_unless_forced(
    tmp_path: Path,
) -> None:
    session = _session(tmp_path / "lift", "lift the cube")
    session.submit("program.py", PROGRAM.encode(), "first try")
    session.freeze(1)
    with pytest.raises(ValueError, match=r"no held-out score yet \(0 of 3 held-out trials run\)"):
        export_skill(session, tmp_path / "skills")
    session.evaluate(1, "heldout", lambda trial: _measure(trial, False))
    with pytest.raises(ValueError, match=r"passed 0/3 held-out trials \(pass rate 0.00\)"):
        export_skill(session, tmp_path / "skills")

    skill = export_skill(session, tmp_path / "skills", force=True)

    assert validate_skill(skill)["metadata"]["heldout-pass-rate"] == "0.00"
    assert validate_skill(skill)["metadata"]["model"] == "none"
    assert "Held-out: 0/3 passed." in (skill / "SKILL.md").read_text()


@pytest.mark.parametrize("task", ["Pick UP the cube -- and   put it IN the bowl, " * 5, "?!", ""])
def test_any_task_text_gives_a_valid_name(tmp_path: Path, task: str) -> None:
    session = _session(tmp_path / "s", task or "x")
    session.submit("program.py", PROGRAM.encode(), "h")
    session.freeze(1)
    skill = export_skill(session, tmp_path / "skills", skill_name(task), force=True)
    assert len(validate_skill(skill)["name"]) <= 64


def test_original_arm_recording_can_be_exported_after_relocation(tmp_path: Path) -> None:
    directory = tmp_path / "old"
    candidate = directory / "candidates/01"
    frames = candidate / "trials/heldout-00/keyframes"
    frames.mkdir(parents=True)
    (frames / "one.png").write_bytes(b"old keyframe")
    (directory / "library").mkdir()
    (directory / "session.json").write_text(
        json.dumps(
            {
                "task": "lift cube",
                "budget": {
                    "iterations": 2,
                    "candidates_per_round": 3,
                    "dev_trials": 1,
                    "heldout_trials": 1,
                },
                "status": "scored",
                "heldout_rate": 1.0,
                "blueprint": "autoresearch-metal-sim",
            }
        )
    )
    (directory / "frozen").write_text("1")
    (candidate / "program.py").write_text(PROGRAM)
    (candidate / "hypothesis.md").write_text("baseline")
    (candidate / "results.json").write_text(
        json.dumps(
            [
                {
                    "split": "heldout",
                    "index": 0,
                    "success": True,
                    "stage": "done",
                    "note": "lifted",
                    "status": "completed",
                    "duration_s": 1,
                    "start_ts": 0,
                    "episode": 0,
                    "backend": "sim",
                    "trace": "/previous-host/session/candidates/01/trials/heldout-00/trace.jsonl",
                }
            ]
        )
    )
    session = Session(directory)
    folder = export_skill(session, tmp_path / "skills")
    assert session.info().budget.max_candidates == 6
    assert (folder / "scripts/program.py").read_text() == PROGRAM
    assert (folder / "assets/keyframes/one.png").read_bytes() == b"old keyframe"
    assert validate_skill(folder)["metadata"]["heldout-pass-rate"] == "1.00"
