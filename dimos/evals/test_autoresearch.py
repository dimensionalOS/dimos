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

"""The autoresearch driver without a lab: launch settings, settling, the
prompt and the grader over a session directory built with the session API."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from pytest_mock import MockerFixture

from dimos.autoresearch.prompt import EXCLUDED_KEYWORDS
from dimos.autoresearch.session import Budget, Session, Split, TrialRecord
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.constants import DENIED
from dimos.evals.environments.autoresearch import AutoresearchEnvironment
from dimos.evals.runner import forbidden_call
from dimos.evals.suites.autoresearch_metal import (
    BUDGET,
    REPORT_NAME,
    SUITE,
    TOOLS,
    heldout_rate,
)
from dimos.evals.types import Metrics, Outcome, ToolCall, Trajectory

PROGRAM = "import math\n\ndef run(robot, task):\n    robot.arm.home()\n"


def environment(tmp_path: Path) -> AutoresearchEnvironment:
    return AutoresearchEnvironment(task="lift the cube", budget=BUDGET, session_root=tmp_path)


def launched(tmp_path: Path) -> tuple[AutoresearchEnvironment, Path]:
    env = environment(tmp_path)
    env.configure_launch(DimosCliCall())
    return env, env.session_dir


def trial(split: Split, index: int, success: bool) -> TrialRecord:
    return TrialRecord(
        split=split,
        index=index,
        success=success,
        stage="done" if success else "grasp",
        note="",
        status="completed",
        duration_s=1.0,
        trace=Path("/nowhere/trace.jsonl"),
        start_ts=0.0,
        episode=None,
        backend="fake",
    )


def trajectory(*commands: tuple[str, str], at: float = 1000.0) -> Trajectory:
    """One agent step per (bash command, result), ten seconds apart, 100 prompt tokens each."""
    builder = TrajectoryBuilder("go", name="t", model="m")
    for index, (command, result) in enumerate(commands):
        call_id = f"c{index}"
        builder.step(
            message="",
            tool_calls=(
                ToolCall(
                    tool_call_id=call_id, function_name="bash", arguments={"command": command}
                ),
            ),
            metrics=Metrics(prompt_tokens=100, completion_tokens=10),
            latency_s=2.0,
            request=Path("r"),
            response=Path("s"),
            at=at + 10.0 * index,
        )
        builder.observe(call_id, result)
    return builder.build("answer")


def test_the_suite_tells_the_actor_every_tool_and_the_program_api() -> None:
    (lift,) = SUITE
    assert lift.id == "lift_the_cube"
    for tool in TOOLS:
        assert f"{tool}(" in lift.inputs
    assert "run(robot, task)" in lift.inputs and "import math" in lift.inputs
    assert str(BUDGET.candidates_per_round) in lift.inputs
    assert lift.threshold == BUDGET.threshold
    assert isinstance(lift.environment, AutoresearchEnvironment)
    assert lift.environment.config.blueprint == ["autoresearch-metal-sim"]
    assert lift.environment.config.agent_artifacts == ("session",)


def test_launch_writes_the_lab_settings_and_a_fresh_session_dir(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    env = environment(tmp_path)
    proc = DimosCliCall()
    env.configure_launch(proc)
    session = Path(proc.extra_env["AUTORESEARCHMODULE__SESSION_DIR"])
    assert session.parent == tmp_path and session.is_dir() and not list(session.iterdir())
    assert proc.extra_env["AUTORESEARCHMODULE__TASK"] == "lift the cube"
    assert Budget.model_validate_json(proc.extra_env["AUTORESEARCHMODULE__BUDGET"]) == BUDGET
    assert proc.extra_env["RECORDER__RECORDING"] == str(session / "recording")
    assert proc.extra_env["EPISODES__TASK"] == "lift the cube"
    assert proc.simulator == "mujoco"
    assert proc.global_args == ["--record-topics", "coordinator_joint_state,body_poses,tf"]

    mocker.patch.object(env, "wait_ready")
    assert env.prepare_recording(mocker.MagicMock(), tmp_path / "memory.db", 0.0) == {
        "session": session
    }

    env.configure_launch(DimosCliCall())
    assert env.session_dir != session and env.session_dir.parent == tmp_path


def test_settle_returns_once_scored_and_raises_when_the_budget_runs_out(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    env, session = launched(tmp_path)
    info = session / "session.json"
    info.write_text(json.dumps({"status": "frozen"}))
    sleep = mocker.patch(
        "dimos.evals.environments.autoresearch.time.sleep",
        side_effect=lambda s: info.write_text(json.dumps({"status": "scored", "heldout_rate": 1})),
    )

    env.settle(60.0)

    assert sleep.call_count == 1
    info.write_text(json.dumps({"status": "frozen"}))
    with pytest.raises(TimeoutError, match="frozen"):
        env.settle(0.0)
    info.unlink()
    with pytest.raises(TimeoutError, match="missing"):
        env.settle(0.0)


def test_grade_reads_the_frozen_candidates_heldout_rate_and_reports(tmp_path: Path) -> None:
    session = Session.create(tmp_path / "s", task="lift the cube", budget=BUDGET, model="m")
    session.submit(PROGRAM, "straight down")
    second = session.submit(PROGRAM.replace("home()", "home()\n    robot.gripper.open()"), "open")
    session.record_trial(1, trial("dev", 0, False))
    session.record_trial(second, trial("dev", 0, True))
    for index in range(5):
        session.record_trial(second, trial("heldout", index, index < 4))
    session.freeze(second)
    # The lab appends the held-out score once its trials are done.
    info = json.loads((session.dir / "session.json").read_text())
    info.update(status="scored", heldout_rate=0.8)
    (session.dir / "session.json").write_text(json.dumps(info))
    steps = trajectory(
        ("dimos mcp call task", "lift the cube"),
        ("dimos mcp call submit --json-args @1", "candidate 1"),
        ("dimos mcp call run --json-args '{\"candidate\": 1}'", "started"),
        ("dimos mcp call results --json-args '{\"candidate\": 1}'", "[...]"),
        ("dimos mcp call submit --json-args @2", "candidate 2"),
        ("dimos mcp call freeze --json-args '{\"candidate\": 2}'", "frozen"),
    )
    outcome = Outcome(trajectory=steps, artifacts={"session": session.dir})

    assert heldout_rate(outcome) == pytest.approx(0.8)

    report = json.loads((session.dir / REPORT_NAME).read_text())
    assert report["frozen"] == 2 and report["threshold"] == BUDGET.threshold
    assert [row["candidate"] for row in report["leaderboard"]] == [2, 1]
    assert report["leaderboard"][0]["heldout_passed"] == 4
    assert report["iterations"] == 2
    assert [row["prompt_tokens"] for row in report["per_iteration"]] == [200, 300]
    assert [row["seconds"] for row in report["per_iteration"]] == [12.0, 22.0]

    info.update(status="frozen")
    (session.dir / "session.json").write_text(json.dumps(info))
    with pytest.raises(RuntimeError, match="not scored"):
        heldout_rate(outcome)


def test_the_keyword_guard_catches_a_shell_call_into_a_movement_skill() -> None:
    reached = trajectory(('dimos mcp call pick_object --json-args \'{"name": "cube"}\'', "ok"))
    assert forbidden_call(reached, EXCLUDED_KEYWORDS).startswith("invalid: step 2 ran bash")
    denied = trajectory(("dimos mcp call pick_object", f"{DENIED}: pick_object"))
    assert forbidden_call(denied, EXCLUDED_KEYWORDS) == ""
    allowed = trajectory(("dimos mcp call submit --json-args @1", "candidate 1"))
    assert forbidden_call(allowed, EXCLUDED_KEYWORDS) == ""
