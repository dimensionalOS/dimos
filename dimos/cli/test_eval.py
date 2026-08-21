"""CLI contract tests for executable evaluations."""

import json
from pathlib import Path
from types import SimpleNamespace

from pytest_mock import MockerFixture
from typer.testing import CliRunner

from dimos.benchmark.evaluation import container as evaluation_container
from dimos.cli import eval as eval_cli
from dimos.cli.dimos import main

CASE = (
    Path(__file__).parents[1]
    / "benchmark"
    / "libero_pro"
    / "cases"
    / "goal-task-0-single-trial"
    / "evaluation.json"
)


def test_run_dispatches_to_locked_evaluation_container(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    run_container = mocker.patch.object(evaluation_container, "run_evaluation", return_value=0)
    output = tmp_path / "output"

    invocation = CliRunner().invoke(
        main,
        ["eval", "run", str(CASE), "--output", str(output), "--json", "--quiet"],
    )

    assert invocation.exit_code == 0
    run_container.assert_called_once_with(
        CASE,
        output=output,
        api_key_env="OPENAI_API_KEY",
        json_output=True,
        quiet=True,
    )


def test_check_dispatches_to_shared_container_preflight(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    check_environment = mocker.patch.object(evaluation_container, "check_environment")
    workspace = tmp_path / "check"

    invocation = CliRunner().invoke(
        main,
        ["eval", "check", "--workspace", str(workspace)],
    )

    assert invocation.exit_code == 0
    check_environment.assert_called_once_with(workspace)


def test_inside_run_treats_zero_score_as_completed(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    payload = {
        "status": "completed",
        "report": {"native_result": {"kind": "inline", "value": {"score": 0.0}}},
    }
    result = SimpleNamespace(
        status="completed",
        model_dump_json=lambda: json.dumps(payload),
    )
    execute = mocker.patch.object(eval_cli, "execute_evaluation", return_value=result)
    output = tmp_path / "output"

    invocation = CliRunner().invoke(
        main,
        ["eval", "_inside-run", str(CASE), "--output", str(output), "--json", "--quiet"],
    )

    assert invocation.exit_code == 0
    assert json.loads(invocation.stdout) == payload
    execute.assert_called_once_with(
        CASE,
        output=output,
        api_key_env="OPENAI_API_KEY",
        progress=None,
    )
