"""CLI contract tests for executable evaluations."""

import json
from pathlib import Path
from types import SimpleNamespace

from pytest_mock import MockerFixture
from typer.testing import CliRunner

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


def test_libero_pro_smoke_json_treats_zero_score_as_completed(
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
        ["eval", "run", str(CASE), "--output", str(output), "--json", "--quiet"],
    )

    assert invocation.exit_code == 0
    assert json.loads(invocation.stdout) == payload
    execute.assert_called_once_with(
        CASE,
        output=output,
        api_key_env="OPENAI_API_KEY",
        progress=None,
    )
