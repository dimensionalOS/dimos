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

import builtins
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import textwrap

from click import unstyle
import pytest
from typer.testing import CliRunner

from dimos.benchmark.evaluation.models import (
    CodePolicyAgentConfig,
    EvaluationIdentity,
    EvaluationReference,
    EvaluationReport,
    EvaluationRun,
    EvaluationRunError,
    EvaluationRunSpecification,
    InlineNativeResult,
    RuntimeIdentity,
    SummaryItem,
)
from dimos.benchmark.evaluation.progress import (
    AssistantTextProgress,
    StatusProgress,
    ToolEndProgress,
)
from dimos.cli.dimos import main
import dimos.cli.eval as eval_cli


def _result(status: str = "completed") -> EvaluationRun:
    now = datetime.now(timezone.utc)
    completed = status == "completed"
    return EvaluationRun(
        run_id="run-1",
        specification=EvaluationRunSpecification(
            evaluation=EvaluationReference(name="fixture", config={}),
            agent=CodePolicyAgentConfig(),
        ),
        evaluation=EvaluationIdentity(name="fixture", provider="tests", version="1"),
        runtime=RuntimeIdentity(
            driver_version="test",
            model="gpt-5.6-luna",
            thinking_level="medium",
        ),
        status=status,
        started_at=now,
        finished_at=now,
        duration_seconds=1.0,
        report=(
            EvaluationReport(
                summary=(SummaryItem(key="native_score", label="Native score", value=0.5),),
                native_result=InlineNativeResult(value={"score": 0.5}),
            )
            if completed
            else None
        ),
        error=(
            None
            if completed
            else EvaluationRunError(
                stage="evaluation",
                error_type="RuntimeError",
                message="agent failed",
            )
        ),
    )


def _spec(tmp_path: Path) -> Path:
    path = tmp_path / "spec.json"
    path.write_text("{}")
    return path


def test_eval_run_uses_operational_api_key_and_renders_native_summary(
    tmp_path,
    monkeypatch,
) -> None:
    captured = {}

    def execute(path, *, api_key_env, progress, output):
        captured.update(
            path=path,
            api_key_env=api_key_env,
            progress=progress,
            output=output,
        )
        progress(StatusProgress(channel="eval", message="loading specification"))
        return _result()

    monkeypatch.setattr(eval_cli, "execute_evaluation", execute)
    output = tmp_path / "run"

    result = CliRunner().invoke(
        main,
        ["eval", "run", str(_spec(tmp_path)), f"--output={output}"],
    )

    assert result.exit_code == 0, result.output
    assert captured["api_key_env"] == "OPENAI_API_KEY"
    assert captured["output"] == output
    assert "✓ Evaluation completed" in result.stdout
    assert "Native score" in result.stdout
    assert "[eval] loading specification" in result.stderr


def test_eval_run_accepts_named_api_key_env_and_json(tmp_path, monkeypatch) -> None:
    captured = {}

    def execute(*args, **kwargs):
        captured.update(kwargs)
        return _result()

    monkeypatch.setattr(eval_cli, "execute_evaluation", execute)
    output = tmp_path / "run"
    result = CliRunner().invoke(
        main,
        [
            "eval",
            "run",
            str(_spec(tmp_path)),
            "--api-key-env=MY_OPENAI_KEY",
            f"--output={output}",
            "--json",
        ],
    )

    assert result.exit_code == 0, result.output
    assert captured["api_key_env"] == "MY_OPENAI_KEY"
    payload = json.loads(result.stdout)
    assert payload["status"] == "completed"
    assert payload["report"]["native_result"]["value"] == {"score": 0.5}


@pytest.mark.parametrize(("status", "exit_code"), [("failed", 1), ("cancelled", 130)])
def test_eval_exit_codes_for_noncompleted_runs(
    status,
    exit_code,
    tmp_path,
    monkeypatch,
) -> None:
    monkeypatch.setattr(eval_cli, "execute_evaluation", lambda *a, **k: _result(status))

    result = CliRunner().invoke(
        main,
        ["eval", "run", str(_spec(tmp_path)), f"--output={tmp_path / 'run'}", "--quiet"],
    )

    assert result.exit_code == exit_code


def test_eval_preflight_failure_uses_exit_two(tmp_path, monkeypatch) -> None:
    def preflight(*_args, **_kwargs):
        raise FileNotFoundError("evaluation plugin missing")

    monkeypatch.setattr(eval_cli, "execute_evaluation", preflight)
    result = CliRunner().invoke(
        main,
        ["eval", "run", str(_spec(tmp_path)), f"--output={tmp_path / 'run'}"],
    )

    assert result.exit_code == 2


def test_eval_help_exposes_thin_operational_settings(tmp_path) -> None:
    runner = CliRunner()
    help_result = runner.invoke(main, ["eval", "run", "--help"], color=True)
    missing_output = runner.invoke(main, ["eval", "run", str(_spec(tmp_path))])

    assert help_result.exit_code == 0
    help_text = unstyle(help_result.stdout)
    assert "--api-key-env" in help_text
    assert "--agent.model" not in help_text
    assert "--output" in help_text
    assert missing_output.exit_code == 2


def test_lazy_runtime_import_has_actionable_error(monkeypatch) -> None:
    original_import = builtins.__import__

    def fail_runtime(name, *args, **kwargs):
        if name == "dimos.benchmark.evaluation.runner":
            raise ModuleNotFoundError("No module named 'mcp'")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fail_runtime)

    with pytest.raises(RuntimeError, match="uv sync --extra agents"):
        eval_cli.execute_evaluation(Path("spec.json"), output=Path("output"))


def test_base_cli_help_imports_without_eval_runtime() -> None:
    script = textwrap.dedent(
        """
        import sys

        class BlockEvalImports:
            def find_spec(self, fullname, path=None, target=None):
                if fullname.split('.')[0] in {'mcp', 'ipykernel', 'jupyter_client', 'uvicorn'}:
                    raise RuntimeError(f'eval runtime import attempted: {fullname}')
                return None

        sys.meta_path.insert(0, BlockEvalImports())
        from typer.testing import CliRunner
        from dimos.cli.dimos import main
        result = CliRunner().invoke(main, ['--help'])
        assert result.exit_code == 0, result.output
        assert 'eval' in result.stdout
        """
    )
    completed = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr


def test_progress_renderer_ignores_leading_assistant_whitespace(capsys) -> None:
    renderer = eval_cli.ProgressRenderer()

    renderer(AssistantTextProgress(delta="\n"))

    assert capsys.readouterr().err == ""


def test_progress_renderer_truncates_large_tool_results(capsys) -> None:
    renderer = eval_cli.ProgressRenderer()
    result = "x" * 10_000

    renderer(ToolEndProgress(ok=True, result=result, duration_seconds=0.1))

    rendered = capsys.readouterr().err
    assert len(rendered) < len(result)
    assert "terminal output truncated" in rendered
