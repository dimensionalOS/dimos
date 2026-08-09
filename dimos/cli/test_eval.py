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
import json
from pathlib import Path
import subprocess
import sys
import textwrap

from click import unstyle
import pytest
from typer.testing import CliRunner

from dimos.benchmark.agent_eval.models import CompactEvalResult
from dimos.benchmark.agent_eval.progress import (
    AssistantTextProgress,
    StatusProgress,
    ToolEndProgress,
)
from dimos.benchmark.space_qa.run import SpaceRunSummary
from dimos.cli.dimos import main
import dimos.cli.eval as eval_cli


def _result(
    *,
    passed: bool | None = True,
    infra_error: str | None = None,
    recording: str | None = "go2_hongkong_office",
) -> CompactEvalResult:
    return CompactEvalResult(
        case_id="demo-room-count",
        recording=recording,
        progress=1.0 if recording is not None else None,
        model="gpt-5.6-luna",
        thinking_level="medium",
        final_response="ANSWER: 4" if passed is not None else "",
        prediction_status="parsed" if passed is not None else "not_evaluated",
        integer_answer=4 if passed is not None else None,
        passed=passed,
        validator_revision="v1",
        tool_call_count=7,
        duration_seconds=42.75,
        infra_error=infra_error,
    )


def _case(tmp_path: Path) -> Path:
    path = tmp_path / "case.json"
    path.write_text("{}")
    return path


def _space_summary(tmp_path: Path) -> SpaceRunSummary:
    return SpaceRunSummary(
        task="SAtt_text",
        seed=20260808,
        groups=8,
        questions=32,
        mean_accuracy=62.5,
        run_dir=tmp_path,
        manifest_path=tmp_path / "manifest.json",
        results_path=tmp_path / "space" / "dimos_qa" / "20260808_010203" / "results.json",
        records_path=tmp_path / "cases.jsonl",
    )


def test_eval_run_uses_api_key_default_and_separates_progress(tmp_path, monkeypatch) -> None:
    captured = {}

    def execute(path, *, config, progress, output):
        captured.update(path=path, config=config, progress=progress, output=output)
        progress(StatusProgress(channel="eval", message="loading case"))
        return _result()

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)
    output = tmp_path / "run"
    result = CliRunner().invoke(main, ["eval", "run", str(_case(tmp_path)), f"--output={output}"])
    assert result.exit_code == 0, result.output
    assert captured["config"].agent.api_key_env == "OPENAI_API_KEY"
    assert captured["output"] == output
    assert "✓ Evaluation passed" in result.stdout
    assert "[eval] loading case" in result.stderr


def test_eval_run_accepts_named_api_key_env_and_json(tmp_path, monkeypatch) -> None:
    captured = {}

    def execute(*args, **kwargs):
        captured.update(kwargs)
        return _result()

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)
    output = tmp_path / "run"
    result = CliRunner().invoke(
        main,
        [
            "eval",
            "run",
            str(_case(tmp_path)),
            "--agent.api-key-env=MY_OPENAI_KEY",
            f"--output={output}",
            "--json",
        ],
    )
    assert result.exit_code == 0, result.output
    assert captured["config"].agent.api_key_env == "MY_OPENAI_KEY"
    assert json.loads(result.stdout)["passed"] is True


def test_eval_exit_codes_distinguish_infra_semantic_and_preflight(tmp_path, monkeypatch) -> None:
    output = tmp_path / "run"
    monkeypatch.setattr(
        eval_cli,
        "execute_single_case",
        lambda *a, **k: _result(passed=None, infra_error="Pi failed"),
    )
    infra = CliRunner().invoke(
        main, ["eval", "run", str(_case(tmp_path)), f"--output={output}", "--quiet"]
    )
    monkeypatch.setattr(eval_cli, "execute_single_case", lambda *a, **k: _result(passed=False))
    semantic = CliRunner().invoke(
        main, ["eval", "run", str(_case(tmp_path)), f"--output={output}", "--quiet"]
    )

    def preflight(*_args, **_kwargs):
        raise FileNotFoundError("extension build missing")

    monkeypatch.setattr(eval_cli, "execute_single_case", preflight)
    preflight_result = CliRunner().invoke(
        main, ["eval", "run", str(_case(tmp_path)), f"--output={output}"]
    )
    assert infra.exit_code == 1
    assert semantic.exit_code == 0
    assert preflight_result.exit_code == 2


def test_eval_help_is_typed_and_output_is_required(tmp_path) -> None:
    runner = CliRunner()
    help_result = runner.invoke(main, ["eval", "run", "--help"], color=True)
    missing_output = runner.invoke(main, ["eval", "run", str(_case(tmp_path))])
    assert help_result.exit_code == 0
    help_text = unstyle(help_result.stdout)
    assert "--agent.api-key-env" in help_text
    assert "--output" in help_text
    assert missing_output.exit_code == 2


def test_lazy_runtime_import_has_actionable_error(monkeypatch) -> None:
    original_import = builtins.__import__

    def fail_single_case(name, *args, **kwargs):
        if name == "dimos.benchmark.agent_eval.single_case":
            raise ModuleNotFoundError("No module named 'mcp'")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fail_single_case)
    with pytest.raises(RuntimeError, match="uv sync --extra agents"):
        eval_cli.execute_single_case(Path("case.json"), config=None)


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
        [sys.executable, "-c", script], capture_output=True, text=True, check=False
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr


def test_format_result_reports_an_externally_scored_answer_as_recorded() -> None:
    rendered = eval_cli.format_result(_result(passed=None))

    assert rendered.startswith("· Answer recorded for external scoring")
    assert "not_evaluated" in rendered


def test_format_result_names_a_run_without_an_environment() -> None:
    rendered = eval_cli.format_result(_result(recording=None))

    assert "no environment" in rendered


def test_eval_space_passes_the_subset_and_worker_count_to_the_runner(tmp_path, monkeypatch) -> None:
    captured = {}

    def run(**kwargs):
        captured.update(kwargs)
        return _space_summary(tmp_path)

    monkeypatch.setattr(eval_cli, "run_space_task", run)
    result = CliRunner().invoke(
        main,
        [
            "eval",
            "space",
            "--task=SAtt_text",
            "--groups=8",
            "--seed=20260808",
            "--workers=2",
            f"--output={tmp_path}",
        ],
    )

    assert result.exit_code == 0, result.output
    assert captured == {
        "task_name": "SAtt_text",
        "groups": 8,
        "seed": 20260808,
        "workers": 2,
        "output": tmp_path,
    }
    assert "62.5% (scored by SPACE)" in result.stdout
    assert "results.json" in result.stdout


def test_eval_space_failure_exits_two_and_says_why(tmp_path, monkeypatch) -> None:
    def refuse(**_kwargs):
        raise KeyError("unregistered task: 'SAtt_vision'")

    monkeypatch.setattr(eval_cli, "run_space_task", refuse)
    result = CliRunner().invoke(main, ["eval", "space", "--task=SAtt_vision", "--seed=1"])

    assert result.exit_code == 2
    assert "unregistered task" in result.stderr


def test_eval_space_help_names_the_task_the_seed_and_the_first_run_download() -> None:
    result = CliRunner().invoke(main, ["eval", "space", "--help"], color=True)

    assert result.exit_code == 0
    help_text = unstyle(result.stdout)
    assert "--task" in help_text
    assert "--seed" in help_text
    assert "3.6 GB" in help_text
    assert "~/.cache/dimos/space-benchmark/" in help_text


def test_eval_help_renders_without_the_space_runtime() -> None:
    result = CliRunner().invoke(main, ["eval", "--help"])

    assert result.exit_code == 0
    assert "space" in unstyle(result.stdout)


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
