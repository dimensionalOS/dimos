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

"""The documented way to run a SPACE task, without running one."""

from pathlib import Path

from click import unstyle
from typer.testing import CliRunner

from dimos.benchmark.space_qa.run import SpaceRunSummary
from dimos.cli.dimos import main
import dimos.cli.eval as eval_cli


def _summary(tmp_path: Path) -> SpaceRunSummary:
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


def test_the_subset_and_the_worker_count_reach_the_runner(tmp_path, monkeypatch) -> None:
    captured = {}

    def run(**kwargs):
        captured.update(kwargs)
        return _summary(tmp_path)

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


def test_a_failed_run_exits_two_and_says_why(tmp_path, monkeypatch) -> None:
    def refuse(**_kwargs):
        raise KeyError("unregistered task: 'SAtt_vision'")

    monkeypatch.setattr(eval_cli, "run_space_task", refuse)
    result = CliRunner().invoke(main, ["eval", "space", "--task=SAtt_vision", "--seed=1"])

    assert result.exit_code == 2
    assert "unregistered task" in result.stderr


def test_the_help_names_the_task_the_seed_and_the_first_run_download() -> None:
    result = CliRunner().invoke(main, ["eval", "space", "--help"], color=True)

    assert result.exit_code == 0
    help_text = unstyle(result.stdout)
    assert "--task" in help_text
    assert "--seed" in help_text
    assert "3.6 GB" in help_text
    assert "~/.cache/dimos/space-benchmark/" in help_text


def test_the_runtime_is_not_imported_to_render_the_help() -> None:
    """`eval --help` must stay usable without the SPACE extra installed."""
    result = CliRunner().invoke(main, ["eval", "--help"])

    assert result.exit_code == 0
    assert "space" in unstyle(result.stdout)
