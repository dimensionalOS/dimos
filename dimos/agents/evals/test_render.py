# Copyright 2025-2026 Dimensional Inc.
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

"""Shard collection, grouping and figure rendering, on synthetic shards.

The renderer is the last step of the eval and the one whose output gets
committed, so the two things that must not regress are the grouping (a figure
that mixes two configurations into one row is worse than no figure) and the
75 KB ceiling the repository's large-file hook enforces.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.agents.evals import render
from dimos.agents.evals.contracts import AnswerRecord, Outcome, ScoreResult
from dimos.agents.evals.render import (
    collect_shard_paths,
    group_by_configuration,
    render_figure,
)
from dimos.agents.evals.scorer import ScoredCase, append_shard

MODELS = ("gpt-5.6-luna", "openai:gpt-4o")
PROMPTS = ("plain", "spatial")


def make_case(
    question_id: str,
    model_id: str = "gpt-5.6-luna",
    prompt_id: str = "spatial",
    error_m: float | None = 0.8,
    outcome: Outcome = "predicted",
) -> ScoredCase:
    answer = AnswerRecord(
        question_id=question_id,
        outcome=outcome,
        goal_x=1.0 if error_m is not None else None,
        goal_y=0.0 if error_m is not None else None,
        goal_yaw=0.0 if error_m is not None else None,
        n_goals=1 if error_m is not None else 0,
        tool_invoked=True,
        tool_queries=[question_id],
        model_id=model_id,
        prompt_id=prompt_id,
        prompt_sha256="0" * 64,
        wall_time_s=3.0,
    )
    result = ScoreResult(
        question_id=question_id,
        passed=error_m is not None and error_m <= 1.5,
        reason=outcome,
        score=1.0 if (error_m is not None and error_m <= 1.5) else 0.0,
        error_m=error_m,
        outcome=outcome,
    )
    return ScoredCase(answer=answer, result=result)


#: Six questions' worth of errors: three inside the 1.5 m threshold, two
#: outside it, and one question the agent never answered.
ERRORS_M = (0.35, 0.9, 1.4, 2.05, 3.6, None)


def make_configuration_cases(model_id: str, prompt_id: str) -> list[ScoredCase]:
    """One case's worth of scored questions -- what a single shard file holds."""
    return [
        make_case(
            f"go2-bigoffice-q{index}",
            model_id=model_id,
            prompt_id=prompt_id,
            error_m=error_m,
            outcome="predicted" if error_m is not None else "no_prediction",
        )
        for index, error_m in enumerate(ERRORS_M)
    ]


def make_sweep_cases() -> list[ScoredCase]:
    """The whole 2x2 sweep: six questions under each of the four configurations."""
    return [
        case
        for model_id in MODELS
        for prompt_id in PROMPTS
        for case in make_configuration_cases(model_id, prompt_id)
    ]


def write_shard(path: Path, cases: list[ScoredCase]) -> Path:
    for case in cases:
        append_shard(path, case.answer, case.result)
    return path


# --- collecting shards ------------------------------------------------------


def test_collect_shard_paths_accepts_a_file_a_directory_and_a_glob(tmp_path: Path) -> None:
    """One flag has to serve `--shards out/`, `--shards a.jsonl b.jsonl` and `out/*.jsonl`."""
    directory = tmp_path / "shards"
    first = write_shard(directory / "b.jsonl", [make_case("q1")])
    second = write_shard(directory / "a.jsonl", [make_case("q2")])
    (directory / "notes.txt").write_text("not a shard")

    assert collect_shard_paths([str(first)]) == [first]
    assert collect_shard_paths([str(directory)]) == [second, first]  # sorted, and only *.jsonl
    assert collect_shard_paths([str(directory / "*.jsonl")]) == [second, first]


def test_collect_shard_paths_refuses_to_render_nothing(tmp_path: Path) -> None:
    with pytest.raises(SystemExit, match="no shard files matched"):
        collect_shard_paths([str(tmp_path / "does-not-exist" / "*.jsonl")])


# --- grouping ---------------------------------------------------------------


def test_cases_group_by_model_and_prompt_together() -> None:
    """A configuration is the pair: the same model under two prompts is two rows."""
    cases = [
        make_case("q1", model_id="gpt-5.6-luna", prompt_id="plain"),
        make_case("q2", model_id="gpt-5.6-luna", prompt_id="spatial"),
        make_case("q3", model_id="openai:gpt-4o", prompt_id="plain"),
        make_case("q4", model_id="gpt-5.6-luna", prompt_id="plain"),
    ]
    grouped = group_by_configuration(cases)

    assert list(grouped) == [
        ("gpt-5.6-luna", "plain"),
        ("gpt-5.6-luna", "spatial"),
        ("openai:gpt-4o", "plain"),
    ]
    assert [case.answer.question_id for case in grouped["gpt-5.6-luna", "plain"]] == ["q1", "q4"]


def test_grouping_follows_the_order_the_shards_were_given() -> None:
    """Row order is the caller's, not a hash order that moves between runs."""
    forward = list(group_by_configuration([make_case("q1", "a"), make_case("q2", "b")]))
    backward = list(group_by_configuration([make_case("q2", "b"), make_case("q1", "a")]))
    assert forward == [("a", "spatial"), ("b", "spatial")]
    assert backward == [("b", "spatial"), ("a", "spatial")]


# --- the figure -------------------------------------------------------------


def test_figure_of_the_full_sweep_fits_the_large_file_limit(tmp_path: Path) -> None:
    """24 points over 4 configurations -- the shape the eval actually produces."""
    out = render_figure(
        make_sweep_cases(), tmp_path / "figures" / "error_distribution.png", threshold_m=1.5
    )
    assert out.exists()
    assert out.stat().st_size <= render.MAX_PNG_BYTES


def test_figure_needs_something_to_plot(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="no scored cases"):
        render_figure([], tmp_path / "empty.png")


def test_an_oversized_figure_fails_and_is_not_left_behind(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The limit is a hook in pre-commit; a too-big PNG must not be committable.

    The ceiling is lowered instead of the figure being inflated, so the test
    stays fast and does not depend on how matplotlib compresses this month.
    """
    monkeypatch.setattr(render, "MAX_PNG_BYTES", 1024)
    out = tmp_path / "too-big.png"
    with pytest.raises(ValueError, match="over the 1024-byte large-file limit"):
        render_figure(make_sweep_cases(), out, threshold_m=1.5)
    assert not out.exists()


def test_main_renders_a_directory_of_shards_and_reports_the_counts(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    shards = tmp_path / "shards"
    for model_id in MODELS:
        for prompt_id in PROMPTS:
            # One shard per case: parallel pytest cases never share a file, and
            # a file may therefore hold each question id at most once.
            write_shard(
                shards / f"{model_id.replace(':', '-')}__{prompt_id}.jsonl",
                make_configuration_cases(model_id, prompt_id),
            )
    out = tmp_path / "error_distribution.png"

    assert render.main(["--shards", str(shards), "--out", str(out), "--threshold-m", "1.5"]) == 0

    printed = capsys.readouterr().out
    assert f"{out}" in printed
    assert "24 cases from 4 shard(s)" in printed
    # 5 of the 6 questions produced a goal, and 3 of those are within 1.5 m.
    assert printed.count("n_pred 5/6  no-prediction 17%  pass 50%") == 4
    assert out.stat().st_size <= render.MAX_PNG_BYTES
