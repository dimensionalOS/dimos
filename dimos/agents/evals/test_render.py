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
import statistics

import pytest

from dimos.agents.evals import render
from dimos.agents.evals.contracts import AnswerRecord, Outcome, ScoreResult
from dimos.agents.evals.render import (
    collect_shard_paths,
    group_by_configuration,
    render_figure,
)
from dimos.agents.evals.scorer import ScoredCase, append_shard, read_shards

MODELS = ("gpt-5.6-luna", "openai:gpt-5.6-sol")
PROMPTS = ("shipping", "spatial")

#: One sweep's stamp; a repeat of the same configuration carries another.
RUN_ID = "20260730T091500-4242"


def make_case(
    question_id: str,
    model_id: str = "gpt-5.6-luna",
    prompt_id: str = "spatial",
    error_m: float | None = 0.8,
    outcome: Outcome = "predicted",
    run_id: str = RUN_ID,
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
        run_id=run_id,
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


# Collecting shards.


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


# Grouping.


def test_cases_group_by_model_and_prompt_together() -> None:
    """A configuration is the pair: the same model under two prompts is two rows."""
    cases = [
        make_case("q1", model_id="gpt-5.6-luna", prompt_id="shipping"),
        make_case("q2", model_id="gpt-5.6-luna", prompt_id="spatial"),
        make_case("q3", model_id="openai:gpt-5.6-sol", prompt_id="shipping"),
        make_case("q4", model_id="gpt-5.6-luna", prompt_id="shipping"),
    ]
    grouped = group_by_configuration(cases)

    assert list(grouped) == [
        ("gpt-5.6-luna", "shipping"),
        ("gpt-5.6-luna", "spatial"),
        ("openai:gpt-5.6-sol", "shipping"),
    ]
    assert [case.answer.question_id for case in grouped["gpt-5.6-luna", "shipping"]] == ["q1", "q4"]


def test_grouping_follows_the_order_the_shards_were_given() -> None:
    """Row order is the caller's, not a hash order that moves between runs."""
    forward = list(group_by_configuration([make_case("q1", "a"), make_case("q2", "b")]))
    backward = list(group_by_configuration([make_case("q2", "b"), make_case("q1", "a")]))
    assert forward == [("a", "spatial"), ("b", "spatial")]
    assert backward == [("b", "spatial"), ("a", "spatial")]


# The figure.


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


def test_two_recordings_shards_are_refused_rather_than_pooled(tmp_path: Path) -> None:
    """The failure mode that produces a plausible figure instead of a broken one.

    Rows are keyed by ``(model_id, prompt_id)`` alone, so two recordings under
    one shard directory land in the same row: dots pooled across two rooms, two
    question sets and two threshold ranges, with nothing on the figure saying
    so. Both dataset names are in the message because the operator has to know
    which directories they collapsed.
    """
    cases = [make_case("go2-bigoffice-houseplant"), make_case("go2-short-houseplant")]

    with pytest.raises(ValueError) as excinfo:
        render_figure(cases, tmp_path / "pooled.png")

    message = str(excinfo.value)
    assert "go2-bigoffice" in message
    assert "go2-short" in message
    assert not (tmp_path / "pooled.png").exists()


def test_one_row_refuses_two_prompt_texts(tmp_path: Path) -> None:
    """Same (model, prompt_id) under two prompt hashes is two configurations, not runs."""
    from dataclasses import replace

    before = make_case("q1", model_id="gpt-5.6-luna", prompt_id="shipping")
    after = make_case("q1", model_id="gpt-5.6-luna", prompt_id="shipping", run_id=RUN_ID + "-later")
    after = ScoredCase(answer=replace(after.answer, prompt_sha256="f" * 64), result=after.result)
    with pytest.raises(ValueError, match="one figure row is one prompt text"):
        render_figure([before, after], tmp_path / "mixed_prompt.png")


def test_main_refuses_a_shard_directory_holding_two_recordings(tmp_path: Path) -> None:
    """The gate is reachable the way an operator actually reaches it.

    ``--shards`` takes a directory, and the mistake is pointing it at the sweep
    root instead of at one ``<dataset>/`` beneath it.
    """
    shards = tmp_path / "shards"
    write_shard(shards / "gpt-5-6-luna__spatial__a.jsonl", [make_case("go2-bigoffice-houseplant")])
    write_shard(shards / "gpt-5-6-luna__spatial__b.jsonl", [make_case("go2-short-houseplant")])

    with pytest.raises(ValueError, match="2 datasets"):
        render.main(["--shards", str(shards), "--out", str(tmp_path / "pooled.png")])


def test_a_dataset_is_the_first_two_tokens_of_a_question_id() -> None:
    """The key the gate is built on, spelled out against the committed ids.

    Label slugs vary in length and contain hyphens of their own, so the dataset
    is read off the front rather than by stripping from the back.
    """
    assert render.dataset_of("go2-bigoffice-window-sill") == "go2-bigoffice"
    assert render.dataset_of("go2-short-office-desk") == "go2-short"


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
    # Nothing broke, so the annotation does not carry a broken count at all.
    assert "broken" not in printed
    assert out.stat().st_size <= render.MAX_PNG_BYTES


def test_two_runs_of_one_configuration_are_one_row(tmp_path: Path) -> None:
    """Repeating a configuration adds measurements to its row, not a second row.

    Two rows would invite reading run-to-run noise as a difference between
    configurations, and would halve each row's apparent sample. The annotation
    says ``runs 2`` so the doubled ``n_total`` is explained rather than
    surprising, and every error from both runs is plotted.
    """
    shards = tmp_path / "shards"
    for run_id in (RUN_ID, "20260730T104500-4243"):
        write_shard(
            shards / f"gpt-5-6-luna__spatial__{run_id}.jsonl",
            [
                make_case(f"go2-bigoffice-q{index}", error_m=error_m, run_id=run_id)
                for index, error_m in enumerate((0.35, 0.9, 1.4))
            ],
        )
    out = tmp_path / "error_distribution.png"

    assert render.main(["--shards", str(shards), "--out", str(out), "--threshold-m", "1.5"]) == 0

    cases = read_shards(collect_shard_paths([str(shards)]))
    grouped = group_by_configuration(cases)
    assert list(grouped) == [("gpt-5.6-luna", "spatial")]
    assert render._summary(cases) == "n_pred 6/6  no-prediction 0%  pass 100%  runs 2"
    assert out.stat().st_size <= render.MAX_PNG_BYTES


def test_a_single_run_row_says_nothing_about_runs() -> None:
    """``runs 1`` is the normal case, so it is left out rather than stated."""
    cases = [make_case(f"q{index}", error_m=0.4) for index in range(3)]
    assert render._summary(cases) == "n_pred 3/3  no-prediction 0%  pass 100%"


def test_the_annotation_names_the_measurements_the_rates_dropped() -> None:
    """A row whose percentages are over a smaller sample has to say so.

    Two questions here died in the harness. The rates are computed over the
    four that did not -- charging a crashed worker to the model would be the
    bug -- so ``n_pred 3/6`` next to ``pass 75%`` would look like arithmetic
    nobody can reproduce until ``broken 2`` explains it.
    """
    cases = [
        make_case("q1", error_m=0.4),
        make_case("q2", error_m=0.9),
        make_case("q3", error_m=1.4),
        make_case("q4", error_m=None, outcome="no_prediction"),
        make_case("q5", error_m=None, outcome="harness_error"),
        make_case("q6", error_m=None, outcome="tool_error"),
    ]
    assert render._summary(cases) == "n_pred 3/6  no-prediction 25%  pass 75%  broken 2"


def test_the_row_marker_is_the_median_of_that_rows_errors(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The marker is the row's only summary statistic, so it has to be the median.

    ``sorted(values)[len // 2]`` is the *upper middle* value: on the four
    errors below it reports 1.4 m where the median is 1.15 m. Six questions
    minus the ones that produced no goal is an even count often enough for the
    difference to be the number a reader takes away.
    """
    real_median = statistics.median
    medianed: list[list[float]] = []

    def spy(values: list[float]) -> float:
        medianed.append(list(values))
        return real_median(values)

    monkeypatch.setattr(render.statistics, "median", spy)
    errors = (0.35, 0.9, 1.4, 2.05)
    cases = [
        make_case(f"go2-bigoffice-q{index}", error_m=error) for index, error in enumerate(errors)
    ]
    render_figure(cases, tmp_path / "median.png")

    assert medianed == [list(errors)]
    assert real_median(errors) == pytest.approx(1.15)
    assert sorted(errors)[len(errors) // 2] == 1.4  # what the old formula returned
