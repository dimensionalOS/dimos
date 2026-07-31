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

"""Render eval shards into one goal-error distribution figure.

Usage::

    uv run python -m dimos.agents.evals.render \\
        --shards path/to/shards/ --out error_distribution.png --threshold-m 3.5

``--shards`` takes any mix of files, directories (every ``*.jsonl`` inside) and
globs. Shards are the per-case JSONL files written by
``dimos.agents.evals.scorer.append_shard`` -- one file per pytest case, tagged
lines carrying an ``AnswerRecord`` and a ``ScoreResult`` per question. Each
``(model_id, prompt_id)`` pair found across the shards becomes one row of the
figure; repeated runs of one pair (distinct ``run_id``) are aggregated into that
row rather than split, and the row then reports ``runs n``.

**Point it at one dataset's shard directory** -- the sweep writes
``$DIMOS_EVAL_SHARD_DIR/<dataset>/``, and this is invoked once per such
directory. Rows are keyed by ``(model_id, prompt_id)`` and nothing else, so a
run over a directory holding two recordings' shards would otherwise silently
average one row's dots across two rooms, two question sets and two threshold
ranges. :func:`render_figure` refuses that input outright rather than plotting
it -- see :func:`dataset_of` for how a question id names its dataset.

The figure is a per-configuration strip of measured errors (one dot per
question that produced a goal, a marker at the median) annotated with
``n_pred / n_total``, the no-prediction rate and the pass rate -- the counts
matter as much as the dots, because a configuration that rarely answers can
otherwise look accurate. The rates divide by the agent-attributable results
only, so a row that lost questions to the harness also prints ``broken n``:
the denominator the percentages are over is never left to be guessed at.

The pass threshold is per question (``questions.THRESHOLD_MARGIN_M``), so
``--threshold-m`` draws one reference line rather than the line each dot was
scored against; pass or fail per dot is not readable off the figure and is not
meant to be. The verdicts live in the shards.

Output is checked against the repository's 75 KB large-file limit and the run
fails if the PNG exceeds it, so a sample figure can be committed alongside the
eval.
"""

from __future__ import annotations

import argparse
from collections import OrderedDict
from collections.abc import Sequence
import glob as globlib
from pathlib import Path
import statistics
import sys

from dimos.agents.evals.scorer import (
    ScoredCase,
    broken_count,
    errors_m,
    no_prediction_rate,
    pass_rate,
    read_shards,
)

# pyproject.toml [tool.largefiles] max_size_kb: anything committed must fit.
MAX_PNG_BYTES = 75 * 1024

# Deterministic vertical spread so repeated renders of the same shards are
# byte-comparable (a random jitter would churn the committed sample figure).
_JITTER_STEPS = (0.0, 0.16, -0.16, 0.08, -0.08, 0.24, -0.24)


def collect_shard_paths(patterns: Sequence[str]) -> list[Path]:
    """Expand files, directories and globs into a sorted list of shard files."""
    paths: list[Path] = []
    for pattern in patterns:
        candidate = Path(pattern)
        if candidate.is_dir():
            paths.extend(sorted(candidate.glob("*.jsonl")))
        elif candidate.is_file():
            paths.append(candidate)
        else:
            paths.extend(sorted(Path(match) for match in globlib.glob(pattern)))
    if not paths:
        raise SystemExit(f"no shard files matched {list(patterns)}")
    return paths


def dataset_of(question_id: str) -> str:
    """The dataset a question id belongs to: its first two hyphen-joined tokens.

    ``questions.py`` builds every id as ``<dataset slug>-<raw label slug>``,
    where the dataset slug is the recording's own name with its underscore
    turned into a hyphen -- ``go2_bigoffice`` -> ``go2-bigoffice-organization``,
    ``go2_short`` -> ``go2-short-houseplant``. The label slug is what varies in
    length and can itself contain hyphens, so the *prefix* is the readable end:
    two tokens, taken from the front.

    A blunt key on purpose. It is used only to refuse a mixed input, so it has
    to be derivable from the shard alone -- the shards carry question ids, not
    dataset names -- and it must not need the reference tree to be present.
    """
    return "-".join(question_id.split("-")[:2])


def assert_single_dataset(cases: Sequence[ScoredCase]) -> None:
    """Refuse an input whose question ids come from more than one dataset.

    The figure's x axis is a distance error against one recording's references,
    and its rows are keyed by ``(model_id, prompt_id)`` alone. Two recordings'
    shards in one input therefore do not produce a wrong-looking figure -- they
    produce a *plausible* one whose dots are pooled across two rooms, two
    question sets and two threshold ranges, with nothing on it saying so.

    Raises ``ValueError`` naming every dataset found, because the fix is to
    invoke the renderer once per ``$DIMOS_EVAL_SHARD_DIR/<dataset>/`` and the
    operator needs to know which ones they collapsed.
    """
    datasets = sorted({dataset_of(case.answer.question_id) for case in cases})
    if len(datasets) > 1:
        raise ValueError(
            f"the shards hold questions from {len(datasets)} datasets ({', '.join(datasets)}); "
            "one figure is one recording -- its x axis is a distance error against that "
            "recording's own references, so pooling them would average across rooms, "
            "question sets and threshold ranges. Render each "
            "$DIMOS_EVAL_SHARD_DIR/<dataset>/ directory separately."
        )


def group_by_configuration(
    cases: Sequence[ScoredCase],
) -> OrderedDict[tuple[str, str], list[ScoredCase]]:
    """Group scored cases by configuration, keyed by ``(model_id, prompt_id)``.

    ``run_id`` is deliberately *not* part of the key: repeating a configuration
    measures the same thing twice, so both runs belong on one row -- every error
    plotted, the rates over all of them, and the run count in the annotation.
    Splitting them into two rows would invite reading run-to-run noise as a
    difference between configurations.

    Insertion order is kept so the figure's rows follow the shard order the
    caller passed rather than an arbitrary hash order.
    """
    grouped: OrderedDict[tuple[str, str], list[ScoredCase]] = OrderedDict()
    for case in cases:
        grouped.setdefault((case.answer.model_id, case.answer.prompt_id), []).append(case)
    return grouped


def _summary(cases: Sequence[ScoredCase]) -> str:
    """One-line stats annotation for a configuration row.

    ``n_pred``/``n_total`` counts every case in the row -- across runs, so
    ``n_total`` is the question count times ``runs`` -- while the two rates
    divide by the agent-attributable ones among them. ``broken n`` names the
    difference, so a row whose percentages are computed over a shrunken sample
    says so; it is omitted when nothing broke, which is the normal case.
    ``runs n`` appears only when the row aggregates more than one, since a
    single-run row is the normal case and needs no qualifier.
    """
    results = [case.result for case in cases]
    n_pred = len(errors_m(results))
    n_broken = broken_count(results)
    n_runs = len({case.answer.run_id for case in cases})
    broken = f"  broken {n_broken}" if n_broken else ""
    runs = f"  runs {n_runs}" if n_runs > 1 else ""
    return (
        f"n_pred {n_pred}/{len(results)}  "
        f"no-prediction {no_prediction_rate(results):.0%}  "
        f"pass {pass_rate(results):.0%}{broken}{runs}"
    )


def render_figure(
    cases: Sequence[ScoredCase],
    out_path: Path,
    *,
    threshold_m: float | None = None,
    title: str = "Spatial goal selection: error distribution per configuration",
    dpi: int = 100,
) -> Path:
    """Draw the error-distribution figure and save it as a PNG.

    Raises ``ValueError`` if *cases* mix two recordings (:func:`assert_single_dataset`),
    or if the saved file exceeds :data:`MAX_PNG_BYTES`; the oversized file is
    removed rather than left behind for someone to commit.
    """
    # Before matplotlib is even imported: a mixed input is an operator error, and
    # the gate belongs here rather than in `main` so a programmatic caller cannot
    # route around it.
    assert_single_dataset(cases)

    import matplotlib

    matplotlib.use("Agg")  # headless: this runs in CI and over SSH
    import matplotlib.pyplot as plt

    grouped = group_by_configuration(cases)
    if not grouped:
        raise ValueError("no scored cases to plot")

    configurations = list(grouped)
    # Kept deliberately small: the whole figure has to stay under the
    # repository's 75 KB large-file limit so a sample can be committed.
    fig, ax = plt.subplots(figsize=(8.0, 1.2 * len(configurations) + 1.6))

    largest = 0.0
    for row, label in enumerate(configurations):
        results = [case.result for case in grouped[label]]
        values = errors_m(results)
        largest = max([largest, *values])
        ys = [row + _JITTER_STEPS[i % len(_JITTER_STEPS)] for i in range(len(values))]
        color = f"C{row % 10}"
        ax.plot(values, ys, "o", color=color, alpha=0.75, markersize=6, zorder=3)
        if values:
            median = statistics.median(values)
            ax.plot(
                [median],
                [row],
                marker="|",
                color=color,
                markersize=26,
                markeredgewidth=2.5,
                zorder=4,
            )
        ax.text(
            0.01,
            row + 0.42,
            _summary(grouped[label]),
            transform=ax.get_yaxis_transform(),
            fontsize=8.5,
            color="0.25",
        )

    upper = max(largest, threshold_m or 0.0) * 1.18 or 1.0
    if threshold_m is not None:
        ax.axvline(threshold_m, linestyle="--", linewidth=1.0, color="0.45", zorder=1)
        ax.text(
            threshold_m,
            -0.66,
            f" pass threshold {threshold_m:g} m",
            fontsize=8.5,
            color="0.45",
            va="top",
        )

    ax.set_yticks(range(len(configurations)))
    ax.set_yticklabels([f"{model}\n{prompt}" for model, prompt in configurations], fontsize=9)
    ax.set_ylim(-0.7, len(configurations) - 0.25)
    ax.set_xlim(0.0, upper)
    ax.invert_yaxis()
    ax.set_xlabel("goal error (m), map-frame XY")
    ax.set_title(title, fontsize=11)
    ax.grid(axis="x", alpha=0.3)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)

    size = out_path.stat().st_size
    if size > MAX_PNG_BYTES:
        out_path.unlink()
        raise ValueError(
            f"rendered figure is {size} bytes, over the {MAX_PNG_BYTES}-byte "
            f"large-file limit (removed {out_path}); lower --dpi or drop configurations"
        )
    return out_path


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--shards",
        nargs="+",
        required=True,
        metavar="PATH",
        help="shard files, directories of *.jsonl shards, or globs",
    )
    parser.add_argument("--out", type=Path, required=True, help="PNG path to write")
    parser.add_argument(
        "--threshold-m",
        type=float,
        default=None,
        help="draw the pass threshold used for scoring (metres)",
    )
    parser.add_argument("--dpi", type=int, default=100, help="figure resolution (default: 100)")
    args = parser.parse_args(argv)

    paths = collect_shard_paths(args.shards)
    cases = read_shards(paths)
    if not cases:
        raise SystemExit(f"no scored cases found in {len(paths)} shard file(s)")

    out = render_figure(cases, args.out, threshold_m=args.threshold_m, dpi=args.dpi)
    print(f"{out} ({out.stat().st_size} bytes) <- {len(cases)} cases from {len(paths)} shard(s)")
    for (model_id, prompt_id), group in group_by_configuration(cases).items():
        print(f"  {model_id} / {prompt_id}: {_summary(group)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
