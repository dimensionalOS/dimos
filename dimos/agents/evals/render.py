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
        --shards path/to/shards/ --out error_distribution.png --threshold-m 1.5

``--shards`` takes any mix of files, directories (every ``*.jsonl`` inside) and
globs. Shards are the per-case JSONL files written by
``dimos.agents.evals.scorer.append_shard`` -- one file per pytest case, tagged
lines carrying an ``AnswerRecord`` and a ``ScoreResult`` per question. Each
``(model_id, prompt_id)`` pair found across the shards becomes one row of the
figure.

The figure is a per-configuration strip of measured errors (one dot per
question that produced a goal, a marker at the median) annotated with
``n_pred / n_total``, the no-prediction rate and the pass rate -- the counts
matter as much as the dots, because a configuration that rarely answers can
otherwise look accurate.

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
import sys

from dimos.agents.evals.scorer import (
    ScoredCase,
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


def group_by_configuration(
    cases: Sequence[ScoredCase],
) -> OrderedDict[tuple[str, str], list[ScoredCase]]:
    """Group scored cases by configuration, keyed by ``(model_id, prompt_id)``.

    Insertion order is kept so the figure's rows follow the shard order the
    caller passed rather than an arbitrary hash order.
    """
    grouped: OrderedDict[tuple[str, str], list[ScoredCase]] = OrderedDict()
    for case in cases:
        grouped.setdefault((case.answer.model_id, case.answer.prompt_id), []).append(case)
    return grouped


def _summary(cases: Sequence[ScoredCase]) -> str:
    """One-line stats annotation for a configuration row."""
    results = [case.result for case in cases]
    n_pred = len(errors_m(results))
    return (
        f"n_pred {n_pred}/{len(results)}  "
        f"no-prediction {no_prediction_rate(results):.0%}  "
        f"pass {pass_rate(results):.0%}"
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

    Raises ``ValueError`` if the saved file exceeds :data:`MAX_PNG_BYTES`; the
    oversized file is removed rather than left behind for someone to commit.
    """
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
            median = sorted(values)[len(values) // 2]
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
