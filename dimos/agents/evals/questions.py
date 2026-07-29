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

"""Turns reviewed reference rows into the question set the evals run.

``teacher.py`` produces positions that are geometrically sound but semantically
raw: an open-vocabulary detector calls a backlit bottle shelf a "bookstore", and
a question built straight from that label would measure the detector's
vocabulary rather than the agent. A human therefore reviews the crops and
records a verdict per detector label in ``review.json``::

    {"bookstore": {"status": "renamed", "display_name": "liquor shelf",
                   "note": "backlit bottle display; label was wrong"}}

``status`` is ``verified`` (ask about it under its own label), ``renamed`` (ask
about it under ``display_name``) or ``dropped`` (never ask). A label the overlay
does not mention is **dropped**: silence is not approval, and an unreviewed
label is exactly the kind that turns out to be a wall or a light streak.

Two conditions abort the run rather than emit a subtly broken question set:

* **Duplicate display names.** Two questions with the same wording have no
  single right answer, and renaming is how duplicates get introduced.
* **Two questions on one location group.** ``teacher.py`` flags references that
  sit on the same physical spot under different labels; asking about both would
  score the same place twice and mark an agent wrong for the name it did not
  guess. Review has to keep one and drop the rest.

The question wording comes from one fixed template. The instruction about *how*
to answer belongs to the system prompt under test and is deliberately not part
of the question text: prompts get swept, and a swept answer-format instruction
would measure format compliance instead of spatial ability.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

from dimos.agents.evals.contracts import QuestionSpec

#: The only question wording. ``display_name`` is the reviewed name.
QUESTION_TEMPLATE = "Where is the {display_name}? Use your navigation tools to go to it."

VALID_STATUSES = ("verified", "renamed", "dropped")
DEFAULT_THRESHOLD_M = 1.5


def load_refs(path: Path) -> list[dict[str, Any]]:
    """Read ``refs.jsonl`` in file order."""
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def load_review(path: Path) -> dict[str, dict[str, Any]]:
    """Read the human review overlay, keyed by detector label."""
    review = json.loads(path.read_text())
    if not isinstance(review, dict):
        raise ValueError(f"{path}: expected an object mapping raw_label -> verdict")
    return review


def slugify(text: str) -> str:
    """Lowercase *text* into a filename-safe slug (runs of other chars -> ``-``)."""
    slug = "".join(c if c.isalnum() else "-" for c in text.lower())
    return "-".join(part for part in slug.split("-") if part)


def build_questions(
    refs: list[dict[str, Any]], review: dict[str, dict[str, Any]], threshold_m: float
) -> tuple[list[QuestionSpec], list[str], dict[str, int]]:
    """Apply *review* to *refs*.

    Returns the questions sorted by id, the list of problems that must abort the
    run, and the per-status counts. Problems are collected rather than raised at
    the first one, so a reviewer sees every offending label in one pass.
    """
    problems: list[str] = []
    counts = dict.fromkeys(VALID_STATUSES, 0)
    counts["unreviewed"] = 0

    questions: list[QuestionSpec] = []
    kept_groups: dict[str, str] = {}
    kept_names: dict[str, str] = {}
    kept_ids: dict[str, str] = {}

    for ref in refs:
        raw_label = ref["raw_label"]
        verdict = review.get(raw_label)
        if verdict is None:
            counts["unreviewed"] += 1
            continue

        status = verdict.get("status")
        if status not in VALID_STATUSES:
            problems.append(f"{raw_label!r}: status {status!r} is not one of {VALID_STATUSES}")
            continue
        counts[status] += 1
        if status == "dropped":
            continue

        display_name = verdict.get("display_name") or raw_label
        if status == "renamed" and not verdict.get("display_name"):
            problems.append(f"{raw_label!r}: status 'renamed' needs a display_name")
            continue
        if status == "verified" and verdict.get("display_name", raw_label) != raw_label:
            problems.append(
                f"{raw_label!r}: status 'verified' cannot also set display_name "
                f"{verdict['display_name']!r} — use 'renamed'"
            )
            continue

        if display_name in kept_names:
            problems.append(
                f"display name {display_name!r} is claimed by both "
                f"{kept_names[display_name]!r} and {raw_label!r}"
            )
            continue
        group = ref["location_group"]
        if group in kept_groups:
            problems.append(
                f"{raw_label!r} and {kept_groups[group]!r} are the same location ({group}); "
                "review must keep at most one of them"
            )
            continue
        # The id is the key every later artifact joins on, so two names that
        # differ only in punctuation are as bad as two identical names.
        question_id = f"{slugify(ref['dataset'])}-{slugify(display_name)}"
        if question_id in kept_ids:
            problems.append(
                f"{display_name!r} and {kept_ids[question_id]!r} both slugify to "
                f"question id {question_id!r}"
            )
            continue
        kept_names[display_name] = raw_label
        kept_groups[group] = raw_label
        kept_ids[question_id] = display_name

        questions.append(
            QuestionSpec(
                question_id=question_id,
                display_name=display_name,
                raw_label=raw_label,
                question_text=QUESTION_TEMPLATE.format(display_name=display_name),
                ref_x=ref["x"],
                ref_y=ref["y"],
                ref_z=ref["z"],
                n_views=ref["n_independent_views"],
                spread_m=ref["spread_m"],
                human_review=status,
                threshold_m=threshold_m,
            )
        )

    questions.sort(key=lambda q: q.question_id)
    return questions, problems, counts


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m dimos.agents.evals.questions",
        description=(
            "Apply a human review overlay to a reference table and emit the "
            "question set. Labels the overlay does not mention are dropped."
        ),
    )
    parser.add_argument("--refs", required=True, help="refs.jsonl written by teacher.py")
    parser.add_argument("--review", required=True, help="review.json: raw_label -> verdict")
    parser.add_argument("--out", required=True, help="questions.jsonl to write")
    parser.add_argument(
        "--threshold-m",
        type=float,
        default=DEFAULT_THRESHOLD_M,
        help="map-frame XY distance within which a goal counts as correct",
    )
    return parser


def main(argv: list[str]) -> int:
    args = _build_parser().parse_args(argv)
    refs = load_refs(Path(args.refs))
    review = load_review(Path(args.review))

    questions, problems, counts = build_questions(refs, review, args.threshold_m)

    unmatched = sorted(set(review) - {ref["raw_label"] for ref in refs})
    if unmatched:
        print(
            f"warning: {len(unmatched)} reviewed label(s) are not in this reference table "
            f"and were ignored: {', '.join(repr(label) for label in unmatched)}",
            file=sys.stderr,
        )

    if problems:
        print(
            f"error: the review overlay is inconsistent ({len(problems)} problems):",
            file=sys.stderr,
        )
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1

    Path(args.out).write_text("".join(f"{q.to_json()}\n" for q in questions))
    print(
        f"reviewed:  {counts['verified']} verified, {counts['renamed']} renamed, "
        f"{counts['dropped']} dropped, {counts['unreviewed']} unreviewed (dropped)\n"
        f"questions: {len(questions)} -> {args.out}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
