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

Four conditions abort the run rather than emit a subtly broken question set:

* **Duplicate display names.** Two questions with the same wording have no
  single right answer, and renaming is how duplicates get introduced.
* **Two questions on one location group.** ``teacher.py`` flags references that
  sit on the same physical spot under different labels; asking about both would
  score the same place twice and mark an agent wrong for the name it did not
  guess. Review has to keep one and drop the rest.
* **A question no agent could pass.** The threshold is an observation
  envelope, not a point-accuracy tolerance (see :data:`THRESHOLD_MARGIN_M`), so
  a reference the teacher only ever saw from further away than the threshold is
  unpassable however well the agent behaves. Each threshold is now *derived*
  from the nearest viewpoint in its own reference row's ``robot_poses``, so this
  holds by construction below the cap; the check stays because it is the
  invariant the artifact is read under, and at the cap it can still fire.
* **Two questions one retrieval could answer.** Two adjacent objects can share
  a viewing position: if any pose the teacher saw *B* from lies inside *A*'s
  pass radius, an agent that retrieved *B* passes *A*, and *A* has stopped
  measuring whether the agent found *A*. This is the location-group rule one
  step out -- a group catches two names for one object, this catches two objects
  one viewpoint answers for -- and it is checked in both directions, because the
  two radii differ.

The question wording comes from one fixed template. The instruction about *how*
to answer belongs to the system prompt under test and is deliberately not part
of the question text: prompts get swept, and a swept answer-format instruction
would measure format compliance instead of spatial ability.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

from dimos.agents.evals.contracts import QuestionSpec

#: The only question wording. ``display_name`` is the reviewed name.
QUESTION_TEMPLATE = "Where is the {display_name}? Use your navigation tools to go to it."

VALID_STATUSES = ("verified", "renamed", "dropped")

#: Added to a question's own nearest teacher viewpoint to get its pass
#: threshold. The threshold is an *observation envelope*, not point accuracy:
#: the shipping ``navigate_with_text`` builds its goal from the *robot pose* of
#: the retrieved frame, so even a perfect retrieval lands about one viewing
#: distance away from the LiDAR-derived object centroid the reference holds.
#: "Passed" therefore means "the goal is about as close as the teacher itself
#: ever got", and the margin is the slack for a retrieval that picked a
#: different frame of the same object. It is per question because the viewing
#: distances are: a 3.5 m radius around an object the teacher stood 0.9 m from
#: is wide enough to swallow half the room, which is how two neighbouring
#: objects end up sharing an answer. The fine-grained signal is ``error_m``,
#: which the figure plots in full.
THRESHOLD_MARGIN_M = 0.5

#: Ceiling on the derived threshold: the teacher's own observation envelope
#: (``teacher.GateParams.max_range_m``, 3.2 m), rounded up. It sits *above* the
#: envelope, so an object measured at the furthest range the teacher accepts is
#: still passable, and *below* envelope-plus-margin, so it starts binding at a
#: 3.0 m viewing distance rather than never. Without a ceiling every question
#: would be passable by construction, including one about an object the robot
#: only ever saw from across the room -- the cap is what keeps the passability
#: check below able to fire at all.
MAX_THRESHOLD_M = 3.5


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


def nearest_viewpoint_m(ref: dict[str, Any]) -> float:
    """XY distance from a reference to the closest pose it was observed from.

    ``inf`` when the row records no poses at all, which is a row carrying no
    evidence that the object was ever reached.
    """
    return min(
        (math.hypot(pose[0] - ref["x"], pose[1] - ref["y"]) for pose in ref["robot_poses"]),
        default=float("inf"),
    )


def threshold_for(ref: dict[str, Any], margin_m: float, cap_m: float) -> float:
    """This reference's pass radius: its own nearest viewpoint plus *margin_m*.

    Capped at *cap_m*, and rounded to the precision the reference positions
    themselves carry so the artifact stays diffable.
    """
    return round(min(nearest_viewpoint_m(ref) + margin_m, cap_m), 4)


def confusability_problems(kept: list[tuple[QuestionSpec, dict[str, Any]]]) -> list[str]:
    """Report every ordered pair of questions one retrieval could answer for both.

    For questions *A* and *B*: if the teacher ever saw *B* from a pose inside
    *A*'s pass radius, then an agent that retrieves a *B* frame -- the wrong
    object -- sets a goal that passes *A*. *A* then measures the neighbourhood
    rather than the object, and no scored number can show it. Ordered pairs
    because the radii differ: *A* can be confusable with *B* without the reverse
    holding.

    Distance is map-frame XY, matching the scorer, and the reported number is the
    offending viewpoint's distance so a reviewer can see how far inside it sits.
    """
    problems: list[str] = []
    for spec, _ in kept:
        for other_spec, other_ref in kept:
            if other_spec.question_id == spec.question_id:
                continue
            nearest = min(
                (
                    math.hypot(pose[0] - spec.ref_x, pose[1] - spec.ref_y)
                    for pose in other_ref["robot_poses"]
                ),
                default=float("inf"),
            )
            if nearest <= spec.threshold_m:
                problems.append(
                    f"{spec.raw_label!r} is confusable with {other_spec.raw_label!r}: a "
                    f"viewpoint of {other_spec.raw_label!r} is {nearest:.2f} m from the "
                    f"{spec.raw_label!r} reference, inside its {spec.threshold_m} m pass "
                    "radius, so retrieving the wrong object would pass this question"
                )
    return problems


def build_questions(
    refs: list[dict[str, Any]],
    review: dict[str, dict[str, Any]],
    threshold_cap_m: float = MAX_THRESHOLD_M,
    margin_m: float = THRESHOLD_MARGIN_M,
) -> tuple[list[QuestionSpec], list[str], dict[str, int]]:
    """Apply *review* to *refs*.

    Returns the questions sorted by id, the list of problems that must abort the
    run, and the per-status counts. Problems are collected rather than raised at
    the first one, so a reviewer sees every offending label in one pass.

    Each question's ``threshold_m`` is its own: the nearest pose the teacher
    observed that object from, plus *margin_m*, capped at *threshold_cap_m* (see
    :data:`THRESHOLD_MARGIN_M`). A per-question radius is what keeps a question
    about an object the robot practically touched from also being satisfied by
    the shelf behind it.

    The last two checks are passability -- a question whose own nearest viewpoint
    is at or beyond its (capped) threshold could not be passed by any agent --
    and confusability across the kept set, which needs every threshold to exist
    first and therefore runs once at the end.
    """
    problems: list[str] = []
    counts = dict.fromkeys(VALID_STATUSES, 0)
    counts["unreviewed"] = 0

    questions: list[QuestionSpec] = []
    kept: list[tuple[QuestionSpec, dict[str, Any]]] = []
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
        # The id is the key every later artifact joins on -- shards, the
        # figure, longitudinal comparisons between runs -- so it is built from
        # the *detector* label, which no reviewer can change: renaming a
        # question in review.json changes its wording, not its identity. Two
        # labels that differ only in punctuation are still as bad as two
        # identical ones.
        question_id = f"{slugify(ref['dataset'])}-{slugify(raw_label)}"
        if question_id in kept_ids:
            problems.append(
                f"{raw_label!r} and {kept_ids[question_id]!r} both slugify to "
                f"question id {question_id!r}"
            )
            continue

        # The shipping goal is a viewpoint: `navigate_with_text` answers from
        # the robot pose of the retrieved frame, while the reference is a
        # LiDAR-derived object centroid. An object the teacher never got closer
        # to than the threshold therefore cannot be reached by any agent, and a
        # question set holding one reports a failure that is not the agent's.
        nearest = nearest_viewpoint_m(ref)
        threshold_m = threshold_for(ref, margin_m, threshold_cap_m)
        if nearest >= threshold_m:
            problems.append(
                f"{raw_label!r}: unpassable by construction: nearest teacher viewpoint "
                f"{nearest:.2f} m >= threshold {threshold_m} m"
            )
            continue

        kept_names[display_name] = raw_label
        kept_groups[group] = raw_label
        kept_ids[question_id] = raw_label

        spec = QuestionSpec(
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
        questions.append(spec)
        kept.append((spec, ref))

    questions.sort(key=lambda q: q.question_id)
    kept.sort(key=lambda pair: pair[0].question_id)
    problems.extend(confusability_problems(kept))
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
        "--threshold-margin-m",
        type=float,
        default=THRESHOLD_MARGIN_M,
        help=(
            "added to each question's own nearest teacher viewpoint to get the "
            "map-frame XY distance within which its goal counts as correct, so "
            "'passed' means the robot ends up about as close as the teacher did "
            "(the continuous signal is error_m)"
        ),
    )
    parser.add_argument(
        "--threshold-cap-m",
        type=float,
        default=MAX_THRESHOLD_M,
        help="ceiling on the derived per-question threshold (metres)",
    )
    return parser


def main(argv: list[str]) -> int:
    args = _build_parser().parse_args(argv)
    refs = load_refs(Path(args.refs))
    review = load_review(Path(args.review))

    questions, problems, counts = build_questions(
        refs, review, args.threshold_cap_m, args.threshold_margin_m
    )

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
    thresholds = sorted(q.threshold_m for q in questions)
    span = f"{thresholds[0]}-{thresholds[-1]} m" if thresholds else "n/a"
    print(
        f"reviewed:  {counts['verified']} verified, {counts['renamed']} renamed, "
        f"{counts['dropped']} dropped, {counts['unreviewed']} unreviewed (dropped)\n"
        f"threshold: {span} per question (nearest viewpoint + "
        f"{args.threshold_margin_m} m, capped at {args.threshold_cap_m} m)\n"
        f"questions: {len(questions)} -> {args.out}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
