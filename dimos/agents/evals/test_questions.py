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

"""How the human review overlay turns reference rows into questions.

Everything a reviewer can get wrong is checked here, because the failure mode
is silent: a question set with two names for one shelf, or with a name the
reviewer never actually looked at, still runs and still produces a number.
The last test re-derives the committed question set from the committed
reference table and review overlay, so the shipped artifacts cannot drift
apart from the code that made them.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.agents.evals.contracts import QuestionSpec
from dimos.agents.evals.questions import (
    DEFAULT_THRESHOLD_M,
    QUESTION_TEMPLATE,
    build_questions,
    load_refs,
    load_review,
    slugify,
)

REFERENCE_DIR = Path(__file__).parent / "reference"

#: The number of questions the shipped table holds. Frozen because the figure,
#: the PR's reported counts and the sweep's runtime are all quoted against it.
EXPECTED_QUESTION_COUNT = 6


def make_ref(raw_label: str, location_group: str, **overrides: Any) -> dict[str, Any]:
    """One ``refs.jsonl`` row, with only the fields ``build_questions`` reads."""
    ref: dict[str, Any] = {
        "dataset": "go2_bigoffice",
        "raw_label": raw_label,
        "location_group": location_group,
        "x": 1.0,
        "y": 2.0,
        "z": 0.3,
        "n_independent_views": 2,
        "spread_m": 0.25,
    }
    ref.update(overrides)
    return ref


def test_question_template_is_frozen() -> None:
    """The wording is part of the measurement; changing it is a deliberate act.

    Answer-format instructions live in the swept system prompts, never here --
    a swept format instruction would measure format compliance instead of
    spatial ability -- so this asserts the literal text, not just that it
    interpolates.
    """
    assert (
        QUESTION_TEMPLATE == "Where is the {display_name}? Use your navigation tools to go to it."
    )

    questions, problems, _ = build_questions(
        [make_ref("houseplant", "loc-01")], {"houseplant": {"status": "verified"}}, 1.5
    )
    assert not problems
    assert questions[0].question_text == (
        "Where is the houseplant? Use your navigation tools to go to it."
    )


def test_verified_label_becomes_a_question_under_its_own_name() -> None:
    questions, problems, counts = build_questions(
        [make_ref("houseplant", "loc-01", x=-0.2125, y=0.6, z=0.55, spread_m=0.125)],
        {"houseplant": {"status": "verified", "note": "potted plant, unique in the scene"}},
        1.5,
    )
    assert not problems
    assert counts["verified"] == 1
    assert questions == [
        QuestionSpec(
            question_id="go2-bigoffice-houseplant",
            display_name="houseplant",
            raw_label="houseplant",
            question_text="Where is the houseplant? Use your navigation tools to go to it.",
            ref_x=-0.2125,
            ref_y=0.6,
            ref_z=0.55,
            n_views=2,
            spread_m=0.125,
            human_review="verified",
            threshold_m=1.5,
        )
    ]


def test_renamed_label_is_asked_under_the_new_name_and_keeps_the_old_one() -> None:
    """The detector's word is what makes the rename auditable afterwards."""
    questions, problems, counts = build_questions(
        [make_ref("bookstore", "loc-03")],
        {"bookstore": {"status": "renamed", "display_name": "liquor shelf"}},
        1.5,
    )
    assert not problems
    assert counts["renamed"] == 1
    question = questions[0]
    assert question.display_name == "liquor shelf"
    assert question.raw_label == "bookstore"
    assert question.question_id == "go2-bigoffice-liquor-shelf"
    assert question.human_review == "renamed"
    assert "liquor shelf" in question.question_text


def test_a_label_the_overlay_never_mentions_is_dropped() -> None:
    """Silence is not approval: unreviewed labels are the wall-and-light-streak ones."""
    questions, problems, counts = build_questions(
        [make_ref("houseplant", "loc-01"), make_ref("dance floor", "loc-02")],
        {"houseplant": {"status": "verified"}},
        1.5,
    )
    assert not problems
    assert counts["unreviewed"] == 1
    assert [q.raw_label for q in questions] == ["houseplant"]


def test_dropped_label_is_counted_but_never_asked() -> None:
    _, problems, counts = build_questions(
        [make_ref("dance floor", "loc-02")],
        {"dance floor": {"status": "dropped", "note": "floor region, not an object"}},
        1.5,
    )
    assert not problems
    assert counts["dropped"] == 1


def test_rename_without_a_new_name_is_a_problem() -> None:
    questions, problems, _ = build_questions(
        [make_ref("bookstore", "loc-03")], {"bookstore": {"status": "renamed"}}, 1.5
    )
    assert questions == []
    assert problems == ["'bookstore': status 'renamed' needs a display_name"]


def test_verified_cannot_quietly_rename() -> None:
    """`verified` means "the label is right", so it may not also change it."""
    questions, problems, _ = build_questions(
        [make_ref("window sill", "loc-18")],
        {"window sill": {"status": "verified", "display_name": "row of white bottles"}},
        1.5,
    )
    assert questions == []
    assert len(problems) == 1
    assert "cannot also set display_name" in problems[0]


def test_unknown_status_is_a_problem() -> None:
    questions, problems, _ = build_questions(
        [make_ref("shelf", "loc-17")], {"shelf": {"status": "maybe"}}, 1.5
    )
    assert questions == []
    assert problems == ["'shelf': status 'maybe' is not one of ('verified', 'renamed', 'dropped')"]


def test_two_labels_renamed_to_the_same_thing_abort_the_run() -> None:
    """Two identically worded questions have no single right answer."""
    questions, problems, _ = build_questions(
        [make_ref("bookstore", "loc-03"), make_ref("wine cooler", "loc-19")],
        {
            "bookstore": {"status": "renamed", "display_name": "liquor shelf"},
            "wine cooler": {"status": "renamed", "display_name": "liquor shelf"},
        },
        1.5,
    )
    assert [q.raw_label for q in questions] == ["bookstore"]
    assert problems == [
        "display name 'liquor shelf' is claimed by both 'bookstore' and 'wine cooler'"
    ]


def test_two_labels_on_one_location_group_abort_the_run() -> None:
    """One physical object under two names would be scored twice and missed once."""
    questions, problems, _ = build_questions(
        [make_ref("podium", "loc-14"), make_ref("speaker", "loc-14")],
        {
            "podium": {"status": "verified"},
            "speaker": {"status": "verified"},
        },
        1.5,
    )
    assert [q.raw_label for q in questions] == ["podium"]
    assert problems == [
        "'speaker' and 'podium' are the same location (loc-14); "
        "review must keep at most one of them"
    ]


def test_names_that_differ_only_in_punctuation_abort_the_run() -> None:
    """The id is what every later artifact joins on, so a slug clash is fatal."""
    questions, problems, _ = build_questions(
        [make_ref("bookstore", "loc-03"), make_ref("organization", "loc-12")],
        {
            "bookstore": {"status": "renamed", "display_name": "liquor shelf"},
            "organization": {"status": "renamed", "display_name": "liquor-shelf"},
        },
        1.5,
    )
    assert [q.display_name for q in questions] == ["liquor shelf"]
    assert problems == [
        "'liquor-shelf' and 'liquor shelf' both slugify to question id 'go2-bigoffice-liquor-shelf'"
    ]


def test_every_problem_is_reported_in_one_pass() -> None:
    """A reviewer fixes the overlay once, not once per offending label."""
    _, problems, _ = build_questions(
        [make_ref("bookstore", "loc-03"), make_ref("shelf", "loc-17")],
        {"bookstore": {"status": "renamed"}, "shelf": {"status": "maybe"}},
        1.5,
    )
    assert len(problems) == 2


def test_questions_are_sorted_by_id_and_carry_the_threshold() -> None:
    questions, problems, _ = build_questions(
        [
            make_ref("window sill", "loc-18"),
            make_ref("elevator door", "loc-06"),
            make_ref("houseplant", "loc-09"),
        ],
        {
            "window sill": {"status": "verified"},
            "elevator door": {"status": "verified"},
            "houseplant": {"status": "verified"},
        },
        2.0,
    )
    assert not problems
    assert [q.question_id for q in questions] == [
        "go2-bigoffice-elevator-door",
        "go2-bigoffice-houseplant",
        "go2-bigoffice-window-sill",
    ]
    assert {q.threshold_m for q in questions} == {2.0}


def test_slugify_collapses_runs_of_punctuation() -> None:
    assert slugify("Row of White Bottles") == "row-of-white-bottles"
    assert slugify("  liquor / shelf  ") == "liquor-shelf"
    assert slugify("gpt-5.6-luna") == "gpt-5-6-luna"


def test_committed_question_set_is_what_the_committed_inputs_produce() -> None:
    """The shipped artifacts are re-derivable, and the set is still K=6.

    ``reference/questions.jsonl`` is what the live sweep reads, so it must stay
    exactly what ``refs.jsonl`` plus ``review.json`` produce -- an artifact
    edited by hand, or left behind by a change to the rules above, would move
    every score without anything failing.
    """
    refs = load_refs(REFERENCE_DIR / "refs.jsonl")
    review = load_review(REFERENCE_DIR / "review.json")
    questions, problems, _ = build_questions(refs, review, DEFAULT_THRESHOLD_M)
    committed = [
        QuestionSpec.from_json(line)
        for line in (REFERENCE_DIR / "questions.jsonl").read_text().splitlines()
        if line.strip()
    ]

    assert problems == []
    assert questions == committed
    assert len(committed) == EXPECTED_QUESTION_COUNT
    assert len({q.question_id for q in committed}) == EXPECTED_QUESTION_COUNT
    assert len({q.display_name for q in committed}) == EXPECTED_QUESTION_COUNT
