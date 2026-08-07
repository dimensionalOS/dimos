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
The last two tests re-derive *every committed* question set from its own
reference table and review overlay, so no shipped artifact can drift apart from
the code that made it, and check that two recordings' question ids stay
disjoint. They are parametrized over ``reference_sets.dataset_names()``, so a
new recording is covered by committing its directory.
"""

from __future__ import annotations

from typing import Any

import pytest

from dimos.agents.evals.contracts import QuestionSpec
from dimos.agents.evals.questions import (
    MAX_THRESHOLD_M,
    QUESTION_TEMPLATE,
    THRESHOLD_MARGIN_M,
    build_questions,
    load_refs,
    load_review,
    nearest_viewpoint_m,
    slugify,
    threshold_for,
)
from dimos.agents.evals.reference_sets import dataset_dir, dataset_names, load_question_set
from dimos.agents.evals.teacher import GateParams


def make_ref(raw_label: str, location_group: str, **overrides: Any) -> dict[str, Any]:
    """One ``refs.jsonl`` row, with only the fields ``build_questions`` reads.

    ``robot_poses`` defaults to two viewpoints 0.6 m and 0.9 m from wherever
    the reference is, so a row is passable at any threshold worth testing;
    tests about the passability gate itself pass their own.
    """
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
    ref.setdefault(
        "robot_poses",
        [[ref["x"] + 0.6, ref["y"], 0.31], [ref["x"], ref["y"] + 0.9, 0.31]],
    )
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
            # make_ref's nearest viewpoint is 0.6 m away, plus the 0.5 m margin.
            threshold_m=0.6 + THRESHOLD_MARGIN_M,
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
    assert question.question_id == "go2-bigoffice-bookstore"
    assert question.human_review == "renamed"
    assert "liquor shelf" in question.question_text


def test_a_rename_changes_the_wording_but_never_the_id() -> None:
    """The id is what a later run joins on, so review may not be able to move it.

    Renaming a question in ``review.json`` is a routine, low-ceremony edit --
    somebody sharpens "liquor shelf" to "backlit bottle shelf" months later. If
    that rewrote the id, every shard, figure and comparison recorded under the
    old one would quietly stop lining up, and nothing would report it.
    """
    ref = [make_ref("bookstore", "loc-03")]
    first, _, _ = build_questions(
        ref, {"bookstore": {"status": "renamed", "display_name": "liquor shelf"}}, 1.5
    )
    renamed, problems, _ = build_questions(
        ref, {"bookstore": {"status": "renamed", "display_name": "backlit bottle shelf"}}, 1.5
    )

    assert not problems
    assert first[0].question_id == renamed[0].question_id == "go2-bigoffice-bookstore"
    assert first[0].question_text != renamed[0].question_text


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


def test_labels_that_differ_only_in_punctuation_abort_the_run() -> None:
    """The id is what every later artifact joins on, so a slug clash is fatal.

    Two questions cannot share one id even when the detector's own vocabulary
    is what collided; the reviewer has to drop one.
    """
    questions, problems, _ = build_questions(
        [make_ref("window sill", "loc-18"), make_ref("window-sill", "loc-12")],
        {
            "window sill": {"status": "verified"},
            "window-sill": {"status": "verified"},
        },
        1.5,
    )
    assert [q.raw_label for q in questions] == ["window sill"]
    assert problems == [
        "'window-sill' and 'window sill' both slugify to question id 'go2-bigoffice-window-sill'"
    ]


def test_every_problem_is_reported_in_one_pass() -> None:
    """A reviewer fixes the overlay once, not once per offending label."""
    _, problems, _ = build_questions(
        [make_ref("bookstore", "loc-03"), make_ref("shelf", "loc-17")],
        {"bookstore": {"status": "renamed"}, "shelf": {"status": "maybe"}},
        1.5,
    )
    assert len(problems) == 2


def test_questions_are_sorted_by_id_and_carry_a_threshold_each() -> None:
    """Ids sort; thresholds do not have to agree, and here deliberately do not.

    The three objects are 10 m apart so the confusability gate has nothing to
    say about them, and seen from 0.6 m, 1.5 m and 2.5 m respectively -- which is
    the normal case, not a contrived one: a robot does not keep its distance
    constant, and that is why one radius for the whole set cannot be right.
    """
    questions, problems, _ = build_questions(
        [
            make_ref("window sill", "loc-18", x=0.0, y=0.0),
            make_ref("elevator door", "loc-06", x=10.0, y=0.0, robot_poses=[[11.5, 0.0, 0.31]]),
            make_ref("houseplant", "loc-09", x=20.0, y=0.0, robot_poses=[[22.5, 0.0, 0.31]]),
        ],
        {
            "window sill": {"status": "verified"},
            "elevator door": {"status": "verified"},
            "houseplant": {"status": "verified"},
        },
    )
    assert not problems
    assert [q.question_id for q in questions] == [
        "go2-bigoffice-elevator-door",
        "go2-bigoffice-houseplant",
        "go2-bigoffice-window-sill",
    ]
    assert [q.threshold_m for q in questions] == [2.0, 3.0, 1.1]


def test_the_threshold_cap_covers_the_teachers_observation_envelope() -> None:
    """The cap is the envelope, and the envelope is the teacher's own.

    ``max_range_m`` is the furthest the teacher accepted a measurement from, so
    it is also the furthest a correct viewpoint-shaped goal can legitimately land
    from a reference. A cap below it would fail questions on the geometry of the
    recording rather than on anything the agent did. Sitting below
    envelope-plus-margin is the other half: that is what makes the cap bind at a
    3.0 m viewing distance instead of never, and an unpassable question therefore
    reachable at all.
    """
    envelope = GateParams().max_range_m
    assert envelope < MAX_THRESHOLD_M < envelope + THRESHOLD_MARGIN_M


def test_a_question_no_agent_could_reach_aborts_the_run() -> None:
    """The scored goal is a viewpoint, so an object only seen from far away is unpassable.

    ``navigate_with_text`` answers with the robot pose of the retrieved frame,
    and the reference is the object's own centroid. If the teacher never got
    within the threshold of the object, a perfect retrieval still scores as a
    failure -- and the run would report the recording's geometry as the model's
    fault. The generator refuses to emit such a question at all.

    Since a threshold is derived from that same viewpoint, this can only happen
    at the cap: the cap is passed as 1.5 m here, well below the 2.9 m the
    2.4 m viewpoint would otherwise have earned.
    """
    questions, problems, _ = build_questions(
        [make_ref("elevator door", "loc-06", robot_poses=[[3.4, 2.0, 0.31], [1.0, 5.2, 0.31]])],
        {"elevator door": {"status": "verified"}},
        1.5,
    )
    assert questions == []
    assert problems == [
        "'elevator door': unpassable by construction: nearest teacher viewpoint "
        "2.40 m >= threshold 1.5 m"
    ]


def test_the_same_question_is_passable_once_the_threshold_follows_the_viewpoint() -> None:
    """Same reference, same viewpoints: it is the threshold that decides.

    This is the shape of the bug the gate exists to catch -- with a cap this
    tight the question is unpassable, and with the threshold derived from the
    2.4 m viewpoint it is exactly passable -- so the two halves are asserted
    together rather than one in isolation.
    """
    refs = [make_ref("elevator door", "loc-06", robot_poses=[[3.4, 2.0, 0.31], [1.0, 5.2, 0.31]])]
    review = {"elevator door": {"status": "verified"}}

    questions, problems, _ = build_questions(refs, review)
    assert problems == []
    assert [q.question_id for q in questions] == ["go2-bigoffice-elevator-door"]
    assert questions[0].threshold_m == pytest.approx(2.4 + THRESHOLD_MARGIN_M, abs=1e-3)


def test_a_reference_with_no_viewpoints_at_all_is_unpassable() -> None:
    """A row with no ``robot_poses`` carries no evidence that it was ever reached."""
    _, problems, _ = build_questions(
        [make_ref("houseplant", "loc-09", robot_poses=[])],
        {"houseplant": {"status": "verified"}},
    )
    assert len(problems) == 1
    assert "unpassable by construction" in problems[0]


# Per-question thresholds.


def test_threshold_is_the_nearest_viewpoint_plus_the_margin() -> None:
    """The radius follows the evidence for that object, and nothing else."""
    ref = make_ref("houseplant", "loc-09", robot_poses=[[3.0, 2.0, 0.31], [1.0, 3.4, 0.31]])
    assert nearest_viewpoint_m(ref) == pytest.approx(1.4)
    assert threshold_for(ref, THRESHOLD_MARGIN_M, MAX_THRESHOLD_M) == pytest.approx(1.9)


def test_the_threshold_is_capped_however_far_away_the_robot_stayed() -> None:
    """Past the cap the radius stops growing, which is what lets a question fail.

    A radius that kept following the viewpoint would make every question
    passable by construction, including one about an object the robot only ever
    saw from across the room -- which is the case that says more about the
    recording than about any agent.
    """
    far = make_ref("elevator door", "loc-06", robot_poses=[[9.0, 2.0, 0.31]])
    assert nearest_viewpoint_m(far) == pytest.approx(8.0)
    assert threshold_for(far, THRESHOLD_MARGIN_M, MAX_THRESHOLD_M) == MAX_THRESHOLD_M


def test_a_row_with_no_viewpoints_gets_the_cap_and_then_fails_passability() -> None:
    """``inf + margin`` must not escape into a spec; it becomes the cap and is caught."""
    empty = make_ref("houseplant", "loc-09", robot_poses=[])
    assert threshold_for(empty, THRESHOLD_MARGIN_M, MAX_THRESHOLD_M) == MAX_THRESHOLD_M


# Confusable questions.


def test_two_objects_sharing_a_viewing_position_abort_the_run() -> None:
    """A radius wide enough to reach the neighbour stops measuring this object.

    The two references are 1.2 m apart, and each was observed from 1.5 m away on
    the far side of the other -- so each object's viewpoint is 0.3 m from its
    neighbour's reference, well inside the neighbour's 2.0 m radius. An agent that
    retrieved the *wrong* one would set a goal that passes both questions. Both
    directions are reported, because a reviewer has to see that neither question
    survives as it stands rather than being told to fix one of them.
    """
    refs = [
        make_ref("houseplant", "loc-09", x=0.0, y=0.0, robot_poses=[[1.5, 0.0, 0.31]]),
        make_ref("shelf", "loc-17", x=1.2, y=0.0, robot_poses=[[-0.3, 0.0, 0.31]]),
    ]
    review = {"houseplant": {"status": "verified"}, "shelf": {"status": "verified"}}

    questions, problems, _ = build_questions(refs, review)

    # The specs are still built -- the gate reports, the caller aborts.
    assert [q.raw_label for q in questions] == ["houseplant", "shelf"]
    assert len(problems) == 2
    assert problems[0] == (
        "'houseplant' is confusable with 'shelf': a viewpoint of 'shelf' is 0.30 m "
        "from the 'houseplant' reference, inside its 2.0 m pass radius, so retrieving "
        "the wrong object would pass this question"
    )
    assert problems[1].startswith("'shelf' is confusable with 'houseplant'")


def test_objects_far_enough_apart_are_not_confusable() -> None:
    """The gate is about geometry, not about how many questions there are."""
    refs = [
        make_ref("houseplant", "loc-09", x=0.0, y=0.0, robot_poses=[[1.5, 0.0, 0.31]]),
        make_ref("shelf", "loc-17", x=9.0, y=0.0, robot_poses=[[7.5, 0.0, 0.31]]),
    ]
    review = {"houseplant": {"status": "verified"}, "shelf": {"status": "verified"}}

    questions, problems, _ = build_questions(refs, review)
    assert problems == []
    assert len(questions) == 2


def test_confusability_is_reported_per_direction() -> None:
    """One radius can swallow a neighbour while the neighbour's does not reach back.

    ``far post`` was seen from 3 m, so its radius is the 3.5 m cap and reaches
    ``near post``'s viewpoint 3.4 m away; ``near post`` was seen from 0.4 m, so
    its radius is 0.9 m and reaches nothing -- ``far post``'s own viewpoint is
    4.2 m off to the side. Only the first question is broken, and reporting the
    pair symmetrically would send a reviewer to drop the wrong one.
    """
    refs = [
        make_ref("far post", "loc-01", x=0.0, y=0.0, robot_poses=[[0.0, 3.0, 0.31]]),
        make_ref("near post", "loc-02", x=3.0, y=0.0, robot_poses=[[3.4, 0.0, 0.31]]),
    ]
    review = {"far post": {"status": "verified"}, "near post": {"status": "verified"}}

    _, problems, _ = build_questions(refs, review)
    assert len(problems) == 1
    assert problems[0].startswith("'far post' is confusable with 'near post'")


def test_a_dropped_label_cannot_make_another_question_confusable() -> None:
    """Only kept questions are compared: a dropped object is never retrieved *as* one.

    This is what makes the gate actionable -- dropping one of a confusable pair
    resolves it -- and it is why the check runs after review rather than over the
    whole reference table.
    """
    refs = [
        make_ref("houseplant", "loc-09", x=0.0, y=0.0, robot_poses=[[1.5, 0.0, 0.31]]),
        make_ref("shelf", "loc-17", x=1.2, y=0.0, robot_poses=[[2.7, 0.0, 0.31]]),
    ]
    review = {
        "houseplant": {"status": "verified"},
        "shelf": {"status": "dropped", "note": "class-ambiguous"},
    }

    questions, problems, _ = build_questions(refs, review)
    assert problems == []
    assert [q.raw_label for q in questions] == ["houseplant"]


def test_slugify_collapses_runs_of_punctuation() -> None:
    assert slugify("Row of White Bottles") == "row-of-white-bottles"
    assert slugify("  liquor / shelf  ") == "liquor-shelf"
    assert slugify("gpt-5.6-luna") == "gpt-5-6-luna"


@pytest.mark.parametrize("dataset", dataset_names())
def test_committed_question_set_is_what_the_committed_inputs_produce(dataset: str) -> None:
    """Every shipped question set is exactly what its own inputs produce.

    ``reference/<dataset>/questions.jsonl`` is what the live sweep reads, so it
    must stay exactly what that dataset's ``refs.jsonl`` plus ``review.json``
    produce -- an artifact edited by hand, or left behind by a change to the
    rules above, would move every score without anything failing. That includes
    the thresholds, which are derived per question and so cannot be eyeballed.

    The assertions are invariants rather than a frozen question count, and the
    parametrization comes from discovery rather than from a list: adding a
    recording is adding a directory, and a K that had to be declared here would
    be a second place to forget it. The per-dataset counts the PR text and the
    figures quote are recorded in ``reference/README.md``, next to the funnel
    they came out of.
    """
    directory = dataset_dir(dataset)
    refs = load_refs(directory / "refs.jsonl")
    review = load_review(directory / "review.json")
    questions, problems, _ = build_questions(refs, review)
    committed = load_question_set(dataset)

    assert problems == []
    assert questions == committed
    assert committed, f"{dataset!r} ships an empty question set"
    assert len({q.question_id for q in committed}) == len(committed)
    assert len({q.display_name for q in committed}) == len(committed)


def test_question_ids_are_unique_across_every_committed_dataset() -> None:
    """No two recordings can produce the same question id.

    ``question_id`` is ``<dataset slug>-<raw label slug>``, and it is the join
    key everywhere downstream: shard lines, the figure's rows, and any
    longitudinal comparison across runs. Two datasets sharing one -- a second
    recording of the same room, or a dataset name that slugifies onto another's
    -- would silently merge two different objects into one series. Nothing in
    ``build_questions`` can catch that, because it only ever sees one table.
    """
    seen: dict[str, str] = {}
    for dataset in dataset_names():
        for question in load_question_set(dataset):
            collision = seen.setdefault(question.question_id, dataset)
            assert collision == dataset, (
                f"question id {question.question_id!r} is shipped by both "
                f"{collision!r} and {dataset!r}"
            )
