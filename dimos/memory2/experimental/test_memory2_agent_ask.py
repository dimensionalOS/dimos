# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Opt-in end-to-end tests for `dimos.memory2.experimental.memory2_agent.ask`.

Runs the real LangChain agent against a recorded SQLite memory store and a
live OpenAI model. Marked `experimental` so it does NOT execute under the
default `pytest` invocation (see addopts in pyproject.toml). Run with:

    export OPENAI_API_KEY=...
    export MEMORY2_AGENT_DB=/path/to/recording.db
    pytest -m experimental dimos/memory2/experimental/ -v

Skips quietly when prerequisites are missing:
  - OPENAI_API_KEY (via `skipif_no_openai`)
  - the recording .db file pointed to by MEMORY2_AGENT_DB (required — no
    default; the content-grounded cases below were written against a
    specific recording, so the test will assert on its contents)

Important: Keep these tests, while this is experimental, out of the common test suite.
"""

from __future__ import annotations

from collections.abc import Callable
import os
from pathlib import Path
import re

import pytest

DEFAULT_MODEL = (
    "gpt-5.5"  # Matches `dimos/memory2/experimental/memory2_agent/ask.py`'s --model default.
)


# Cardinal-only — ordinals ("first", "second") would create false positives
# in answers like "the first room I saw …". Compound forms ("twenty-one")
# are out of scope; none of the recording's ground-truth answers need them.
_NUMBER_WORDS: dict[str, int] = {
    "zero": 0,
    "one": 1,
    "two": 2,
    "three": 3,
    "four": 4,
    "five": 5,
    "six": 6,
    "seven": 7,
    "eight": 8,
    "nine": 9,
    "ten": 10,
    "eleven": 11,
    "twelve": 12,
    "thirteen": 13,
    "fourteen": 14,
    "fifteen": 15,
    "sixteen": 16,
    "seventeen": 17,
    "eighteen": 18,
    "nineteen": 19,
    "twenty": 20,
    "thirty": 30,
    "forty": 40,
    "fifty": 50,
    "sixty": 60,
    "seventy": 70,
    "eighty": 80,
    "ninety": 90,
    "hundred": 100,
}

_AFFIRMATIVES = (
    "yes",
    "yeah",
    "yep",
    "yup",
    "sure",
    "correct",
    "affirmative",
    "indeed",
)
_NEGATIVES = ("no", "nope", "not", "never", "negative")


def _first_number(s: str) -> float | None:
    """Return the first number found in *s* as a float.

    Recognises digit forms (``80``, ``2.5``, ``-3``) and small English
    number words (``two``, ``eighty``) — whichever appears first wins, so
    "About eighty m²" → 80 and "2 rooms" → 2."""
    for tok in re.finditer(r"-?\d+(?:\.\d+)?|[A-Za-z]+", s):
        text = tok.group(0)
        if text[0].isdigit() or (text[0] == "-" and len(text) > 1 and text[1].isdigit()):
            return float(text)
        word = text.lower()
        if word in _NUMBER_WORDS:
            return float(_NUMBER_WORDS[word])
    return None


def _has_word(s: str, *words: str) -> bool:
    """Whole-word case-insensitive match for any of *words*."""
    text = s.lower()
    return any(re.search(rf"\b{re.escape(w.lower())}\b", text) for w in words)


def _is_affirmative(s: str) -> bool:
    """True if *s* reads as a yes-answer.

    Requires an affirmative word AND no negation, so "Yes" passes,
    "No, but yes for the kitchen" fails, "Not the same room" fails."""
    if _has_word(s, *_NEGATIVES):
        return False
    return _has_word(s, *_AFFIRMATIVES)


def _is_negative(s: str) -> bool:
    """True if *s* reads as a no-answer.

    Mirror of `_is_affirmative`: requires a negation word AND no
    affirmative, so "No" passes, "Yes" fails, "No, but yes for R2" fails."""
    if _has_word(s, *_AFFIRMATIVES):
        return False
    return _has_word(s, *_NEGATIVES)


# Cues that an answer declines to commit a measured result rather than
# fabricating one. Substring (not whole-word) matching: these phrases are
# specific enough that a confident measured answer is unlikely to trip them,
# and several carry apostrophes / compound forms ("can't", "no way to") that
# `_has_word` wouldn't catch.
_REFUSAL_CUES = (
    "can't",
    "cannot",
    "can not",
    "couldn't",
    "could not",
    "unable",
    "not able",
    "no way to",
    "not possible",
    "without inventing",
    "would be inventing",
    "fabricat",  # fabricate / fabricated / fabricating
    "too noisy",
    "not reliable",
    "no reliable",
    "not precise",
    "imprecise",
    "insufficient",
    "not measurable",
    "don't have",
    "do not have",
    "eyeball",  # "would be an eyeballed estimate, not a measurement"
)


def _declines_measurement(s: str) -> bool:
    """True if the answer declines to commit a measured result.

    The agent's tools can't return object-bounded metric geometry — lidar
    `near` yields point-cloud metadata, not points, and the map is a
    rendered image with no edge coordinates — so the honest response to a
    "measure the table and the door" question is to say it can't measure
    them reliably rather than invent numbers. See `table_fit_door_refuses`."""
    return any(cue in s.lower() for cue in _REFUSAL_CUES)


def _picked_choice_letter(s: str) -> str | None:
    """Return the LAST standalone A–D letter in *s*.

    Using the last occurrence is the robust pick when the agent echoes
    the option list before deciding ("A) … B) … C) … D) … Answer: B")."""
    matches = re.findall(r"\b([A-D])\b", s)
    return matches[-1] if matches else None


def _parse_xy(s: str) -> tuple[float, float] | None:
    """Extract the first two signed decimal numbers in *s* as an (x, y) pair.

    Tolerates parentheses, brackets, plus signs, whitespace — anything as
    long as two parseable numbers appear in order. Returns None if fewer
    than two numbers are found."""
    nums = re.findall(r"[-+]?\d+(?:\.\d+)?", s)
    if len(nums) < 2:
        return None
    return float(nums[0]), float(nums[1])


# Multi-choice with the right answer permuted to position B (avoiding the
# first-position bias some LLMs show without breaking determinism).
_MULTI_CHOICE = (
    "State which is true. Answer with just the letter (A, B, C, or D):\n"
    "A) There were trashcans behind both robots I saw.\n"
    "B) There were plants behind the first robot I saw, "
    "and a trashcan behind the second one I saw.\n"
    "C) There were plants behind the second robot I saw, "
    "and a trashcan behind the first one I saw.\n"
    "D) There were plants behind both robots I saw."
)


@pytest.fixture(scope="module")
def recording_db() -> Path:
    raw = os.environ.get("MEMORY2_AGENT_DB")
    if not raw:
        pytest.skip("MEMORY2_AGENT_DB not set; point it at the recording .db")
    db = Path(raw)
    if not db.exists():
        pytest.skip(f"recording not at {db}; set MEMORY2_AGENT_DB to an existing .db")
    return db


@pytest.fixture(scope="module")
def store(recording_db: Path):
    from dimos.memory2.store.sqlite import SqliteStore

    s = SqliteStore(path=str(recording_db))
    yield s
    s.stop()


@pytest.fixture(scope="module")
def recording_db_hongkong() -> Path:
    """Path to the Hong Kong office recording. Must be supplied via the
    MEMORY2_AGENT_DB_HONGKONG env var — no default, tests skip otherwise.
    The Hong Kong recording is longer and richer than go2_short.db; the
    content-grounded cases below are bound to its specific layout
    (elevator room, white robots seen, total floor area)."""
    raw = os.environ.get("MEMORY2_AGENT_DB_HONGKONG")
    if not raw:
        pytest.skip("MEMORY2_AGENT_DB_HONGKONG not set; point it at the Hong Kong .db")
    db = Path(raw)
    if not db.exists():
        pytest.skip(f"recording not at {db}; set MEMORY2_AGENT_DB_HONGKONG to an existing .db")
    return db


@pytest.fixture(scope="module")
def store_hongkong(recording_db_hongkong: Path):
    from dimos.memory2.store.sqlite import SqliteStore

    s = SqliteStore(path=str(recording_db_hongkong))
    yield s
    s.stop()


@pytest.fixture(scope="module")
def clip():
    # CLIP weights take seconds to load — share across the module's tests.
    from dimos.models.embedding.clip import CLIPModel

    return CLIPModel()


def _model() -> str:
    return os.environ.get("MEMORY2_AGENT_MODEL", DEFAULT_MODEL)


@pytest.mark.experimental
@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
def test_lists_streams(store, clip) -> None:
    """The agent should call `list_streams` to orient itself and report
    the four streams produced by `build_memory.py`."""
    from dimos.memory2.experimental.memory2_agent.agent import run_question

    res = run_question(
        store,
        clip,
        "How many streams does this memory store have?",
        model=_model(),
    )
    assert res.error is None, res.error
    called = {tc["name"] for tc in res.tool_calls}
    assert "list_streams" in called
    assert "4" in res.final_answer


@pytest.mark.experimental
@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
def test_visual_question_uses_image_tool(store, clip) -> None:
    """Spatial/visual questions should drive the agent into a tool that
    returns image content (`show_image`, `recall_view`, `walkthrough`,
    `show_map`, `frames_facing`). Confirms the langgraph Command path is
    end-to-end functional — the agent both invokes the tool AND produces
    a non-empty grounded answer afterwards."""
    from dimos.memory2.experimental.memory2_agent.agent import run_question

    res = run_question(
        store,
        clip,
        "At t=22s show me what the robot saw directly forward and describe it in one sentence.",
        model=_model(),
    )
    assert res.error is None, res.error
    image_tools = {"show_image", "recall_view", "walkthrough", "show_map", "frames_facing"}
    called = {tc["name"] for tc in res.tool_calls}
    assert called & image_tools, f"no image tool was called; got {called}"
    assert res.final_answer.strip()


# Content-grounded questions about the go2_short.db recording. Each case
# pairs the prompt with a check that runs on the agent's final answer.
# Add cases here as the recording's verified ground-truth coverage grows.
# Each prompt ends with an explicit format directive so the agent commits
# a clean, parseable final answer (single number / word / letter) instead
# of dumping its full reasoning chain into `submit_final_answer`. The
# system prompt also tells the agent to use `note()` for intermediate
# work, but in practice gpt-4.1-mini ignores that without a format nudge
# at the question level.
_QA_CASES: list[tuple[str, str, Callable[[str], bool]]] = [
    (
        "rooms_count_2",
        "How many rooms are there? Reply with only the number, nothing else.",
        lambda ans: "2" in ans.split(),
    ),
    (
        "biggest_room_area_~80m2",
        "What's the area of the biggest room? Reply with only the number in m², nothing else.",
        # Ground truth ~80 m²; accept ±20% to absorb measurement noise.
        # Range check, so keep `_first_number` here (contains-style doesn't
        # express a range).
        lambda ans: (n := _first_number(ans)) is not None and 64 <= n <= 96,
    ),
    (
        "start_equals_end_room",
        "Did you start in the same room as you ended? Reply with only yes or no, nothing else.",
        _is_affirmative,
    ),
    (
        "closest_to_meeting_table_2m",
        "What's the closest distance in meters that you got to the long "
        "meeting table? Round to whole numbers no decimals. Reply with "
        "only the number, nothing else.",
        # Ground truth: ≤ 2 m. 1 m is acceptable; 3 m is not.
        lambda ans: (n := _first_number(ans)) is not None and n <= 2,
    ),
    (
        "white_robots_count_2",
        "How many white robots did you pass by? Reply with only the number, nothing else.",
        lambda ans: "2" in ans.split(),
    ),
    (
        "white_robots_distance_apart",
        "What's the approximate straight-line distance in meters between "
        "the two white robots (not walking distance — the real distance, "
        "even across walls)? Round to a whole number. Reply with only "
        "the number, nothing else.",
        # Ground truth: the two robots are between 3 m and 6 m apart (inclusive).
        lambda ans: (n := _first_number(ans)) is not None and 3 <= n <= 6,
    ),
    (
        "man_in_black_moved_hand",
        "What did the man in black move at the end? Reply with only the "
        "single body part, nothing else.",
        # Accept either "hand" or "finger" (or plurals via substring).
        lambda ans: any(w in ans.lower() for w in ("hand", "finger")),
    ),
    (
        "multi_choice_letter_B",
        _MULTI_CHOICE,
        lambda ans: _picked_choice_letter(ans) == "B",
    ),
    (
        "exploration_waypoint_roi",
        "What's the highest-ROI waypoint to explore next to expand the map? "
        "Reply with only the coordinate in the format `x, y` (two numbers "
        "separated by a comma), nothing else.",
        # Ground truth: ~(+4.2, +9.0), the east-lobe frontier. ±1.5 m on each
        # axis to absorb the agent's choice between nearby frontier cells.
        lambda ans: (
            (xy := _parse_xy(ans)) is not None
            and abs(xy[0] - 4.2) <= 1.5
            and abs(xy[1] - 9.0) <= 1.5
        ),
    ),
    (
        "passed_through_doorway_top_left",
        "Where is the doorway you passed through that's at the top-left of "
        "your trajectory? Reply with only the coordinate in the format "
        "`x, y` (two numbers separated by a comma), nothing else.",
        # Ground truth: ~(-2.0, +9.1), the interior doorway at the upper-left
        # bend of the trajectory loop. ±1.5 m on each axis.
        lambda ans: (
            (xy := _parse_xy(ans)) is not None
            and abs(xy[0] - (-2.0)) <= 1.5
            and abs(xy[1] - 9.1) <= 1.5
        ),
    ),
]


@pytest.mark.experimental
@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.parametrize(
    "question,verify",
    [(q, v) for _id, q, v in _QA_CASES],
    ids=[case_id for case_id, _q, _v in _QA_CASES],
)
def test_short_recording_qa(
    store,
    clip,
    question: str,
    verify: Callable[[str], bool],
) -> None:
    """Content-grounded QA over `go2_short.db`. Each case asserts a known
    fact about the recording; see `_QA_CASES` for the truth table."""
    from dimos.memory2.experimental.memory2_agent.agent import run_question

    res = run_question(store, clip, question, model=_model())
    assert res.error is None, res.error
    assert verify(res.final_answer), f"unexpected answer for {question!r}: {res.final_answer!r}"


# Content-grounded QA over `go2_hongkong_office.db` — the longer recording
# of the Hong Kong office. Discovery-permissive bounds for the questions
# whose ground-truth coordinate / area we haven't pinned down yet; tighten
# once the agent's first answers are reviewed.
#
# A case is `(id, question, verify)` with an optional 4th element carrying a
# pytest mark (e.g. `pytest.mark.skip(...)`) for cases whose ground truth is
# not pinned down yet.
_HKCase = (
    tuple[str, str, Callable[[str], bool]]
    | tuple[str, str, Callable[[str], bool], pytest.MarkDecorator]
)
_QA_CASES_HONGKONG: list[_HKCase] = [
    (
        "white_robots_count_4_hk",
        "How many white robots did you pass by? Reply with only the number, nothing else.",
        # Ground truth: 4 white robots in the recording. 3 is acceptable
        # because one of them is barely visible in only two frames, so the
        # agent reasonably under-counts it.
        lambda ans: (n := _first_number(ans)) is not None and n in (3, 4),
    ),
    (
        "musical_instruments_last_5min_yes",
        "Did you see any musical instruments in the last five minutes? "
        "Reply with only yes or no, nothing else.",
        # Ground truth: yes — an acoustic guitar on a retail shelf is visible
        # at t~303-312s (see `acoustic_guitar_closest_robot_distance`), a
        # ~10s pass near (-1, +21). The recording is ~557s, so the last five
        # minutes start at ~t=257s and the sighting falls comfortably inside.
        # NOTE: "four minutes" (window from ~t=317s) would EXCLUDE it — the
        # guitar is seen ~5s before that window opens — so a four-minute
        # phrasing should expect "no", not "yes".
        _is_affirmative,
    ),
    (  # Flakey - this task is challening for the harness
        "acoustic_guitar_closest_robot_distance",
        "What's the straight-line distance in meters between the acoustic "
        "guitar and the closest robot to it? Round to a whole number. "
        "Reply with only the number, nothing else.",
        # Ground truth: ~11 m (manual triangulation places the acoustic
        # guitar near (-2, +22) and the closest white service robot near
        # (-5.7, +10.8), giving ~11.8 m). ±2 m to absorb the agent's
        # depth-estimation noise from a short-baseline frame cluster.
        lambda ans: (n := _first_number(ans)) is not None and 9 <= n <= 13,
        # xfail (non-strict): short-baseline depth triangulation is noisy.
        # The true distance is ~11 m (corroborated independently), but the
        # agent often lands just outside the 9-13 band (e.g. 14). Record the
        # miss without failing the suite; a lucky in-band run still XPASSes.
        pytest.mark.xfail(
            reason="noisy short-baseline depth triangulation; agent lands "
            "near ~11 m but often just outside the 9-13 band",
            strict=False,
        ),
    ),
    (
        "elevator_room_center",
        "What's the center coordinate of the room with the elevators? "
        "Reply with only the coordinate in the format `x, y` (two numbers "
        "separated by a comma), nothing else.",
        # Ground truth: ~(+4.55, +2.22) — at the boundary between the lower
        # central corridor (R3) and the right connector (R2). ±1.5 m on
        # each axis to absorb the agent's choice between adjacent room
        # centroids in that area.
        lambda ans: (
            (xy := _parse_xy(ans)) is not None
            and abs(xy[0] - 4.55) <= 1.5
            and abs(xy[1] - 2.22) <= 1.5
        ),
    ),
    (  # Flakey - this number changes a lot and I don't have a proper measurement
        "total_floor_area",
        "What's the total floor area of the office, summed across all "
        "rooms, in square meters? Reply with only the number, nothing else.",
        # Ground truth: ~400 m² (eyeballed). ±100 m² to absorb the agent's
        # variance in polygon tightness and whether corridors get counted.
        lambda ans: (n := _first_number(ans)) is not None and 300 <= n <= 500,
    ),
    (
        "tennis_court_no_room_fits",
        "Could we have enough space for a standard singles tennis court in "
        "any of the rooms if we removed everything in it? If no room fits, "
        "say no. Otherwise, say which one.",
        # Ground truth: no. The singles playing lines are 23.77 m x 8.23 m
        # (~196 m²), but an actual ITF court needs ~36.58 m x 18.29 m including
        # run-off (6.40 m behind each baseline, 3.66 m beyond each sideline) —
        # larger than the whole office footprint (~24.5 m x 28 m). Even the bare
        # lines don't fit any single room: the largest isolated room measured
        # here is ~3.4 m x 8 m, and the entire connected floor is just ~289 m²
        # (sprawling, not a clear rectangle). So no room comes close.
        lambda ans: (
            _has_word(ans, "no", "none", "not", "never", "negative")
            and not _has_word(ans, *_AFFIRMATIVES)
        ),
    ),
    (
        "desks_need_cable_management",
        "How many desks need cable management? Reply with only the number, nothing else.",
        # Placeholder verify: just asserts the agent commits a number. Ground
        # truth (the desk count + an acceptable tolerance) is not pinned yet;
        # review the recording, set the bound, then drop the skip mark below.
        lambda ans: _first_number(ans) is not None,
        pytest.mark.skip(reason="ground truth not pinned yet; review recording to set the count"),
    ),
    (
        "table_fit_door_refuses",
        "For the table the man is sitting on, could we realistically fit it "
        "through the nearest door without lifting it from the ground - just "
        "by rotating it about a vertical axis and pushing it? Answer this "
        "QUANTITATIVELY, do not eyeball: (1) measure the table's minimum "
        "(narrow) horizontal dimension in meters, including any leg/base "
        "footprint; (2) locate the relevant doorway and measure its clear "
        "opening width in meters; (3) since the table stays flat on the "
        "floor, rotating about a vertical axis only changes which horizontal "
        "dimension faces the opening, so compare the table's narrow "
        "dimension against the door clear width and give a yes/no with the "
        "margin in cm. Use the spatial tools (position_of_thing, "
        "measure_distance, lidar, the map) to derive actual numbers. Report "
        "your measured numbers and their uncertainty, then the verdict.",
        # Expected behavior: refusal. Asked to MEASURE — without being handed
        # an explicit bearing-triangulation recipe — the agent has no way to
        # get object-bounded metric widths: lidar `near` returns point-cloud
        # metadata, not points, and the map is a rendered image with no edge
        # coordinates. We accept this answer (the refusal) since our
        # experimental units make very noisy measurements: the honest move is
        # to decline rather than commit a cm margin off noisy data. Spelling
        # out the triangulation method instead does get committed numbers,
        # but that's a different, prescriptive prompt.
        _declines_measurement,
    ),
    (
        "longest_time_room_is_a_lounge",
        "What room did you spend the longest time in? Reply with a short description of the room.",
        # Ground truth: a lounge / sofa seating area. Two such zones dominate
        # the walk — a left lounge/meeting area (~21% of dwell) and a right
        # lounge/reception (~19%) — within sampling noise of each other, and
        # which one wins depends on where the agent draws the partition
        # boundary between them, so accept either. Verified by integrating
        # odom dwell time over the agent's verified room polygons (left 118s
        # vs right 107s of 558s total; next room down is ~93s). Don't assert
        # left-vs-right specifically — that flips run to run.
        lambda ans: _has_word(ans, "lounge", "sofa", "sofas", "seating", "reception", "showroom"),
    ),
    (
        "mixed_use_office_and_retail",
        "Is this place a regular office, a retail store, or both? "
        "Reply with one of: office, store, or both.",
        # Ground truth: both. The walk passes office desks, meeting rooms and
        # lounges AND a stocked retail section (snack/drink aisles with price
        # labels). An agent that noticed only the offices, or only the store,
        # fails — the answer requires synthesizing the whole walkthrough.
        lambda ans: _has_word(ans, "both")
        or (_has_word(ans, "office") and _has_word(ans, "store", "retail", "shop")),
    ),
    (
        "lounge_closer_to_elevators_than_store",
        "Which did you pass closer to the elevators — the lounge with the "
        "sofas, or the retail store with the snack aisles? "
        "Reply with only 'lounge' or 'store'.",
        # Ground truth: the lounge. Measured from the agent's own verified room
        # polygons, the lounge/reception centroids sit ~6 m from the elevator
        # lobby; the retail aisles ~17 m — a ~2.5x separation, so the A-vs-B
        # comparison is robust to partition noise.
        lambda ans: _has_word(ans, "lounge"),
    ),
    (
        "greeting_to_guitar_avoids_elevators_no",
        "Is it possible to go from the room with the greeting robot to the "
        "room with the acoustic guitar without passing by any elevators? "
        "Reply with only yes or no, nothing else.",
        # Ground truth: no — the route between the greeting-robot room and the
        # acoustic-guitar room runs through the elevator lobby, so there's no
        # way around it.
        _is_negative,
    ),
    (
        "elevator_to_pantry_avoids_white_robots_yes",
        "Is it possible to go from the elevator room to the pantry/shelves "
        "room without passing by the room with the several white robots? "
        "Reply with only yes or no, nothing else.",
        # Ground truth: yes — there's a route from the elevator room to the
        # pantry/shelves room that does not pass through the white-robots room.
        _is_affirmative,
    ),
    (
        "failed_nav_attempt_place",
        "Is there a place you tried to go to but couldn't? If no, say no. If "
        "yes, reply with only the coordinate in the format `x, y` (two "
        "numbers separated by a comma).",
        # Placeholder verify: just asserts the answer is well-formed per the
        # prompt — either a negative, or a parseable coordinate. Ground truth
        # (whether a nav attempt actually failed in this recording, and its
        # coordinate) is not pinned yet; review the recording, set the bound,
        # then drop the skip mark below.
        lambda ans: _is_negative(ans) or _parse_xy(ans) is not None,
        pytest.mark.skip(
            reason="ground truth not pinned yet; review recording to confirm "
            "whether a nav attempt failed and, if so, its coordinate"
        ),
    ),
    (
        "nonstanding_robot_on_bed_or_dog",
        "Did you see the robot that is not standing? What is it on?",
        # Ground truth (confirmed by frame inspection): at ~t197s, world
        # ~(-2.0, +4.5), beside the glass meeting pods, a white/grey quadruped
        # robot DOG lies folded (non-standing) on a dark-blue cushion / dog
        # BED on the floor. A correct answer mentions the bed it rests on or
        # that the robot is a dog. Substring match so "dog bed", "beds",
        # "robot dog" all count.
        lambda ans: any(w in ans.lower() for w in ("bed", "dog")),
        # xfail: the agent currently MISSES this. Asked for "the robot that is
        # not standing," it fixates on the white wheeled service robots (which
        # are upright on wheels) or the west office area and answers "on the
        # floor," never finding the dog-on-bed. The real target is small,
        # low-contrast and at the frame edge, so CLIP under-ranks it (~0.30,
        # no better than unrelated frames) and the agent doesn't retrieve it.
        # Non-strict (the default): gpt-5.5 is nondeterministic and may
        # occasionally stumble onto it, which should surface as XPASS rather
        # than fail the suite. Remove this mark once retrieval/localization of
        # small edge-of-frame objects improves enough to find it reliably.
        pytest.mark.xfail(
            reason="agent fixates on the wheeled service robots and misses the "
            "small, edge-of-frame robot dog resting on a dog bed at ~(-2,+4.5)"
        ),
    ),
    (
        "looked_into_glass_room_not_entered",
        "Were you able to look clearly into a room that you didn't walk into? "
        "If no, say no. If yes, briefly name the room and give its "
        "approximate center coordinate in the format `x, y`.",
        # Ground truth: yes — a glass-walled meeting/office room at ~(+15.5,
        # -2.5) that the robot passes alongside in the corridor (nearest
        # trajectory approach ~1.6 m at ~t84s) but never enters, the pods being
        # sealed glass. This prompt is UNDER-DETERMINED: the recording has
        # several "seen through glass but not entered" rooms, and across three
        # runs the agent named three different ones — (6,-3.5) kitchenette,
        # (13.2,-1.2), and (15.5,-2.5). We pin the last as the ground truth.
        lambda ans: (
            (xy := _parse_xy(ans)) is not None
            and abs(xy[0] - 15.5) <= 1.5
            and abs(xy[1] - (-2.5)) <= 1.5
        ),
        # Non-strict xfail ("fail optional"): because the question is
        # under-determined the agent lands on a different room most runs, so
        # this bound usually misses (XFAIL) but a matching run still XPASSes —
        # neither breaks the suite. It documents the expected answer without
        # asserting the agent reliably reproduces it.
        pytest.mark.xfail(
            reason="under-determined prompt: agent names a different "
            "seen-but-not-walked room most runs; pinned to the (15.5,-2.5) "
            "glass meeting room as ground truth",
            strict=False,
        ),
    ),
]


@pytest.mark.experimental
@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.parametrize(
    "question,verify",
    [
        pytest.param(case[1], case[2], id=case[0], marks=case[3] if len(case) > 3 else ())
        for case in _QA_CASES_HONGKONG
    ],
)
def test_hongkong_recording_qa(
    store_hongkong,
    clip,
    question: str,
    verify: Callable[[str], bool],
) -> None:
    """Content-grounded QA over the Hong Kong office recording. See
    `_QA_CASES_HONGKONG` for the truth table."""
    from dimos.memory2.experimental.memory2_agent.agent import run_question

    res = run_question(store_hongkong, clip, question, model=_model())
    assert res.error is None, res.error
    assert verify(res.final_answer), f"unexpected answer for {question!r}: {res.final_answer!r}"
