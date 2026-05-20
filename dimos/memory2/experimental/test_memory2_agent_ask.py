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
_QA_CASES_HONGKONG: list[tuple[str, str, Callable[[str], bool]]] = [
    (
        "white_robots_count_2_hk",
        "How many white robots did you pass by? Reply with only the number, nothing else.",
        lambda ans: "2" in ans.split(),
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
    (
        "total_floor_area",
        "What's the total floor area of the office, summed across all "
        "rooms, in square meters? Reply with only the number, nothing else.",
        # Ground truth: ~400 m² (eyeballed). ±100 m² to absorb the agent's
        # variance in polygon tightness and whether corridors get counted.
        lambda ans: (n := _first_number(ans)) is not None and 300 <= n <= 500,
    ),
]


@pytest.mark.experimental
@pytest.mark.skipif_in_ci
@pytest.mark.skipif_no_openai
@pytest.mark.parametrize(
    "question,verify",
    [(q, v) for _id, q, v in _QA_CASES_HONGKONG],
    ids=[case_id for case_id, _q, _v in _QA_CASES_HONGKONG],
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
