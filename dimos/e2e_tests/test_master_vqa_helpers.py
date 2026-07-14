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

"""Fast, no-infra unit tests for master_vqa_test.py's pure helpers.

The full test_master_vqa (mujoco + real OpenAI + a running robot) is slow and
environment-heavy; these tests cover the answer-extraction logic it depends
on -- parsing the question template and picking the right message out of a
turn's /agent stream -- with plain synthetic data.
"""

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
import pytest

from dimos.e2e_tests.master_vqa_test import (
    QUESTIONS_FILE,
    RECORDED_QUESTIONS_FILE,
    aggregate_usage,
    decode_agent_messages,
    final_answer,
    load_questions,
    token_usage,
    tool_calls_used,
)


def test_load_questions_skips_blank_and_comment_lines(tmp_path) -> None:
    path = tmp_path / "questions.txt"
    path.write_text(
        "# a header comment\n"
        "\n"
        "What do you see?\n"
        "   \n"
        "# another comment\n"
        "Where is the chair?\n"
    )

    assert load_questions(path) == ["What do you see?", "Where is the chair?"]


def test_load_questions_strips_surrounding_whitespace(tmp_path) -> None:
    path = tmp_path / "questions.txt"
    path.write_text("  What do you see?  \n")

    assert load_questions(path) == ["What do you see?"]


def test_bundled_question_templates_are_nonempty_and_well_formed() -> None:
    for path in (QUESTIONS_FILE, RECORDED_QUESTIONS_FILE):
        questions = load_questions(path)

        assert len(questions) >= 5, path
        for question in questions:
            assert question, f"load_questions leaked a blank line in {path}"
            assert not question.startswith("#"), f"load_questions leaked a comment line in {path}"


def test_recorded_question_set_has_no_ego_now_questions() -> None:
    """The recorded scenarios replay a tour -- there is no meaningful current
    robot position, so 'right now' obstacle/clearance questions belong only
    in the live/sim set."""
    recorded = " ".join(load_questions(RECORDED_QUESTIONS_FILE)).lower()
    for phrase in ("right now", "ahead of you", "your left", "behind you"):
        assert phrase not in recorded, f"ego-now phrase {phrase!r} in recorded question set"


def test_final_answer_picks_last_ai_message_with_content() -> None:
    messages = [
        HumanMessage(content="what do you see?"),
        AIMessage(content="", tool_calls=[{"name": "list_observed_items", "args": {}, "id": "1"}]),
        ToolMessage(content="Observing 2 item(s): ...", tool_call_id="1"),
        AIMessage(content="I see a chair and a backpack."),
    ]

    assert final_answer(messages) == "I see a chair and a backpack."


def test_final_answer_empty_when_no_ai_content() -> None:
    messages = [
        HumanMessage(content="what do you see?"),
        AIMessage(content="", tool_calls=[{"name": "list_observed_items", "args": {}, "id": "1"}]),
        ToolMessage(content="Observing 0 item(s).", tool_call_id="1"),
    ]

    assert final_answer(messages) == ""


def test_decode_agent_messages_round_trips_pickled_langchain_messages() -> None:
    import pickle

    originals = [HumanMessage(content="hi"), AIMessage(content="hello")]
    raw = [pickle.dumps(m) for m in originals]

    decoded = decode_agent_messages(raw)

    assert [type(m) for m in decoded] == [HumanMessage, AIMessage]
    assert decoded[1].content == "hello"


def test_tool_calls_used_lists_every_tool_name_in_order() -> None:
    messages = [
        AIMessage(content="", tool_calls=[{"name": "detect", "args": {}, "id": "1"}]),
        ToolMessage(content="Detected 1 object(s): ...", tool_call_id="1"),
        AIMessage(content="", tool_calls=[{"name": "locate", "args": {}, "id": "2"}]),
        ToolMessage(content="Located 'chair': ...", tool_call_id="2"),
        AIMessage(content="Done."),
    ]

    assert tool_calls_used(messages) == ["detect", "locate"]


def test_tool_calls_used_empty_when_agent_never_calls_a_tool() -> None:
    messages = [AIMessage(content="I already know the answer.")]

    assert tool_calls_used(messages) == []


def _ai(inp: int, out: int, model: str = "gpt-4o-2024-08-06") -> AIMessage:
    return AIMessage(
        content="",
        usage_metadata={"input_tokens": inp, "output_tokens": out, "total_tokens": inp + out},
        response_metadata={"model_name": model},
    )


def test_token_usage_sums_every_model_call_in_a_turn() -> None:
    # A tool-calling turn spans several model calls; usage must be summed.
    usage = token_usage([_ai(1000, 50), ToolMessage(content="...", tool_call_id="1"), _ai(1200, 80)])

    assert usage["input_tokens"] == 2200
    assert usage["output_tokens"] == 130
    assert usage["total_tokens"] == 2330
    assert usage["model"] == "gpt-4o-2024-08-06"
    # gpt-4o: 2200/1e6*2.50 + 130/1e6*10.00 = 0.0055 + 0.0013
    assert usage["cost_usd"] == pytest.approx(0.0068)


def test_token_usage_cost_none_for_unknown_model() -> None:
    usage = token_usage([_ai(100, 10, model="some-local-llm")])

    assert usage["total_tokens"] == 110
    assert usage["cost_usd"] is None


def test_token_usage_handles_messages_without_usage_metadata() -> None:
    # Intermediate tool-call AIMessages / plain messages carry no usage and no
    # model name, so there's nothing to total and no basis to price.
    usage = token_usage([AIMessage(content="hi"), ToolMessage(content="x", tool_call_id="1")])

    assert usage["total_tokens"] == 0
    assert usage["cost_usd"] is None


def test_aggregate_usage_totals_across_questions() -> None:
    results = [
        {"tokens": token_usage([_ai(1000, 50)])},
        {"tokens": token_usage([_ai(2000, 100)])},
    ]

    agg = aggregate_usage(results)

    assert agg["input_tokens"] == 3000
    assert agg["output_tokens"] == 150
    assert agg["total_tokens"] == 3150
    assert agg["questions"] == 2
    assert agg["model"] == "gpt-4o-2024-08-06"
    # 3000/1e6*2.50 + 150/1e6*10.00 = 0.0075 + 0.0015
    assert agg["cost_usd"] == pytest.approx(0.009)
