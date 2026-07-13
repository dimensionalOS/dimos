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

from dimos.e2e_tests.master_vqa_test import (
    QUESTIONS_FILE,
    decode_agent_messages,
    final_answer,
    load_questions,
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


def test_bundled_question_template_is_nonempty_and_well_formed() -> None:
    questions = load_questions(QUESTIONS_FILE)

    assert len(questions) >= 5
    for question in questions:
        assert question, "load_questions leaked a blank line"
        assert not question.startswith("#"), "load_questions leaked a comment line"


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
