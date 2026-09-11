# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

from collections.abc import Sequence

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from langchain_core.messages.base import BaseMessage
import pytest

from dimos.agents.compaction import (
    COMPACTION_MARKER,
    DEFAULT_CONTEXT_WINDOW,
    compact_history,
    estimate_tokens,
    make_model_summarizer,
    resolve_context_window,
)


def _human(text: str) -> HumanMessage:
    return HumanMessage(content=text)


def _ai(text: str) -> AIMessage:
    return AIMessage(content=text)


def _ai_tool_call(name: str, call_id: str, args: dict | None = None) -> AIMessage:
    return AIMessage(
        content="",
        tool_calls=[{"name": name, "args": args or {}, "id": call_id, "type": "tool_call"}],
    )


def _tool_result(text: str, call_id: str) -> ToolMessage:
    return ToolMessage(content=text, tool_call_id=call_id)


def _long_history(turns: int, chars: int = 400) -> list[BaseMessage]:
    history: list[BaseMessage] = []
    for i in range(turns):
        history.append(_human(f"request {i} " + "x" * chars))
        history.append(_ai(f"response {i} " + "y" * chars))
    return history


class TestResolveContextWindow:
    def test_known_models(self) -> None:
        assert resolve_context_window("gpt-4o") == 128_000
        assert resolve_context_window("gpt-4o-mini") == 128_000
        assert resolve_context_window("o3-mini") == 200_000

    def test_longest_prefix_wins(self) -> None:
        # "gpt-4.1" must not be resolved by the shorter "gpt-4" entry.
        assert resolve_context_window("gpt-4.1") == 1_047_576
        assert resolve_context_window("gpt-4") == 8_192

    def test_strips_provider_prefix(self) -> None:
        assert resolve_context_window("ollama:llama3.1:8b") == 131_072

    def test_case_insensitive(self) -> None:
        assert resolve_context_window("GPT-4O") == 128_000

    def test_unknown_and_empty_fall_back_to_default(self) -> None:
        assert resolve_context_window("some-new-model") == DEFAULT_CONTEXT_WINDOW
        assert resolve_context_window(None) == DEFAULT_CONTEXT_WINDOW
        assert resolve_context_window("") == DEFAULT_CONTEXT_WINDOW


class TestEstimateTokens:
    def test_empty_history_is_free(self) -> None:
        assert estimate_tokens([]) == 0

    def test_grows_with_content(self) -> None:
        small = estimate_tokens([_human("hi")])
        large = estimate_tokens([_human("hi " * 1000)])
        assert large > small

    def test_counts_every_message(self) -> None:
        one = estimate_tokens([_human("a" * 400)])
        two = estimate_tokens([_human("a" * 400), _human("a" * 400)])
        assert two > one

    def test_counts_tool_call_arguments(self) -> None:
        bare = estimate_tokens([_ai_tool_call("navigate", "1")])
        with_args = estimate_tokens(
            [_ai_tool_call("navigate", "1", {"destination": "kitchen " * 100})]
        )
        assert with_args > bare

    def test_images_are_charged_heavily(self) -> None:
        image_msg = HumanMessage(
            content=[
                {"type": "text", "text": "what is this?"},
                {"type": "image_url", "image_url": {"url": "data:image/png;base64,AAAA"}},
            ]
        )
        assert estimate_tokens([image_msg]) > 1_000


class TestCompactHistoryNoOp:
    def test_short_history_is_untouched(self) -> None:
        history = [_human("hello"), _ai("hi")]
        result = compact_history(history, context_window=100_000)

        assert result.compacted is False
        assert result.messages == history
        assert result.dropped_messages == 0

    def test_single_message_is_never_compacted(self) -> None:
        history = [_human("x" * 10_000_000)]
        result = compact_history(history, context_window=1_000)

        assert result.compacted is False
        assert result.messages == history

    def test_input_history_is_not_mutated(self) -> None:
        history = _long_history(80)
        original = list(history)

        compact_history(history, context_window=2_000)

        assert history == original

    def test_returns_a_copy_not_the_same_list(self) -> None:
        history = [_human("hello")]
        result = compact_history(history, context_window=100_000)

        assert result.messages is not history


class TestCompactHistoryTriggers:
    def test_compacts_when_over_the_trigger(self) -> None:
        history = _long_history(80)
        result = compact_history(history, context_window=2_000)

        assert result.compacted is True
        assert result.dropped_messages > 0
        assert len(result.messages) < len(history)

    def test_result_fits_the_target(self) -> None:
        history = _long_history(200)
        window = 4_000
        result = compact_history(history, context_window=window, keep_ratio=0.35)

        assert result.tokens_after < window * 0.5
        assert result.tokens_after < result.tokens_before

    def test_newest_messages_are_kept(self) -> None:
        history = _long_history(80)
        result = compact_history(history, context_window=2_000)

        assert result.messages[-1] == history[-1]
        assert result.messages[-2] == history[-2]

    def test_oldest_messages_are_dropped(self) -> None:
        history = _long_history(80)
        result = compact_history(history, context_window=2_000)

        assert history[0] not in result.messages

    def test_summary_is_inserted_first(self) -> None:
        history = _long_history(80)
        result = compact_history(history, context_window=2_000)

        assert isinstance(result.messages[0], HumanMessage)
        assert COMPACTION_MARKER in result.messages[0].content

    def test_default_summary_describes_what_was_dropped(self) -> None:
        history = _long_history(80)
        result = compact_history(history, context_window=2_000)

        assert result.summary is not None
        assert "messages were removed" in result.summary

    def test_repeated_compaction_is_stable(self) -> None:
        history = _long_history(200)
        window = 4_000

        first = compact_history(history, context_window=window)
        assert first.compacted is True

        # Compacting the already-compacted history must be a no-op, otherwise
        # every turn would pay for a summarisation.
        second = compact_history(first.messages, context_window=window)
        assert second.compacted is False


class TestToolCallIntegrity:
    def test_kept_history_never_starts_with_a_tool_result(self) -> None:
        """An orphaned ToolMessage is rejected by the OpenAI API."""
        history: list[BaseMessage] = []
        for i in range(120):
            history.append(_human(f"do thing {i} " + "x" * 300))
            history.append(_ai_tool_call("navigate", f"call-{i}", {"to": "x" * 300}))
            history.append(_tool_result("arrived " + "y" * 300, f"call-{i}"))

        result = compact_history(history, context_window=3_000)

        assert result.compacted is True
        first_real = result.messages[1]  # index 0 is the summary
        assert not isinstance(first_real, ToolMessage)

    def test_every_kept_tool_result_has_its_originating_call(self) -> None:
        history: list[BaseMessage] = []
        for i in range(120):
            history.append(_human(f"do thing {i} " + "x" * 300))
            history.append(_ai_tool_call("navigate", f"call-{i}", {"to": "x" * 300}))
            history.append(_tool_result("arrived " + "y" * 300, f"call-{i}"))

        result = compact_history(history, context_window=3_000)

        seen_call_ids: set[str] = set()
        for message in result.messages:
            for call in getattr(message, "tool_calls", None) or []:
                seen_call_ids.add(call["id"])
            if isinstance(message, ToolMessage):
                assert message.tool_call_id in seen_call_ids, (
                    f"tool result {message.tool_call_id} kept without its call"
                )

    def test_parallel_tool_results_stay_with_their_call(self) -> None:
        history: list[BaseMessage] = _long_history(100)
        history.append(
            AIMessage(
                content="",
                tool_calls=[
                    {"name": "a", "args": {}, "id": "p1", "type": "tool_call"},
                    {"name": "b", "args": {}, "id": "p2", "type": "tool_call"},
                ],
            )
        )
        history.append(_tool_result("ra", "p1"))
        history.append(_tool_result("rb", "p2"))
        history.append(_ai("done"))

        result = compact_history(history, context_window=2_000)

        kept_ids = {
            c["id"] for m in result.messages for c in (getattr(m, "tool_calls", None) or [])
        }
        for message in result.messages:
            if isinstance(message, ToolMessage):
                assert message.tool_call_id in kept_ids


class TestSystemMessage:
    def test_leading_system_message_survives(self) -> None:
        history: list[BaseMessage] = [SystemMessage(content="You are a robot.")]
        history.extend(_long_history(80))

        result = compact_history(history, context_window=2_000)

        assert result.compacted is True
        assert isinstance(result.messages[0], SystemMessage)
        assert result.messages[0].content == "You are a robot."
        assert COMPACTION_MARKER in result.messages[1].content


class TestSummarizer:
    def test_custom_summarizer_receives_only_dropped_messages(self) -> None:
        history = _long_history(80)
        seen: list[Sequence[BaseMessage]] = []

        def summarizer(messages: Sequence[BaseMessage]) -> str:
            seen.append(messages)
            return "custom summary"

        result = compact_history(history, context_window=2_000, summarizer=summarizer)

        assert result.summary == "custom summary"
        assert "custom summary" in result.messages[0].content
        assert len(seen) == 1
        assert len(seen[0]) == result.dropped_messages
        # The kept tail must not have been handed to the summarizer.
        assert history[-1] not in seen[0]

    def test_summarizer_failure_falls_back_to_offline_digest(self) -> None:
        history = _long_history(80)

        def broken(_messages: Sequence[BaseMessage]) -> str:
            raise RuntimeError("model is down")

        result = compact_history(history, context_window=2_000, summarizer=broken)

        assert result.compacted is True
        assert result.summary is not None
        assert "messages were removed" in result.summary

    def test_model_summarizer_uses_the_model_response(self) -> None:
        class FakeModel:
            def __init__(self) -> None:
                self.calls: list[list[BaseMessage]] = []

            def invoke(self, messages: list[BaseMessage]) -> AIMessage:
                self.calls.append(messages)
                return AIMessage(content="the robot drove to the kitchen")

        model = FakeModel()
        summarizer = make_model_summarizer(model)
        summary = summarizer([_human("go to the kitchen"), _ai("on my way")])

        assert summary == "the robot drove to the kitchen"
        assert len(model.calls) == 1

    def test_model_summarizer_truncates_a_huge_transcript(self) -> None:
        class CapturingModel:
            def __init__(self) -> None:
                self.prompt = ""

            def invoke(self, messages: list[BaseMessage]) -> AIMessage:
                self.prompt = messages[0].content
                return AIMessage(content="summary")

        model = CapturingModel()
        summarizer = make_model_summarizer(model, max_chars=500)
        summarizer(_long_history(200))

        assert len(model.prompt) < 2_000

    def test_model_summarizer_falls_back_on_empty_response(self) -> None:
        class EmptyModel:
            def invoke(self, _messages: list[BaseMessage]) -> AIMessage:
                return AIMessage(content="")

        summarizer = make_model_summarizer(EmptyModel())
        summary = summarizer(_long_history(3))

        assert "messages were removed" in summary


class TestValidation:
    @pytest.mark.parametrize("window", [0, -1])
    def test_rejects_non_positive_context_window(self, window: int) -> None:
        with pytest.raises(ValueError, match="context_window must be positive"):
            compact_history([_human("a")], context_window=window)

    @pytest.mark.parametrize("ratio", [0, -0.5, 1.5])
    def test_rejects_invalid_trigger_ratio(self, ratio: float) -> None:
        with pytest.raises(ValueError, match="trigger_ratio"):
            compact_history([_human("a")], context_window=1_000, trigger_ratio=ratio)

    @pytest.mark.parametrize("ratio", [0, -0.5, 1.5])
    def test_rejects_invalid_keep_ratio(self, ratio: float) -> None:
        with pytest.raises(ValueError, match="keep_ratio"):
            compact_history([_human("a")], context_window=1_000, keep_ratio=ratio)

    def test_rejects_keep_ratio_above_trigger_ratio(self) -> None:
        with pytest.raises(ValueError, match="must be smaller than trigger_ratio"):
            compact_history([_human("a")], context_window=1_000, trigger_ratio=0.5, keep_ratio=0.9)


class TestRealisticSession:
    def test_a_long_tool_heavy_session_stays_within_budget(self) -> None:
        """Simulates many command turns and checks the history never overflows."""
        window = 8_000
        history: list[BaseMessage] = []
        max_seen = 0

        for i in range(300):
            history.append(_human(f"drive to waypoint {i} " + "d" * 200))
            history.append(_ai_tool_call("navigate", f"c{i}", {"waypoint": i}))
            history.append(_tool_result("arrived at waypoint " + "r" * 400, f"c{i}"))

            result = compact_history(history, context_window=window)
            history = result.messages
            max_seen = max(max_seen, estimate_tokens(history))

        assert max_seen <= window, f"history peaked at {max_seen} tokens, window is {window}"
        # The most recent turn must still be intact.
        assert isinstance(history[-1], ToolMessage)
        assert history[-1].tool_call_id == "c299"
