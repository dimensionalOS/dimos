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
"""Tests for :mod:`dimos.agents.memory.tokens`."""
from __future__ import annotations

import pytest
from langchain_core.messages import AIMessage, HumanMessage

from dimos.agents.memory.budget import ModelBudget
from dimos.agents.memory.tokens import (
    IMAGE_FULL_COST,
    IMAGE_THUMB_COST,
    HeuristicCounter,
    TiktokenCounter,
    TokenCounter,
    _sum_content_tokens,
    count_image_part,
)


# --- Protocol ------------------------------------------------------------


def test_heuristic_and_tiktoken_implement_token_counter_protocol() -> None:
    pytest.importorskip("tiktoken")
    assert isinstance(HeuristicCounter(), TokenCounter)
    assert isinstance(TiktokenCounter("gpt-4o"), TokenCounter)


# --- HeuristicCounter ----------------------------------------------------


def test_heuristic_count_text_ceil_divides_by_four() -> None:
    c = HeuristicCounter()
    assert c.count_text("") == 0
    # 1..4 chars -> 1 token, 5..8 chars -> 2 tokens, ...
    assert c.count_text("a") == 1
    assert c.count_text("abcd") == 1
    assert c.count_text("abcde") == 2
    assert c.count_text("a" * 12) == 3
    assert c.count_text("a" * 13) == 4


def test_heuristic_count_message_plain_text() -> None:
    c = HeuristicCounter()
    msg = HumanMessage(content="hello world!")  # 12 chars -> 3 tokens
    # Content tokens only; per-message overhead lives on ``ModelBudget``.
    assert c.count_message(msg) == 3


def test_heuristic_count_message_with_image_part_full() -> None:
    c = HeuristicCounter()
    msg = HumanMessage(
        content=[
            {"type": "text", "text": "look"},  # 4 chars -> 1 tok
            {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,ABC"}},
        ]
    )
    expected = 1 + IMAGE_FULL_COST
    assert c.count_message(msg) == expected


def test_heuristic_count_message_with_image_part_low_detail() -> None:
    c = HeuristicCounter()
    msg = HumanMessage(
        content=[
            {"type": "image_url", "image_url": {"url": "...", "detail": "low"}},
        ]
    )
    expected = IMAGE_THUMB_COST
    assert c.count_message(msg) == expected


def test_heuristic_count_message_unknown_part_priced_as_text() -> None:
    c = HeuristicCounter()
    msg = HumanMessage(content=[{"type": "weird", "x": "abcd"}])
    # The stringified dict gets priced via count_text — all we care about is
    # that the result is >= 1 token (never silently dropped).
    assert c.count_message(msg) >= 1


def test_heuristic_count_message_with_non_list_non_str_content() -> None:
    c = HeuristicCounter()
    msg = AIMessage(content="")
    # Empty content — exactly zero tokens now that overhead has moved to
    # ``ModelBudget``.
    assert c.count_message(msg) == 0


# --- count_image_part ---------------------------------------------------


def test_count_image_part_defaults_to_full_cost() -> None:
    assert count_image_part({"type": "image_url", "image_url": {"url": "..."}}) == IMAGE_FULL_COST


def test_count_image_part_respects_low_detail_hint() -> None:
    part = {"type": "image_url", "image_url": {"url": "...", "detail": "low"}}
    assert count_image_part(part) == IMAGE_THUMB_COST


def test_count_image_part_high_detail_hint_is_full() -> None:
    part = {"type": "image_url", "image_url": {"url": "...", "detail": "high"}}
    assert count_image_part(part) == IMAGE_FULL_COST


# --- TiktokenCounter ----------------------------------------------------


def test_tiktoken_counter_matches_tiktoken_directly() -> None:
    tiktoken = pytest.importorskip("tiktoken")
    c = TiktokenCounter("gpt-4o")
    text = "The quick brown fox jumps over the lazy dog."
    expected = len(tiktoken.encoding_for_model("gpt-4o").encode(text))
    assert c.count_text(text) == expected


def test_tiktoken_counter_falls_back_for_unknown_model() -> None:
    pytest.importorskip("tiktoken")
    c = TiktokenCounter("not-a-real-model-v99")
    # Should not raise; should produce a positive count.
    assert c.count_text("hello world") > 0


def test_tiktoken_counter_empty_text_zero_tokens() -> None:
    pytest.importorskip("tiktoken")
    c = TiktokenCounter("gpt-4o")
    assert c.count_text("") == 0


def test_tiktoken_count_message_with_image_adds_fixed_cost() -> None:
    pytest.importorskip("tiktoken")
    c = TiktokenCounter("gpt-4o")
    msg = HumanMessage(
        content=[
            {"type": "text", "text": "hi"},
            {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,..."}},
        ]
    )
    text_tokens = c.count_text("hi")
    # Content only: text tokens + the fixed image cost. No prelude term.
    assert c.count_message(msg) == text_tokens + IMAGE_FULL_COST


# --- content-only contract ---------------------------------------------
#
# These tests lock in the content-only contract: counters measure
# payload, :class:`~dimos.agents.memory.budget.ModelBudget` reserves
# framing. The arithmetic on each side of the ledger must not leak into
# the other.


def test_heuristic_count_message_is_content_only() -> None:
    c = HeuristicCounter()
    parts = [
        {"type": "text", "text": "one two three"},      # 13 chars -> 4 tok
        {"type": "text", "text": "alpha"},              # 5  chars -> 2 tok
        {"type": "text", "text": "abcdefghijklmnop"},   # 16 chars -> 4 tok
    ]
    msg = HumanMessage(content=parts)
    n = sum(c.count_text(p["text"]) for p in parts)
    # Exactly N — no +4 prelude.
    assert c.count_message(msg) == n, (
        f"HeuristicCounter.count_message should be content-only; got "
        f"{c.count_message(msg)} vs expected {n}. If the diff is 4, the "
        "per-message prelude snuck back in — move it to ModelBudget."
    )


def test_tiktoken_count_message_is_content_only() -> None:
    pytest.importorskip("tiktoken")
    c = TiktokenCounter("gpt-4o")
    parts = [
        {"type": "text", "text": "The quick brown fox"},
        {"type": "text", "text": "jumps over the lazy dog."},
        {"type": "text", "text": "Sphinx of black quartz, judge my vow."},
    ]
    msg = HumanMessage(content=parts)
    n = sum(c.count_text(p["text"]) for p in parts)
    assert c.count_message(msg) == n, (
        f"TiktokenCounter.count_message should be content-only; got "
        f"{c.count_message(msg)} vs expected {n}. If the diff is 4, the "
        "per-message prelude snuck back in — move it to ModelBudget."
    )


def test_per_message_overhead_lives_on_budget_not_counter() -> None:
    """Regression: separate the two halves of the token ledger.

    Budget owns per-message framing; counters own payload. Any future
    refactor that re-adds a prelude term inside :meth:`count_message`
    will fail here.
    """
    budget = ModelBudget(
        context_window=100_000,
        output_reserve=1000,
        system_overhead=500,
        per_message_overhead=4,
    )
    # The budget ceiling shrinks by exactly per_message_overhead * n
    # messages — the counter never participates in that subtraction.
    assert budget.effective_budget_for_messages(10) == budget.input_budget - 40

    # Counters return precisely _sum_content_tokens(msg.content, ...).
    msg = HumanMessage(
        content=[
            {"type": "text", "text": "hello"},
            {"type": "text", "text": "world"},
            {"type": "image_url", "image_url": {"url": "x", "detail": "low"}},
        ]
    )
    counter = HeuristicCounter()
    assert counter.count_message(msg) == _sum_content_tokens(
        msg.content, counter.count_text
    )

    tiktoken = pytest.importorskip("tiktoken")  # noqa: F841
    tk_counter = TiktokenCounter("gpt-4o")
    assert tk_counter.count_message(msg) == _sum_content_tokens(
        msg.content, tk_counter.count_text
    )
