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
"""Tests for :mod:`dimos.agents.memory.budget`."""

from __future__ import annotations

import pytest

from dimos.agents.memory.budget import (
    DEFAULT_CONTEXT_WINDOW,
    DEFAULT_OUTPUT_RESERVE,
    DEFAULT_PER_MESSAGE_OVERHEAD,
    DEFAULT_SYSTEM_OVERHEAD,
    ModelBudget,
    resolve_budget,
)


# ModelBudget
def test_input_budget_arithmetic() -> None:
    b = ModelBudget(context_window=128_000, output_reserve=4096, system_overhead=256)
    assert b.input_budget == 128_000 - 4096 - 256


def test_input_budget_honours_defaults() -> None:
    b = ModelBudget(context_window=32_000)
    assert b.output_reserve == DEFAULT_OUTPUT_RESERVE
    assert b.system_overhead == DEFAULT_SYSTEM_OVERHEAD
    assert b.per_message_overhead == DEFAULT_PER_MESSAGE_OVERHEAD
    assert b.input_budget == 32_000 - DEFAULT_OUTPUT_RESERVE - DEFAULT_SYSTEM_OVERHEAD


def test_default_per_message_overhead_value() -> None:
    # Contract: default is 4 tokens/message (OpenAI chat-completions).
    assert DEFAULT_PER_MESSAGE_OVERHEAD == 4
    b = ModelBudget(context_window=32_000)
    assert b.per_message_overhead == 4


def test_model_budget_rejects_negative_per_message_overhead() -> None:
    with pytest.raises(ValueError, match="per_message_overhead"):
        ModelBudget(context_window=32_000, per_message_overhead=-1)


# effective_budget_for_messages
def test_effective_budget_subtracts_per_message_overhead() -> None:
    b = ModelBudget(
        context_window=10_000,
        output_reserve=1000,
        system_overhead=100,
        per_message_overhead=4,
    )
    # input_budget = 10_000 - 1000 - 100 = 8_900
    assert b.input_budget == 8_900
    assert b.effective_budget_for_messages(0) == 8_900
    assert b.effective_budget_for_messages(1) == 8_900 - 4
    assert b.effective_budget_for_messages(10) == 8_900 - 40
    assert b.effective_budget_for_messages(100) == 8_900 - 400


def test_effective_budget_zero_overhead_is_no_op() -> None:
    b = ModelBudget(context_window=10_000, per_message_overhead=0)
    assert b.effective_budget_for_messages(0) == b.input_budget
    assert b.effective_budget_for_messages(999) == b.input_budget


def test_effective_budget_rejects_negative_n_messages() -> None:
    b = ModelBudget(context_window=10_000)
    with pytest.raises(ValueError, match="n_messages"):
        b.effective_budget_for_messages(-1)


def test_effective_budget_may_go_negative_when_over_packed() -> None:
    # Selector contract: over-packing is allowed to return a negative
    # ceiling so the caller can detect "over budget" and evict.
    b = ModelBudget(context_window=1000, output_reserve=500, system_overhead=50)
    # input_budget = 1000 - 500 - 50 = 450; 200 messages * 4 = 800.
    assert b.effective_budget_for_messages(200) == 450 - 800


def test_model_budget_rejects_zero_context_window() -> None:
    with pytest.raises(ValueError, match="context_window"):
        ModelBudget(context_window=0)


def test_model_budget_rejects_negative_reserve() -> None:
    with pytest.raises(ValueError, match="output_reserve"):
        ModelBudget(context_window=1000, output_reserve=-1)


def test_model_budget_rejects_reserve_exceeding_window() -> None:
    with pytest.raises(ValueError, match="strictly less"):
        ModelBudget(context_window=4096, output_reserve=4096, system_overhead=0)


def test_model_budget_is_frozen() -> None:
    b = ModelBudget(context_window=1000, output_reserve=100, system_overhead=10)
    with pytest.raises(Exception):  # dataclasses.FrozenInstanceError
        b.context_window = 2000  # type: ignore[misc]


# resolve_budget
def test_resolve_budget_known_openai_models() -> None:
    b = resolve_budget("gpt-4o")
    assert b.context_window == 128_000

    b = resolve_budget("gpt-4.1")
    assert b.context_window == 1_047_576


def test_resolve_budget_known_ollama_model_with_prefix() -> None:
    b = resolve_budget("ollama:llama3.2")
    assert b.context_window == 128_000


def test_resolve_budget_ollama_tagged_model() -> None:
    # Real-world ollama strings include tags like ``:latest``.
    b = resolve_budget("ollama:llama3.2:latest")
    assert b.context_window == 128_000


def test_resolve_budget_override_wins_over_table() -> None:
    b = resolve_budget("gpt-4o", override=8000)
    assert b.context_window == 8000


def test_resolve_budget_falls_back_to_default_for_unknown() -> None:
    b = resolve_budget("totally-made-up-model-v99")
    assert b.context_window == DEFAULT_CONTEXT_WINDOW


def test_resolve_budget_family_prefix_match() -> None:
    # "llama3.1-8b-instruct" is not in the table but should match the
    # ``llama3.1`` family entry.
    b = resolve_budget("llama3.1-8b-instruct")
    assert b.context_window == 128_000


def test_resolve_budget_respects_custom_reserve_and_overhead() -> None:
    b = resolve_budget("gpt-4o", output_reserve=2000, system_overhead=100)
    assert b.output_reserve == 2000
    assert b.system_overhead == 100
    assert b.input_budget == 128_000 - 2000 - 100


def test_resolve_budget_override_validated() -> None:
    with pytest.raises(ValueError):
        resolve_budget("gpt-4o", override=-1)


def test_resolve_budget_kwargs_are_keyword_only() -> None:
    # Contract: everything after ``model_name`` is keyword-only. Passing
    # ``override`` positionally must fail.
    with pytest.raises(TypeError):
        resolve_budget("gpt-4o", 8000)  # type: ignore[misc]


def test_resolve_budget_propagates_per_message_overhead() -> None:
    b = resolve_budget("gpt-4o", per_message_overhead=7)
    assert b.per_message_overhead == 7
    # And it shows up in the effective-budget arithmetic.
    assert b.effective_budget_for_messages(3) == b.input_budget - 21


def test_resolve_budget_longest_prefix_match_o1_mini_dated() -> None:
    # Regression: ``o1-mini-2024-09-12`` must resolve to the ``o1-mini``
    # entry (128K), NOT the ``o1`` entry (200K). The old first-match
    # implementation returned 200_000; longest-prefix returns 128_000.
    b = resolve_budget("o1-mini-2024-09-12")
    assert b.context_window == 128_000, (
        "Expected longest-prefix match to 'o1-mini' (128K); got "
        f"{b.context_window}. If this returned 200_000 the fallback is "
        "using first-match instead of longest-prefix."
    )


def test_resolve_budget_longest_prefix_match_llama3_point_1() -> None:
    # ``llama3.1-8b-instruct`` must match ``llama3.1`` (128K), not the
    # shorter ``llama3`` (8_192).
    b = resolve_budget("llama3.1-8b-instruct")
    assert b.context_window == 128_000


def test_resolve_budget_longest_prefix_match_gpt_4o_mini_dated() -> None:
    # Both ``gpt-4`` and ``gpt-4o`` and ``gpt-4o-mini`` are prefixes of
    # ``gpt-4o-mini-2024-07-18``; longest wins.
    b = resolve_budget("gpt-4o-mini-2024-07-18")
    assert b.context_window == 128_000
