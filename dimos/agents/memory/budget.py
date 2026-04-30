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

"""Model context-window budgeting.

:class:`ModelBudget` captures the three numbers the selector needs to cap
input length:

* ``context_window`` — the model's hard limit.
* ``output_reserve`` — tokens reserved for the response.
* ``system_overhead`` — slack for chat formatting / tool schemas.

The effective limit on the *assembled prompt* is ``input_budget``
(``context_window - output_reserve - system_overhead``).

:func:`resolve_budget` is the factory used by :class:`MemoryEngine`. It
looks up a model name, honours a user-supplied override and defaults to
a conservative 32K window for anything unknown.
"""

from __future__ import annotations

from dataclasses import dataclass

__all__ = [
    "DEFAULT_CONTEXT_WINDOW",
    "DEFAULT_OUTPUT_RESERVE",
    "DEFAULT_PER_MESSAGE_OVERHEAD",
    "DEFAULT_SYSTEM_OVERHEAD",
    "ModelBudget",
    "resolve_budget",
]

DEFAULT_CONTEXT_WINDOW = 32_000
"""Conservative default window for unknown models."""

DEFAULT_OUTPUT_RESERVE = 4096
"""Default tokens reserved for the LLM's response."""

DEFAULT_SYSTEM_OVERHEAD = 256
"""Default formatting/overhead slack."""

DEFAULT_PER_MESSAGE_OVERHEAD = 4
"""Default per-message framing cost (OpenAI chat-completions is ~4 tok/msg)."""


@dataclass(frozen=True)
class ModelBudget:
    """Immutable token budget for one chat turn.

    Attributes:
        context_window: Model's hard token limit (input + output).
        output_reserve: Tokens held back for the response.
        system_overhead: Small fixed overhead for chat formatting, the
            tool-calling preamble, etc.
        per_message_overhead: Per-message role/framing cost paid on top of
            the encoded content. OpenAI's chat completions charges ~4
            tokens per message for role + separators; the selector
            subtracts ``per_message_overhead * n_messages`` from
            :attr:`input_budget` to keep the assembled prompt below the
            hard limit.
    """

    context_window: int
    output_reserve: int = DEFAULT_OUTPUT_RESERVE
    system_overhead: int = DEFAULT_SYSTEM_OVERHEAD
    per_message_overhead: int = DEFAULT_PER_MESSAGE_OVERHEAD

    def __post_init__(self) -> None:
        if self.context_window <= 0:
            raise ValueError(
                f"ModelBudget.context_window must be positive, got {self.context_window}"
            )
        if self.output_reserve < 0:
            raise ValueError(
                f"ModelBudget.output_reserve must be non-negative, got {self.output_reserve}"
            )
        if self.system_overhead < 0:
            raise ValueError(
                f"ModelBudget.system_overhead must be non-negative, got {self.system_overhead}"
            )
        if self.per_message_overhead < 0:
            raise ValueError(
                f"ModelBudget.per_message_overhead must be non-negative, got "
                f"{self.per_message_overhead}"
            )
        if self.output_reserve + self.system_overhead >= self.context_window:
            raise ValueError(
                f"ModelBudget: output_reserve ({self.output_reserve}) + "
                f"system_overhead ({self.system_overhead}) must be strictly less "
                f"than context_window ({self.context_window})"
            )

    @property
    def input_budget(self) -> int:
        """Hard ceiling for the assembled prompt (excluding response)."""
        return self.context_window - self.output_reserve - self.system_overhead

    def effective_budget_for_messages(self, n_messages: int) -> int:
        """Input budget after subtracting per-message framing overhead.

        The selector shrinks this ceiling as messages accumulate; recompute
        after every add/remove.

        Args:
            n_messages: Number of messages currently in the assembled
                prompt.

        Returns:
            ``input_budget - per_message_overhead * n_messages``. May go
            negative if the caller tries to pack more messages than the
            budget can frame; the selector treats that as "over budget"
            and evicts.

        Raises:
            ValueError: if *n_messages* is negative.
        """
        if n_messages < 0:
            raise ValueError(
                f"n_messages must be non-negative, got {n_messages}"
            )
        return self.input_budget - self.per_message_overhead * n_messages


# Known model windows. Numbers are the widely-documented context lengths
# as of 2026-04. Conservative where ambiguity exists. Keep this table
# sorted alphabetically for easy review.
_MODEL_WINDOWS: dict[str, int] = {
    # --- OpenAI ---
    "gpt-3.5-turbo": 16_385,
    "gpt-3.5-turbo-16k": 16_385,
    "gpt-4": 8_192,
    "gpt-4-32k": 32_768,
    "gpt-4-turbo": 128_000,
    "gpt-4-turbo-preview": 128_000,
    "gpt-4.1": 1_047_576,
    "gpt-4.1-mini": 1_047_576,
    "gpt-4.1-nano": 1_047_576,
    "gpt-4o": 128_000,
    "gpt-4o-mini": 128_000,
    "gpt-5": 400_000,
    "gpt-5-mini": 400_000,
    "o1": 200_000,
    "o1-mini": 128_000,
    "o1-preview": 128_000,
    "o3-mini": 200_000,
    # --- Anthropic ---
    "claude-3-5-sonnet": 200_000,
    "claude-3-5-sonnet-latest": 200_000,
    "claude-3-haiku": 200_000,
    "claude-3-opus": 200_000,
    "claude-3-sonnet": 200_000,
    "claude-4-opus": 200_000,
    "claude-4-sonnet": 200_000,
    # --- Llama / ollama ---
    "llama2": 4_096,
    "llama3": 8_192,
    "llama3.1": 128_000,
    "llama3.2": 128_000,
    "llama3.3": 128_000,
    "llama4": 128_000,
    # --- Qwen / Mistral / misc ---
    "mistral": 32_768,
    "mistral-large": 128_000,
    "mixtral": 32_768,
    "qwen2": 32_768,
    "qwen2.5": 131_072,
    "qwen3": 131_072,
}


def _normalize_model(name: str) -> str:
    """Strip provider prefixes and tag suffixes for table lookup.

    LangChain / ollama model strings are commonly prefixed (``ollama:``,
    ``openai:``) or tagged (``llama3.2:latest``, ``gpt-4o-2024-08-06``).
    The table keys are plain family names; we peel the decorations off so
    operator-supplied strings still resolve to known windows.
    """
    s = name.strip()
    for prefix in ("ollama:", "openai:", "anthropic:", "groq:"):
        if s.startswith(prefix):
            s = s[len(prefix):]
            break
    # Drop ollama ``:tag`` and OpenAI ``-YYYY-MM-DD`` date suffixes.
    if ":" in s:
        s = s.split(":", 1)[0]
    return s


def resolve_budget(
    model_name: str,
    *,
    override: int | None = None,
    output_reserve: int = DEFAULT_OUTPUT_RESERVE,
    system_overhead: int = DEFAULT_SYSTEM_OVERHEAD,
    per_message_overhead: int = DEFAULT_PER_MESSAGE_OVERHEAD,
) -> ModelBudget:
    """Resolve a :class:`ModelBudget` for *model_name*.

    All arguments after ``model_name`` are keyword-only; callers must pass
    ``override=...`` etc. explicitly.

    Args:
        model_name: Provider-qualified model string (e.g. ``"gpt-4o"`` or
            ``"ollama:llama3.2:latest"``).
        override: If set, use this as the ``context_window`` instead of the
            table lookup. Useful when operators want to keep prompts
            artificially small for cost / latency control.
        output_reserve: Tokens held back for the response.
        system_overhead: Formatting / tool-schema slack.
        per_message_overhead: Per-message role/framing cost. See
            :class:`ModelBudget`.

    Returns:
        A frozen :class:`ModelBudget`. Unknown models fall back to
        :data:`DEFAULT_CONTEXT_WINDOW`.

    Raises:
        ValueError: if *override* or any other argument is non-positive /
            negative (validation is done in :class:`ModelBudget`).
    """
    if override is not None:
        window = override
    else:
        normalized = _normalize_model(model_name)
        window = _MODEL_WINDOWS.get(normalized)
        if window is None:
            # Longest-prefix match: for something like
            # ``o1-mini-2024-09-12`` both ``o1`` and ``o1-mini`` are
            # prefixes; we must pick the more specific (longer) one so
            # ``o1-mini`` wins and reports 128K rather than o1's 200K.
            best_key: str | None = None
            for key in _MODEL_WINDOWS:
                if normalized.startswith(key) and (
                    best_key is None or len(key) > len(best_key)
                ):
                    best_key = key
            if best_key is not None:
                window = _MODEL_WINDOWS[best_key]
        if window is None:
            window = DEFAULT_CONTEXT_WINDOW

    return ModelBudget(
        context_window=window,
        output_reserve=output_reserve,
        system_overhead=system_overhead,
        per_message_overhead=per_message_overhead,
    )
