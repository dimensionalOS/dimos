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

"""Token counting primitives used by the memory package.

Two concrete counters are shipped:

* :class:`TiktokenCounter` — accurate for OpenAI models, uses ``tiktoken``.
* :class:`HeuristicCounter` — model-agnostic fallback that estimates four
  characters per token and adds a fixed per-image cost.

Both implement the structural :class:`TokenCounter` protocol that the rest
of the package depends on. Multimodal content parts (``image_url``) are
priced with a constant because tiktoken cannot tokenize images.

**Contract boundary with** :class:`~dimos.agents.memory.budget.ModelBudget`.
:meth:`TokenCounter.count_message` returns **content tokens only**. The
per-message ChatML prelude (``<|im_start|>role\\n…<|im_end|>``, commonly
modelled as ~4 tokens/message) is owned exclusively by
:meth:`ModelBudget.effective_budget_for_messages`, which subtracts
``per_message_overhead * n_messages`` from the assembly ceiling. Splitting
the two sides of the ledger prevents double-counting: counters measure
payload, budgets reserve framing. If you find yourself tempted to re-add
a "+ prelude" term inside a counter, the fix belongs in ``ModelBudget``
instead.
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

from langchain_core.messages.base import BaseMessage

__all__ = [
    "HeuristicCounter",
    "TiktokenCounter",
    "TokenCounter",
    "count_image_part",
]

# --- image token pricing ------------------------------------------------
#
# OpenAI prices image inputs based on detail level. The published "low
# detail" per-image cost is 85 tokens; "high detail" is 85 + 170 * tiles,
# which for a single 512x512 tile works out to around 1105 tokens. We use
# those two constants as our image cost lookup; they are intentionally
# conservative because we would rather over-estimate budget usage than
# blow past the context window.

IMAGE_FULL_COST = 1105
"""Tokens charged for a full-fidelity image part."""

IMAGE_THUMB_COST = 85
"""Tokens charged for a thumbnail-fidelity image part (low-detail OpenAI pricing)."""


@runtime_checkable
class TokenCounter(Protocol):
    """Structural protocol for token counting backends.

    Implementations must be cheap to call (``O(len(text))`` at worst) —
    :mod:`selector` calls them in tight loops when greedily upgrading page
    fidelity.
    """

    def count_message(self, msg: BaseMessage) -> int:
        """Return the content-token cost of a LangChain message.

        **Returns content tokens only.** The per-message ChatML prelude
        overhead is accounted for in
        :meth:`~dimos.agents.memory.budget.ModelBudget.effective_budget_for_messages`,
        not here. Double-counting is prevented by this separation: counters
        measure payload, budgets reserve framing.

        Concretely, an implementation should return
        ``_sum_content_tokens(msg.content, self.count_text)`` (or its
        backend-specific equivalent) with no additive prelude constant.
        """

    def count_text(self, text: str) -> int:
        """Return the token cost of a plain string."""


# --- helpers ------------------------------------------------------------


def count_image_part(part: dict[str, Any]) -> int:
    """Fixed-cost estimate for one ``image_url`` content part.

    We infer "thumbnail vs full" from the ``detail`` hint when present
    (``low`` -> :data:`IMAGE_THUMB_COST`) and otherwise assume FULL.
    Callers that want exact control build representations with an explicit
    ``token_estimate`` at ingestion time.
    """
    detail = part.get("image_url", {}).get("detail")
    if detail == "low":
        return IMAGE_THUMB_COST
    return IMAGE_FULL_COST


def _sum_content_tokens(
    content: Any,
    text_fn: "callable[[str], int]",  # noqa: UP037 (callable used as forward-ref string)
) -> int:
    """Sum tokens across a message ``content`` field.

    ``content`` may be a plain string or a list of LangChain content parts
    (dicts with a ``type`` discriminator). Unknown parts are treated as
    text.
    """
    if isinstance(content, str):
        return text_fn(content)

    if not isinstance(content, list):
        return text_fn(str(content))

    total = 0
    for part in content:
        if not isinstance(part, dict):
            total += text_fn(str(part))
            continue
        ptype = part.get("type")
        if ptype == "text":
            total += text_fn(str(part.get("text", "")))
        elif ptype == "image_url" or ptype == "image":
            total += count_image_part(part)
        else:
            # Unknown part — price conservatively as text over the whole
            # JSON so we never silently under-count.
            total += text_fn(str(part))
    return total


# --- concrete counters --------------------------------------------------


class HeuristicCounter:
    """Character-length fallback counter.

    Approximates OpenAI's BPE at roughly 4 characters per token for English
    prose. Fine for budgeting — the selector rounds up anyway — but should
    not be used to drive billing.

    :meth:`count_message` returns content tokens only; the ChatML prelude
    is owned by :class:`~dimos.agents.memory.budget.ModelBudget`.
    """

    CHARS_PER_TOKEN = 4

    def count_text(self, text: str) -> int:
        if not text:
            return 0
        # ceil(len / 4) — any non-empty string costs at least 1 token.
        return max(1, (len(text) + self.CHARS_PER_TOKEN - 1) // self.CHARS_PER_TOKEN)

    def count_message(self, msg: BaseMessage) -> int:
        return _sum_content_tokens(msg.content, self.count_text)


class TiktokenCounter:
    """tiktoken-based counter, accurate for OpenAI and many llama models.

    The tiktoken encoding is resolved from the model name; unknown models
    fall back to ``cl100k_base`` (gpt-4/gpt-3.5 encoding), which is
    approximately right for modern decoder-only LLMs. Import of tiktoken
    is lazy so the ``agents`` extra is not a hard dependency of the memory
    package.

    :meth:`count_message` returns content tokens only; the ChatML prelude
    is owned by :class:`~dimos.agents.memory.budget.ModelBudget`.
    """

    def __init__(self, model: str) -> None:
        import tiktoken

        try:
            self._encoding = tiktoken.encoding_for_model(model)
        except KeyError:
            self._encoding = tiktoken.get_encoding("cl100k_base")
        self._heuristic_fallback = HeuristicCounter()
        self.model = model

    def count_text(self, text: str) -> int:
        if not text:
            return 0
        try:
            return len(self._encoding.encode(text))
        except Exception:
            # Some tiktoken paths reject non-UTF strings; fall back.
            return self._heuristic_fallback.count_text(text)

    def count_message(self, msg: BaseMessage) -> int:
        return _sum_content_tokens(msg.content, self.count_text)
