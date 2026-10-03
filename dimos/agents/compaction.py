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

"""Compaction of agent conversation history.

An agent that runs for a while accumulates history faster than it is aware of:
every tool call and every tool result is appended verbatim. Once the history no
longer fits in the model's context window the next request fails outright, which
kills a long-running session.

This module keeps the history under control by dropping the oldest messages and
replacing them with a summary, so the agent retains the gist of what happened
without carrying every byte of it.

The entry point is :func:`compact_history`. It is pure: it takes a list of
messages and returns a new list, so it can be tested without a model or a
network. Summarisation is injected via the *summarizer* argument; when it is
omitted a deterministic, offline digest is used instead.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from langchain_core.messages.base import BaseMessage

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Summarizer = Callable[[Sequence[BaseMessage]], str]
TokenCounter = Callable[[Sequence[BaseMessage]], int]

#: Marker prefix identifying a message produced by compaction. Used to recognise
#: (and re-summarise) an earlier summary when compaction runs more than once.
COMPACTION_MARKER = "[compacted history]"

#: Fallback context window for models we do not know about. Deliberately
#: conservative: over-compacting costs a little fidelity, under-compacting
#: fails the request.
DEFAULT_CONTEXT_WINDOW = 128_000

#: Known context windows, matched by longest prefix against the model name.
MODEL_CONTEXT_WINDOWS: dict[str, int] = {
    "gpt-3.5": 16_385,
    "gpt-4-turbo": 128_000,
    "gpt-4o": 128_000,
    "gpt-4.1": 1_047_576,
    "gpt-4": 8_192,
    "gpt-5": 400_000,
    "o1": 200_000,
    "o3": 200_000,
    "o4": 200_000,
    "claude-3": 200_000,
    "claude-4": 200_000,
    "claude-sonnet": 200_000,
    "claude-opus": 200_000,
    "claude-haiku": 200_000,
    "gemini-1.5": 1_048_576,
    "gemini-2": 1_048_576,
    "llama3": 8_192,
    "llama3.1": 131_072,
    "qwen2.5": 32_768,
    "mistral": 32_768,
}

#: Rough bytes-per-token ratio for English text, used by the offline estimator.
_CHARS_PER_TOKEN = 4

#: Per-message envelope cost (role, delimiters, function-call scaffolding).
_MESSAGE_OVERHEAD_TOKENS = 4

#: Flat cost charged for an image part. Real cost is resolution-dependent; this
#: is a deliberate over-estimate so images trigger compaction early rather than
#: late.
_IMAGE_TOKENS = 1_500


def resolve_context_window(model_name: str | None) -> int:
    """Return the context window for *model_name* in tokens.

    Matching is by longest known prefix, after stripping any ``provider:``
    prefix, so ``ollama:llama3.1:8b`` resolves via ``llama3.1``. Unknown models
    fall back to :data:`DEFAULT_CONTEXT_WINDOW`.
    """
    if not model_name:
        return DEFAULT_CONTEXT_WINDOW

    name = model_name.split(":", 1)[1] if ":" in model_name else model_name
    name = name.strip().lower()

    best: str | None = None
    for prefix in MODEL_CONTEXT_WINDOWS:
        if name.startswith(prefix) and (best is None or len(prefix) > len(best)):
            best = prefix

    if best is None:
        logger.debug("Unknown context window for model %r; using default", model_name)
        return DEFAULT_CONTEXT_WINDOW
    return MODEL_CONTEXT_WINDOWS[best]


def _content_tokens(content: Any) -> int:
    """Estimate the token cost of a message ``content`` field."""
    if content is None:
        return 0
    if isinstance(content, str):
        return len(content) // _CHARS_PER_TOKEN
    if isinstance(content, list):
        total = 0
        for part in content:
            if isinstance(part, dict):
                kind = part.get("type")
                if kind in ("image_url", "image"):
                    total += _IMAGE_TOKENS
                    continue
                if kind == "text":
                    total += len(str(part.get("text", ""))) // _CHARS_PER_TOKEN
                    continue
            total += len(str(part)) // _CHARS_PER_TOKEN
        return total
    return len(str(content)) // _CHARS_PER_TOKEN


def estimate_tokens(messages: Sequence[BaseMessage]) -> int:
    """Estimate the token cost of *messages* without calling a tokenizer.

    This is intentionally cheap and approximate. It runs on every turn, so an
    exact count is not worth the cost; the trigger ratio absorbs the error.
    """
    total = 0
    for message in messages:
        total += _MESSAGE_OVERHEAD_TOKENS + _content_tokens(getattr(message, "content", None))
        for call in getattr(message, "tool_calls", None) or []:
            total += len(str(call.get("args", ""))) // _CHARS_PER_TOKEN
            total += len(str(call.get("name", ""))) // _CHARS_PER_TOKEN
    return total


def _default_summarizer(messages: Sequence[BaseMessage]) -> str:
    """Summarise *messages* offline, without a model.

    Produces a compact per-message digest. Used when no model-backed summarizer
    is supplied, and as the fallback when one fails.
    """
    human = sum(1 for m in messages if isinstance(m, HumanMessage))
    ai = sum(1 for m in messages if isinstance(m, AIMessage))
    tools_used: list[str] = []
    for message in messages:
        for call in getattr(message, "tool_calls", None) or []:
            name = call.get("name")
            if name and name not in tools_used:
                tools_used.append(str(name))

    lines = [
        (
            f"{len(messages)} earlier messages were removed to stay within the "
            f"context window ({human} from the user, {ai} from the assistant)."
        ),
    ]
    if tools_used:
        lines.append(f"Tools used in that period: {', '.join(tools_used)}.")

    first_human = next((m for m in messages if isinstance(m, HumanMessage)), None)
    if first_human is not None:
        text = _first_text(first_human)
        if text:
            lines.append(f"The earliest request was: {text[:200]}")

    last_ai = next(
        (m for m in reversed(messages) if isinstance(m, AIMessage) and _first_text(m)), None
    )
    if last_ai is not None:
        lines.append(f"The last thing the assistant reported was: {_first_text(last_ai)[:200]}")

    return "\n".join(lines)


def _first_text(message: BaseMessage) -> str:
    """Return the first textual chunk of *message*, or an empty string."""
    content = getattr(message, "content", None)
    if isinstance(content, str):
        return content.strip()
    if isinstance(content, list):
        for part in content:
            if isinstance(part, dict) and part.get("type") == "text":
                return str(part.get("text", "")).strip()
    return ""


def _split_index(
    messages: Sequence[BaseMessage], target_tokens: int, token_counter: TokenCounter
) -> int:
    """Return the index at which to cut *messages* so the suffix is valid.

    The suffix ``messages[index:]`` is the newest run of messages that fits in
    *target_tokens*, adjusted so it never begins with a :class:`ToolMessage`.
    An orphaned tool result — one whose originating tool call was dropped — is
    rejected by the OpenAI API, so those are dropped along with their call.

    Always keeps at least the final message: sending an empty history is worse
    than sending an oversized one.
    """
    index = len(messages)
    used = 0
    for i in range(len(messages) - 1, -1, -1):
        cost = token_counter([messages[i]])
        if used + cost > target_tokens and i != len(messages) - 1:
            break
        used += cost
        index = i

    while index < len(messages) - 1 and isinstance(messages[index], ToolMessage):
        index += 1

    return index


@dataclass
class CompactionResult:
    """Outcome of a :func:`compact_history` call."""

    messages: list[BaseMessage]
    compacted: bool
    tokens_before: int
    tokens_after: int
    dropped_messages: int = 0
    summary: str | None = field(default=None, repr=False)


def compact_history(
    history: Sequence[BaseMessage],
    *,
    context_window: int,
    trigger_ratio: float = 0.8,
    keep_ratio: float = 0.35,
    summarizer: Summarizer | None = None,
    token_counter: TokenCounter | None = None,
) -> CompactionResult:
    """Shrink *history* to fit in *context_window*, summarising what is dropped.

    Args:
        history: Conversation so far, oldest first. Not mutated.
        context_window: Size of the model's context window, in tokens.
        trigger_ratio: Fraction of the window at which compaction kicks in.
            Below this the history is returned untouched.
        keep_ratio: Fraction of the window the compacted history should target.
            Must be smaller than *trigger_ratio*, otherwise compaction would
            re-trigger on every turn.
        summarizer: Called with the dropped messages to produce the summary
            text. Defaults to an offline digest. If it raises, the offline
            digest is used instead — a failed summary must not fail the turn.
        token_counter: Token estimator. Defaults to :func:`estimate_tokens`.

    Returns:
        A :class:`CompactionResult`. When nothing was dropped, ``messages`` is
        a copy of *history* and ``compacted`` is ``False``.

    Raises:
        ValueError: If the ratios are outside ``(0, 1]`` or *keep_ratio* is not
            smaller than *trigger_ratio*, or if *context_window* is not
            positive.
    """
    if context_window <= 0:
        raise ValueError(f"context_window must be positive, got {context_window}")
    if not 0 < trigger_ratio <= 1:
        raise ValueError(f"trigger_ratio must be in (0, 1], got {trigger_ratio}")
    if not 0 < keep_ratio <= 1:
        raise ValueError(f"keep_ratio must be in (0, 1], got {keep_ratio}")
    if keep_ratio >= trigger_ratio:
        raise ValueError(
            f"keep_ratio ({keep_ratio}) must be smaller than trigger_ratio "
            f"({trigger_ratio}), otherwise compaction re-triggers every turn"
        )

    count = token_counter or estimate_tokens
    tokens_before = count(history)
    messages = list(history)

    if tokens_before <= context_window * trigger_ratio or len(messages) <= 1:
        return CompactionResult(
            messages=messages,
            compacted=False,
            tokens_before=tokens_before,
            tokens_after=tokens_before,
        )

    # A leading system message is instruction, not conversation: never drop it.
    preserved: list[BaseMessage] = []
    body = messages
    if messages and isinstance(messages[0], SystemMessage):
        preserved = [messages[0]]
        body = messages[1:]

    target = int(context_window * keep_ratio)
    split = _split_index(body, target, count)
    dropped = body[:split]
    kept = body[split:]

    if not dropped:
        return CompactionResult(
            messages=messages,
            compacted=False,
            tokens_before=tokens_before,
            tokens_after=tokens_before,
        )

    summary_text: str
    if summarizer is None:
        summary_text = _default_summarizer(dropped)
    else:
        try:
            summary_text = summarizer(dropped)
        except Exception:
            logger.warning(
                "History summarizer failed; falling back to offline digest", exc_info=True
            )
            summary_text = _default_summarizer(dropped)

    summary_message = HumanMessage(content=f"{COMPACTION_MARKER}\n{summary_text}")
    compacted = [*preserved, summary_message, *kept]
    tokens_after = count(compacted)

    logger.info(
        "Compacted agent history: %d -> %d messages, ~%d -> ~%d tokens (window %d, trigger %.0f%%)",
        len(messages),
        len(compacted),
        tokens_before,
        tokens_after,
        context_window,
        trigger_ratio * 100,
    )

    return CompactionResult(
        messages=compacted,
        compacted=True,
        tokens_before=tokens_before,
        tokens_after=tokens_after,
        dropped_messages=len(dropped),
        summary=summary_text,
    )


def make_model_summarizer(model: Any, *, max_chars: int = 6_000) -> Summarizer:
    """Build a :data:`Summarizer` that asks *model* to summarise the history.

    The transcript handed to the model is truncated to *max_chars* so that
    summarising an oversized history does not itself overflow the window.
    """

    def _summarize(messages: Sequence[BaseMessage]) -> str:
        transcript_parts = []
        for message in messages:
            text = _first_text(message)
            if not text:
                continue
            transcript_parts.append(f"{message.__class__.__name__}: {text}")
        transcript = "\n".join(transcript_parts)
        if len(transcript) > max_chars:
            # Keep the tail: recent context matters more than the opening.
            transcript = "...\n" + transcript[-max_chars:]

        prompt = (
            "Summarise the following portion of an agent conversation. It is "
            "being removed to free up context, so preserve anything the agent "
            "still needs: the user's goals, decisions taken, tools run and "
            "their outcomes, and any unfinished work. Be concise and factual.\n\n"
            f"{transcript}"
        )
        response = model.invoke([HumanMessage(content=prompt)])
        return _first_text(response) or _default_summarizer(messages)

    return _summarize
