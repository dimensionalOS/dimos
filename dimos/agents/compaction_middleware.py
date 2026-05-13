# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""LangChain middleware that caps the input the agent sends to its model.

The agent's `_history` would otherwise grow unbounded. This middleware runs
`before_model` on every LLM call and, if the projected input exceeds
`threshold_tokens`, compacts in two stages:

  1. Drop image content blocks (replace with a small text placeholder).
  2. If still over `target_tokens`, summarize the older messages into a single
     `SystemMessage`, keeping the most-recent tail verbatim.

The leading `SystemMessage` (the agent's identity, set via
`create_agent(system_prompt=...)`) is preserved verbatim. Mid-list untagged
messages and prior compaction summaries are eligible for summarization.

Token counting is a pessmistic approximation for now (3 chars/token, 1000 tokens/image) and
is memoized in `msg.additional_kwargs["dimos_tokens"]` so subsequent turns only
pay for newly-added messages.

Design — why a middleware, and why `before_model`
-------------------------------------------------

Compaction is an *invariant* of the prompt the LLM sees, not a feature of any
particular caller. Two consequences shape the design:

1. **Middleware**
   Putting compaction inside the state graph (via `create_agent(middleware=...)`)
   means *every* path into the model is bounded — current callers, future
   callers, alternate agents — without each one having to remember to call a
   compact helper. The contract is enforced via a hook that is always called.

   It also handles a subtlety in the langgraph agent loop: a single user turn
   can invoke the model multiple times (model → tool call → tool result →
   model again → …). External pre-processing on `_history` would only run
   once per user turn, leaving every intra-turn re-invocation unprotected.
   The middleware fires *before each* model call, so the size bound is a true
   invariant of the loop, not a "checked at user-message boundaries" property.

2. **`before_model`, not `after_model` or `wrap_model_call`.**
   `before_model` is the minimal-intervention hook that lets us transform the
   state the model is about to receive. `after_model` runs too late — the
   model has already been called and may have errored on context overflow.
   `wrap_model_call` could work but means owning the entire request/response,
   which conflates compaction with model-call concerns (retries, error
   shaping, tool dispatch). `before_model` keeps the responsibility narrow:
   adjust state in, return; everything else stays the agent loop's job.

The current turn is treated as sacred (see `_current_turn_start`): even when
over threshold we never touch the latest `dimos_turn` group, because that's
the in-flight user query plus any tool calls/responses still being resolved.
Compressing those would either confuse the model mid-step or strip the very
context the user is asking about right now.

Tool-call coherence is the harness's responsibility
---------------------------------------------------

The middleware never *introduces* orphan tool calls: `_split` aligns cuts to
`dimos_turn` boundaries, so an `AIMessage(tool_calls=...)` and its matching
`ToolMessage` — both stamped with the same turn — always travel together
into either the summary or the kept tail. But the middleware doesn't *fix*
orphans it inherits either. If the harness appends an
`AIMessage(tool_calls=...)` without its corresponding `ToolMessage`, the
orphan is passed through verbatim when it lives in the current turn (and the
LLM call will typically raise on the malformed conversation). The middleware
surfaces the issue; it doesn't paper over it. Proper turn-ordering on append
is the caller's job.

Known limitations
-----------------

1. **Summarizer context overflow on huge transcripts.** The transcript fed to
   the summarizer can be arbitrarily large — bounded only by what
   `before_model` decides to summarize. On the very first compaction event of
   a long-running session, the transcript could exceed the summarizer model's
   own context window, raising a provider error that `@retry` will dutifully
   re-issue twice before propagating. Mitigation when it happens in practice:
   either pre-truncate the transcript here, or chunk-and-fold-summarize
   iteratively. Out of scope for the current placeholder-tokenizer phase.

2. **`@retry(on_exception=Exception)` is intentionally broad.** Because the
   summarizer is duck-typed (`Any`), we don't know which provider-specific
   exception classes (httpx, openai, anthropic, …) signal transient vs.
   permanent failures. Catching `Exception` means a permanent error (bad API
   key, invalid schema, programming bug) costs up to 3 attempts + 1s of
   sleeps before propagating. Acceptable trade-off vs. coupling the
   middleware to a specific provider SDK; narrow the exception list if you
   pin to one.
"""

from __future__ import annotations

import json
from typing import Any, cast

from langchain.agents.middleware import AgentMiddleware
from langchain_core.messages import (
    AIMessage,
    BaseMessage,
    HumanMessage,
    RemoveMessage,
    SystemMessage,
)
from langgraph.graph.message import REMOVE_ALL_MESSAGES

from dimos.utils.decorators.decorators import retry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# These are magic numbers, but they are used as pessimistic estimations of
# prompt size. It's better to be more cautious.
CHARS_PER_TOKEN = 3
TOKENS_PER_IMAGE = 1000
IMAGE_PLACEHOLDER = "[image removed during compaction]"

DEFAULT_SUMMARY_PROMPT = """\
You are compacting a conversation between a user and an AI agent controlling a robot.
Write a single concise paragraph in this exact style:

  "User asked X. Agent did A, B, C. User asked Y. Agent did P, Q."

Rules:
  - Preserve user goals, decisions made, tool calls (with results), and current state.
  - Drop pleasantries, intermediate reasoning, and repeated content.
  - Aim for at most {summary_size_tokens} tokens worth of text.
  - Output ONLY the summary. No preamble, no headings.

TRANSCRIPT:
{transcript}
"""


def count_tokens(text: str) -> int:
    """Approximate token count for a string. Pessimistic heuristic: ceil(len/3)."""
    if not text:
        return 0
    return max(1, (len(text) + CHARS_PER_TOKEN - 1) // CHARS_PER_TOKEN)


def count_image_tokens() -> int:
    """Cost of one image content block. Pessimistic heuristic."""
    return TOKENS_PER_IMAGE


def _count_content(content: Any) -> int:
    """Count tokens for a `content` value (str or list of content blocks)."""
    if isinstance(content, str):
        return count_tokens(content)
    if isinstance(content, list):
        total = 0
        for block in content:
            if isinstance(block, dict):
                btype = block.get("type")
                if btype == "text":
                    total += count_tokens(block.get("text", ""))
                elif btype in ("image_url", "image"):
                    total += count_image_tokens()
                else:
                    total += count_tokens(str(block))
            else:
                total += count_tokens(str(block))
        return total
    return count_tokens(str(content))


def count_message_tokens(msg: BaseMessage) -> int:
    """Token count for a message, memoized into `additional_kwargs["dimos_tokens"]`."""
    kwargs = getattr(msg, "additional_kwargs", None)
    if isinstance(kwargs, dict):
        cached = kwargs.get("dimos_tokens")
        if isinstance(cached, int):
            return cached

    total = _count_content(msg.content)
    # AIMessage tool_calls are sent as JSON; count their serialized form.
    tool_calls = getattr(msg, "tool_calls", None)
    if tool_calls:
        total += count_tokens(json.dumps(tool_calls, default=str))

    if isinstance(kwargs, dict):
        kwargs["dimos_tokens"] = total
    return total


def _has_image(content: Any) -> bool:
    if not isinstance(content, list):
        return False
    return any(isinstance(b, dict) and b.get("type") in ("image_url", "image") for b in content)


class DimosCompactionMiddleware(AgentMiddleware):  # type: ignore[misc]
    """`before_model` hook that compacts message history to a token budget.

    See module docstring for the algorithm.
    """

    def __init__(
        self,
        summarizer: Any,
        *,
        threshold_tokens: int,
        target_tokens: int,
        summary_size_tokens: int = 500,
        system_prompt: str | None = None,
        tool_schemas: list[Any] | None = None,
        summary_prompt_template: str = DEFAULT_SUMMARY_PROMPT,
    ) -> None:
        """`summarizer` must duck-type to a langchain chat model:
        `.invoke(messages)` returning an object with a `.content` str attribute.
        """
        if target_tokens >= threshold_tokens:
            raise ValueError("target_tokens must be < threshold_tokens")
        if summary_size_tokens >= target_tokens:
            raise ValueError("summary_size_tokens must be < target_tokens")

        # Hard-cap the summarizer's output to `summary_size_tokens` (provider tokens,
        # measured slightly differently than our placeholder; close enough as a hard
        # upper bound). `.bind()` is on Runnable so works for all langchain chat models;
        # fake models accept and ignore the kwarg.
        try:
            self._summarizer = summarizer.bind(max_tokens=summary_size_tokens)
        except Exception:
            self._summarizer = summarizer
        self._threshold = threshold_tokens
        self._target = target_tokens
        self._summary_size = summary_size_tokens
        self._summary_prompt_template = summary_prompt_template

        self._system_prompt = system_prompt
        self._tool_schemas = tool_schemas or []
        # Static-token cache: invalidated when (prompt, schemas) hash changes.
        self._static_cache: tuple[int, int] | None = None

    # -- public --

    def before_model(self, state: Any, runtime: Any) -> dict[str, Any] | None:
        messages: list[BaseMessage] = list(state.get("messages") or [])
        if not messages:
            return None

        # The CURRENT TURN should remain untouched. Find its boundary using the last message's
        # dimos_turn tag and protect everything from there to the end.
        # This preserves in-flight context: the latest user query, any in-progress tool calls / tool responses,
        # fresh images from perception, etc.
        current_start = self._current_turn_start(messages)
        compactable = messages[:current_start]
        current_turn = messages[current_start:]

        static = self._static_tokens()
        compactable_tokens = sum(count_message_tokens(m) for m in compactable)
        current_turn_tokens = sum(count_message_tokens(m) for m in current_turn)
        total = static + compactable_tokens + current_turn_tokens

        if total <= self._threshold:
            return None

        if not compactable:
            logger.warning(
                "Compaction over threshold but everything is in the current turn; "
                "passing through. Check compaction settings.",
                total_tokens=total,
            )
            return None

        # Stage 1: strip images ONLY in the compactable region.
        stripped = self._strip_images(compactable)
        stripped_tokens = sum(count_message_tokens(m) for m in stripped)
        total_after_strip = static + stripped_tokens + current_turn_tokens
        if total_after_strip <= self._target:
            logger.info(
                "Compaction fired (image-strip).",
                tokens_before=total,
                tokens_after=total_after_strip,
            )
            return {
                "messages": [
                    RemoveMessage(id=REMOVE_ALL_MESSAGES),
                    *stripped,
                    *current_turn,
                ]
            }

        # Stage 2: split compactable into protected / to_summarize / keep_tail.
        # Budget for the keep_tail accounts for the fixed cost of the current turn.
        budget = max(
            0,
            self._target - self._summary_size - current_turn_tokens,
        )
        protected, to_summarize, keep = self._split(stripped, budget=budget)
        if not to_summarize:
            logger.warning(
                "Compaction over threshold but nothing eligible to summarize; passing through.",
                total_tokens=total_after_strip,
            )
            return None

        summary_text = self._summarize(to_summarize)
        summary_msg = self._build_summary_message(summary_text, to_summarize)
        summary_tokens = count_message_tokens(summary_msg)
        final_total = (
            static
            + sum(count_message_tokens(m) for m in protected)
            + summary_tokens
            + sum(count_message_tokens(m) for m in keep)
            + current_turn_tokens
        )
        logger.info(
            "Compaction fired (summarize).",
            tokens_before=total,
            tokens_after=final_total,
            summarized_messages=len(to_summarize),
        )
        return {
            "messages": [
                RemoveMessage(id=REMOVE_ALL_MESSAGES),
                *protected,
                summary_msg,
                *keep,
                *current_turn,
            ]
        }

    # -- internals --

    def _total_tokens(self, messages: list[BaseMessage]) -> int:
        return self._static_tokens() + sum(count_message_tokens(m) for m in messages)

    def _static_tokens(self) -> int:
        """Tokens for the system prompt + tool schemas.

        Computed once and cached forever — both inputs are bound at `__init__`
        and never mutate, so there's no need to recompute (or even rehash) on
        subsequent calls.
        """
        if self._static_cache is not None:
            return self._static_cache[1]
        total = count_tokens(self._system_prompt or "")
        if self._tool_schemas:
            total += count_tokens(json.dumps(self._tool_schemas, default=str))
        self._static_cache = (0, total)  # sentinel key; payload is immutable
        return total

    def _strip_images(self, messages: list[BaseMessage]) -> list[BaseMessage]:
        """Return a new list where image content blocks are replaced with text.

        Messages without images are reused by reference. Messages with images
        are reconstructed via `model_copy` so every other field is preserved —
        `id`, `name`, `tool_calls`, `tool_call_id`, `response_metadata`, etc.
        (Plain `m.__class__(content=..., additional_kwargs=...)` would drop
        them, which silently breaks AIMessages-with-tool_calls and ToolMessages
        — the latter requires `tool_call_id` and would refuse to construct.)
        """
        out: list[BaseMessage] = []
        for m in messages:
            if not _has_image(m.content):
                out.append(m)
                continue

            new_blocks: list[Any] = []
            for block in m.content:  # type: ignore[union-attr]
                if isinstance(block, dict) and block.get("type") in (
                    "image_url",
                    "image",
                ):
                    new_blocks.append({"type": "text", "text": IMAGE_PLACEHOLDER})
                else:
                    new_blocks.append(block)

            new_kwargs = dict(m.additional_kwargs or {})
            new_kwargs.pop("dimos_tokens", None)
            new_msg = m.model_copy(update={"content": new_blocks, "additional_kwargs": new_kwargs})
            out.append(new_msg)
        return out

    def _current_turn_start(self, messages: list[BaseMessage]) -> int:
        """Return the index where the 'current turn' begins.

        The current turn is the contiguous suffix of `messages` that all share
        the highest `dimos_turn` value (plus any trailing untagged messages,
        which are typically in-flight tool calls / responses not yet stamped).
        Everything from this index to the end is preserved verbatim by the
        middleware: no image stripping, no summarization.

        Untagged-history fallback: when no message carries a `dimos_turn` tag
        at all (i.e., a caller wired this middleware in without going through
        McpClient), we anchor on the latest `HumanMessage` — treating it as
        the start of the current turn. Messages before that point are
        eligible for compaction; the latest user input plus any in-flight
        assistant / tool messages after it are protected. If no
        `HumanMessage` exists at all (unusual), we fall back to protecting
        just the last message.
        """
        max_turn: int | None = None
        for m in messages:
            t = (m.additional_kwargs or {}).get("dimos_turn")
            if isinstance(t, int) and (max_turn is None or t > max_turn):
                max_turn = t

        if max_turn is None:
            # No tags. Walk back to find the latest HumanMessage and treat
            # everything from there to the end as the current turn.
            for i in range(len(messages) - 1, -1, -1):
                if isinstance(messages[i], HumanMessage):
                    return i
            return max(0, len(messages) - 1)

        # Walk back from end: anything with dimos_turn == max_turn OR untagged
        # is part of the current turn. Stop at the first message tagged with a
        # turn < max_turn.
        cut = 0
        for i in range(len(messages) - 1, -1, -1):
            t = (messages[i].additional_kwargs or {}).get("dimos_turn")
            if isinstance(t, int) and t < max_turn:
                cut = i + 1
                break
        return cut

    def _split(
        self, messages: list[BaseMessage], *, budget: int
    ) -> tuple[list[BaseMessage], list[BaseMessage], list[BaseMessage]]:
        """Partition messages into (protected_prefix, to_summarize, keep_tail).

        - protected_prefix: leading SystemMessages WITHOUT additional_kwargs[
          "dimos_compacted"]=True. Preserved verbatim.
        - keep_tail: built back-to-front until adding the next message would
          exceed (budget - protected_tokens). If the cut would split a
          `dimos_turn` group, push it older so the entire turn falls on one side.
        - to_summarize: everything in between.
        """
        protected: list[BaseMessage] = []
        rest_start = 0
        for i, m in enumerate(messages):
            if isinstance(m, SystemMessage) and not (m.additional_kwargs or {}).get(
                "dimos_compacted"
            ):
                protected.append(m)
                rest_start = i + 1
            else:
                break
        rest = messages[rest_start:]

        protected_tokens = sum(count_message_tokens(m) for m in protected)
        budget = max(0, budget - protected_tokens)

        # Build keep_tail from the end, walking older until budget exhausted.
        keep: list[BaseMessage] = []
        used = 0
        keep_start_idx = len(rest)  # rest[keep_start_idx:] is what we keep
        for i in range(len(rest) - 1, -1, -1):
            m_tokens = count_message_tokens(rest[i])
            if used + m_tokens > budget and keep:
                # Adding this would overflow; stop. (Always keep at least one
                # message even if a single message exceeds budget, so the agent
                # gets the latest user input.)
                break
            keep.append(rest[i])
            used += m_tokens
            keep_start_idx = i
        keep.reverse()

        # Align the cut to a dimos_turn boundary so tagged tool_call/tool_response
        # pairs aren't split. The kept tail starts at keep_start_idx; if the
        # message there is tagged AND a message just before it shares the same
        # turn, push the cut older to include all messages of that turn.
        if 0 < keep_start_idx < len(rest):
            border_turn = (rest[keep_start_idx].additional_kwargs or {}).get("dimos_turn")
            if border_turn is not None:
                while keep_start_idx > 0:
                    prev_turn = (rest[keep_start_idx - 1].additional_kwargs or {}).get("dimos_turn")
                    if prev_turn != border_turn:
                        break
                    keep_start_idx -= 1
                keep = rest[keep_start_idx:]

        to_summarize = rest[:keep_start_idx]
        return protected, to_summarize, keep

    def _summarize(self, messages: list[BaseMessage]) -> str:
        transcript = _render_transcript(messages)
        prompt = self._summary_prompt_template.format(
            transcript=transcript, summary_size_tokens=self._summary_size
        )
        # `cast` because @retry erases the return type to Any.
        return cast("str", self._invoke_summarizer(prompt))

    @retry(max_retries=2, on_exception=Exception, delay=0.5)  # type: ignore[untyped-decorator]
    def _invoke_summarizer(self, prompt: str) -> str:
        """LLM call, isolated for retry. Raises on final failure (propagates)."""
        response = self._summarizer.invoke([HumanMessage(content=prompt)])
        text = getattr(response, "content", None)
        if isinstance(text, str):
            stripped = text.strip()
            if stripped:
                return stripped
        # Some fakes return raw strings or empty content; coerce.
        if isinstance(text, list):
            joined = " ".join(b.get("text", "") for b in text if isinstance(b, dict))
            if joined.strip():
                return joined.strip()
        raise RuntimeError(f"Summarizer returned empty content: {response!r}")

    def _build_summary_message(
        self, summary_text: str, summarized: list[BaseMessage]
    ) -> SystemMessage:
        max_turn: int | None = None
        for m in summarized:
            t = (m.additional_kwargs or {}).get("dimos_turn")
            if isinstance(t, int) and (max_turn is None or t > max_turn):
                max_turn = t

        kw: dict[str, Any] = {
            "dimos_compacted": True,
            "dimos_covers_count": len(summarized),
        }
        if max_turn is not None:
            kw["dimos_turn"] = max_turn

        return SystemMessage(
            content=f"[Prior conversation summary]\n{summary_text}",
            additional_kwargs=kw,
        )


def _stringify_content(content: Any) -> str:
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        parts: list[str] = []
        for block in content:
            if isinstance(block, dict):
                if block.get("type") == "text":
                    parts.append(block.get("text", ""))
                else:
                    parts.append(f"<{block.get('type', 'unknown')}>")
            else:
                parts.append(str(block))
        return " ".join(parts)
    return str(content)


def _render_transcript(messages: list[BaseMessage]) -> str:
    lines: list[str] = []
    for m in messages:
        role = type(m).__name__.replace("Message", "").lower()
        turn = (m.additional_kwargs or {}).get("dimos_turn", "?")
        content = _stringify_content(m.content)
        if isinstance(m, AIMessage):
            tool_calls = getattr(m, "tool_calls", None) or []
            if tool_calls:
                tc_summary = "; ".join(f"{tc.get('name')}({tc.get('args')})" for tc in tool_calls)
                content = (content + f"\n  tool_calls: {tc_summary}").strip()
        lines.append(f"[turn {turn} {role}] {content}")
    return "\n".join(lines)
