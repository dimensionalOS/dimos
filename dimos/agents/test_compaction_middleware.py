# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Tests for `DimosCompactionMiddleware`.

Hermetic — uses langchain's `FakeListChatModel` / `FakeMessagesListChatModel`
(subclassed to record inputs) so no API key is ever needed. Covers both unit
tests of `before_model` and full-loop integration tests where the middleware
runs inside a real `create_agent` graph.
"""

from __future__ import annotations

from typing import Any, cast

from langchain.agents import create_agent
from langchain_core.language_models.fake_chat_models import (
    FakeListChatModel,
    FakeMessagesListChatModel,
)
from langchain_core.messages import (
    AIMessage,
    BaseMessage,
    HumanMessage,
    RemoveMessage,
    SystemMessage,
    ToolMessage,
)
from langchain_core.tools import tool
from langgraph.graph.message import REMOVE_ALL_MESSAGES
from pydantic import Field
import pytest

from dimos.agents.compaction_middleware import (
    CHARS_PER_TOKEN,
    IMAGE_PLACEHOLDER,
    TOKENS_PER_IMAGE,
    DimosCompactionMiddleware,
    count_image_tokens,
    count_message_tokens,
    count_tokens,
)
from dimos.agents.mcp.mcp_client import _tag_turn


def make_human(text: str, turn: int) -> HumanMessage:
    m = HumanMessage(content=text)
    _tag_turn(m, turn)
    return m


def make_ai(text: str, turn: int, *, tool_calls: list[dict[str, Any]] | None = None) -> AIMessage:
    m = AIMessage(content=text, tool_calls=tool_calls or [])
    _tag_turn(m, turn)
    return m


def make_tool(text: str, tool_call_id: str, turn: int) -> ToolMessage:
    m = ToolMessage(content=text, tool_call_id=tool_call_id)
    _tag_turn(m, turn)
    return m


def make_image_human(turn: int) -> HumanMessage:
    m = HumanMessage(
        content=[
            {"type": "text", "text": "see this"},
            {"type": "image_url", "image_url": {"url": "data:image/png;base64,..."}},
        ]
    )
    _tag_turn(m, turn)
    return m


def build_text_history(n_turns: int, text_per_turn: str = "x" * 60) -> list[BaseMessage]:
    """SystemMessage prefix + n turns of (Human, AI) pairs, each tagged."""
    history: list[BaseMessage] = [SystemMessage(content="You are a test agent.")]
    for i in range(1, n_turns + 1):
        history.append(make_human(f"q{i}: {text_per_turn}", i))
        history.append(make_ai(f"a{i}: {text_per_turn}", i))
    return history


def state(messages: list[BaseMessage]) -> dict[str, Any]:
    return {"messages": messages}


class CountingFake(FakeListChatModel):
    """Langchain's FakeListChatModel + a side-list of every prompt it saw.

    Subclassing (rather than wrapping) is the only way to extend a pydantic v2
    model with new instance state. The mutable list-as-field is safe because
    `append()` mutates in place — no attribute reassignment.
    """

    received: list[str] = Field(default_factory=list)

    def invoke(self, input: Any, *args: Any, **kwargs: Any) -> Any:  # type: ignore[override]
        if isinstance(input, list):
            text = "\n".join(str(getattr(m, "content", "")) for m in input)
        else:
            text = str(input)
        self.received.append(text)
        return super().invoke(input, *args, **kwargs)


def make_counting_fake(responses: list[str]) -> tuple[CountingFake, list[str]]:
    m = CountingFake(responses=responses)
    return m, m.received


def test_token_counter_text() -> None:
    s = "x" * 30
    assert count_tokens(s) == 30 // CHARS_PER_TOKEN  # 10
    m = HumanMessage(content=s)
    assert count_message_tokens(m) == 10
    # memoized
    assert m.additional_kwargs.get("dimos_tokens") == 10
    # second call uses the memo (we verify by mutating the memo and seeing it returned)
    m.additional_kwargs["dimos_tokens"] = 999
    assert count_message_tokens(m) == 999


def test_token_counter_image() -> None:
    m = make_image_human(1)
    n = count_message_tokens(m)
    assert n == count_image_tokens() + count_tokens("see this")
    assert n == TOKENS_PER_IMAGE + 3  # 8 chars / 3 = 3 (rounded up)


def test_static_tokens_cached() -> None:
    mw = DimosCompactionMiddleware(
        summarizer=CountingFake(responses=["UNUSED"]),
        threshold_tokens=10_000,
        target_tokens=5_000,
        summary_size_tokens=500,
        system_prompt="you are a test agent",
        tool_schemas=[{"name": "echo", "args": {"text": "str"}}],
    )
    a = mw._static_tokens()
    b = mw._static_tokens()
    assert a == b
    assert mw._static_cache is not None  # cache populated


def test_below_threshold_is_noop() -> None:
    history = build_text_history(n_turns=2)
    fake, received = make_counting_fake(["UNUSED"])
    mw = DimosCompactionMiddleware(
        summarizer=fake,
        threshold_tokens=10_000,
        target_tokens=5_000,
        summary_size_tokens=500,
    )
    result = mw.before_model(state(history), runtime=None)
    assert result is None
    assert received == []  # summarizer never called


def test_image_stripping_alone_suffices() -> None:
    """An image in an OLDER turn pushes us over; stripping it brings us back under.

    The image must live outside the current turn — the current turn is sacred
    and never gets compacted.
    """
    history: list[BaseMessage] = [SystemMessage(content="sys")]
    history.append(make_human("small msg 1", 1))
    history.append(make_image_human(1))  # ~1003 tokens, in OLD turn 1
    history.append(make_ai("reply 1", 1))
    # Current (latest) turn — protected.
    history.append(make_human("small msg 2", 2))
    history.append(make_ai("reply 2", 2))

    fake, received = make_counting_fake(["UNUSED"])
    mw = DimosCompactionMiddleware(
        summarizer=fake,
        threshold_tokens=500,
        target_tokens=300,
        summary_size_tokens=50,
    )
    result = mw.before_model(state(history), runtime=None)
    assert result is not None
    new_msgs = result["messages"]
    assert isinstance(new_msgs[0], RemoveMessage)
    assert new_msgs[0].id == REMOVE_ALL_MESSAGES

    # The image was replaced with the placeholder.
    new_history = new_msgs[1:]
    found_placeholder = False
    for m in new_history:
        if isinstance(m.content, list):
            for block in m.content:
                if isinstance(block, dict) and block.get("text") == IMAGE_PLACEHOLDER:
                    found_placeholder = True
    assert found_placeholder
    # Summarizer was NOT called — stage 1 alone was enough.
    assert received == []


def test_summarize_when_image_strip_insufficient() -> None:
    history = build_text_history(n_turns=10, text_per_turn="y" * 200)
    fake, received = make_counting_fake(["[summary]"])
    mw = DimosCompactionMiddleware(
        summarizer=fake,
        threshold_tokens=400,
        target_tokens=200,
        summary_size_tokens=20,
    )
    result = mw.before_model(state(history), runtime=None)
    assert result is not None
    new_history = result["messages"][1:]  # skip RemoveMessage sentinel

    # Exactly one summary message present, marked dimos_compacted.
    summaries = [
        m
        for m in new_history
        if isinstance(m, SystemMessage) and (m.additional_kwargs or {}).get("dimos_compacted")
    ]
    assert len(summaries) == 1
    assert "[summary]" in summaries[0].content
    assert isinstance(summaries[0].additional_kwargs["dimos_turn"], int)
    assert len(received) == 1


def test_post_compaction_state_under_threshold_with_large_static() -> None:
    """Stage-2 budget must subtract `static` (system prompt + tool schemas).

    Regression: without that subtraction the kept tail fills to roughly
    `target - summary_size - current_turn`, so the post-compaction total
    lands at ~`static + target`. When `static > threshold - target` (large
    tool schemas with a tight target/threshold gap), that total is still
    above threshold — the next `before_model` re-fires compaction and the
    loop never terminates.

    Setup forces `static > threshold - target` to expose the bug, then
    asserts the post-state is below threshold (i.e. compaction terminates).
    """
    history = build_text_history(n_turns=30, text_per_turn="y" * 200)
    # Inflate the static overhead with a big system prompt + tool schemas.
    big_system_prompt = "S" * 2_400
    big_tool_schemas = [{"name": "tool", "schema": "T" * 1_500}]
    fake, _received = make_counting_fake(["[summary]"])
    mw = DimosCompactionMiddleware(
        summarizer=fake,
        threshold_tokens=5_000,
        target_tokens=4_000,
        summary_size_tokens=100,
        system_prompt=big_system_prompt,
        tool_schemas=big_tool_schemas,
    )
    static_tokens = mw._static_tokens()
    # Bug condition: static larger than the threshold/target gap. Without it,
    # `static + target ≤ threshold` and the loop wouldn't trigger even with
    # the broken budget calc.
    assert static_tokens > mw._threshold - mw._target

    result = mw.before_model(state(history), runtime=None)
    assert result is not None
    new_msgs = result["messages"][1:]  # drop the REMOVE_ALL_MESSAGES sentinel
    post_total = static_tokens + sum(count_message_tokens(m) for m in new_msgs)
    # The compaction contract: the post-state must land below threshold,
    # else the next before_model call sees over-threshold again. Asserting
    # against `threshold` rather than the (no-op) re-trigger result — when
    # the state lands above threshold but with everything wedged into the
    # current turn or already-summarized prefix, before_model returns None
    # (warns "nothing eligible to summarize") and the bug is silent.
    assert post_total <= mw._threshold, (
        f"post-compaction total {post_total} exceeds threshold {mw._threshold}; "
        f"the next model call sees an over-budget prompt"
    )


def test_protected_prefix_preserved() -> None:
    history = build_text_history(n_turns=10, text_per_turn="z" * 200)
    sys_msg = history[0]
    fake, received = make_counting_fake(["[summary]"])
    mw = DimosCompactionMiddleware(
        summarizer=fake,
        threshold_tokens=400,
        target_tokens=200,
        summary_size_tokens=20,
    )
    result = mw.before_model(state(history), runtime=None)
    assert result is not None
    new_history = result["messages"][1:]
    # The original SystemMessage is at index 0 of the new history, unmodified.
    assert new_history[0] is sys_msg


def test_untagged_midlist_is_summarized() -> None:
    """A hand-injected, untagged message in the middle gets folded into the summary."""
    injected_marker = "MIDLIST-UNTAGGED-XYZ"

    history: list[BaseMessage] = [SystemMessage(content="sys")]
    for i in range(1, 6):
        history.append(make_human(f"q{i} " + "x" * 100, i))
        history.append(make_ai(f"a{i} " + "x" * 100, i))
    # Inject an untagged HumanMessage between turn 5 and 6.
    history.append(HumanMessage(content=injected_marker + " " + "x" * 100))
    for i in range(6, 11):
        history.append(make_human(f"q{i} " + "x" * 100, i))
        history.append(make_ai(f"a{i} " + "x" * 100, i))

    fake, received = make_counting_fake(["[summary]"])
    mw = DimosCompactionMiddleware(
        summarizer=fake,
        threshold_tokens=400,
        target_tokens=200,
        summary_size_tokens=20,
    )
    result = mw.before_model(state(history), runtime=None)
    assert result is not None

    # The injected marker appeared in what was sent to the summarizer.
    assert any(injected_marker in p for p in received)

    # And the injected message is NOT in the final history (it was folded in).
    new_history = result["messages"][1:]
    for m in new_history:
        if isinstance(m.content, str):
            assert injected_marker not in m.content


def test_prior_summary_is_resummarized() -> None:
    """A previous compaction's SystemMessage (dimos_compacted=True) folds into the next."""
    prior = SystemMessage(
        content="[Prior conversation summary]\nUser asked older things.",
        additional_kwargs={"dimos_compacted": True, "dimos_turn": 3},
    )

    history: list[BaseMessage] = [SystemMessage(content="sys"), prior]
    for i in range(4, 14):
        history.append(make_human(f"q{i} " + "x" * 200, i))
        history.append(make_ai(f"a{i} " + "x" * 200, i))

    fake, received = make_counting_fake(["[new summary]"])
    mw = DimosCompactionMiddleware(
        summarizer=fake,
        threshold_tokens=400,
        target_tokens=200,
        summary_size_tokens=20,
    )
    result = mw.before_model(state(history), runtime=None)
    assert result is not None

    # The prior summary's content was passed to the summarizer.
    assert any("older things" in p for p in received)

    # Exactly one compacted SystemMessage remains.
    new_history = result["messages"][1:]
    compacted = [
        m
        for m in new_history
        if isinstance(m, SystemMessage) and (m.additional_kwargs or {}).get("dimos_compacted")
    ]
    assert len(compacted) == 1
    assert "[new summary]" in compacted[0].content


def test_recent_turns_kept_verbatim() -> None:
    history = build_text_history(n_turns=10, text_per_turn="w" * 200)
    # Capture object identity of the last few messages.
    tail_ids = {id(m) for m in history[-4:]}

    fake, received = make_counting_fake(["[summary]"])
    mw = DimosCompactionMiddleware(
        summarizer=fake,
        threshold_tokens=400,
        target_tokens=300,
        summary_size_tokens=20,
    )
    result = mw.before_model(state(history), runtime=None)
    assert result is not None
    new_history = result["messages"][1:]
    # At least one of the recent messages should be in the new history by identity.
    assert any(id(m) in tail_ids for m in new_history)


def test_tool_call_pair_coherence() -> None:
    """Tool-call and tool-response with same dimos_turn are never split by the cut."""
    history: list[BaseMessage] = [SystemMessage(content="sys")]
    # Many fluffy turns to push us well over.
    for i in range(1, 8):
        history.append(make_human(f"q{i} " + "x" * 200, i))
        history.append(make_ai(f"a{i} " + "x" * 200, i))
    # A target turn with the tool-call pair.
    target_turn = 8
    history.append(make_human("invoke add", target_turn))
    history.append(
        make_ai(
            "",
            target_turn,
            tool_calls=[{"name": "add", "args": {"a": 1, "b": 2}, "id": "call_xyz"}],
        )
    )
    history.append(make_tool("3", "call_xyz", target_turn))
    history.append(make_ai("1 + 2 = 3", target_turn))
    # And more after.
    for i in range(9, 12):
        history.append(make_human(f"q{i} " + "x" * 200, i))
        history.append(make_ai(f"a{i} " + "x" * 200, i))

    fake, received = make_counting_fake(["[summary]"])
    mw = DimosCompactionMiddleware(
        summarizer=fake,
        threshold_tokens=400,
        target_tokens=250,
        summary_size_tokens=20,
    )
    result = mw.before_model(state(history), runtime=None)
    assert result is not None
    new_history = result["messages"][1:]

    # Find all messages of the target_turn that survived.
    survivors_of_target_turn = [
        m for m in new_history if (m.additional_kwargs or {}).get("dimos_turn") == target_turn
    ]
    # Either: all 4 messages of that turn are in the kept tail,
    # or: none of them are (they were summarized together).
    assert len(survivors_of_target_turn) in (0, 4)


def test_untagged_history_anchors_current_turn_on_latest_human() -> None:
    """If the input has no dimos_turn tags at all, the fallback treats the
    latest HumanMessage as the start of the current turn — its content (and
    anything emitted after it) is protected; older messages are compactable.
    """
    # Manually-built history with NO turn tags anywhere.
    history: list[BaseMessage] = [
        SystemMessage(content="You are a test agent."),
        HumanMessage(content="old q " + "x" * 500),
        AIMessage(content="old a " + "x" * 500),
        HumanMessage(content="old q2 " + "x" * 500),
        AIMessage(content="old a2 " + "x" * 500),
        HumanMessage(content="LATEST_USER_INPUT_UNIQUE_MARKER"),
        AIMessage(
            content="",
            tool_calls=[{"name": "echo", "args": {"text": "x"}, "id": "c1"}],
        ),
    ]
    fake, received = make_counting_fake(["[summary]"])
    mw = DimosCompactionMiddleware(
        summarizer=fake,
        threshold_tokens=400,
        target_tokens=200,
        summary_size_tokens=40,
    )
    result = mw.before_model(state(history), runtime=None)
    assert result is not None, "should compact: total is over threshold"

    new_history = result["messages"][1:]  # skip RemoveMessage sentinel
    # The latest HumanMessage is preserved verbatim in the kept tail.
    assert any(
        isinstance(m, HumanMessage)
        and isinstance(m.content, str)
        and "LATEST_USER_INPUT_UNIQUE_MARKER" in m.content
        for m in new_history
    )
    # The trailing AIMessage(tool_call) is also preserved.
    assert any(
        isinstance(m, AIMessage) and (getattr(m, "tool_calls", None) or [{}])[0].get("id") == "c1"
        for m in new_history
    )
    # And the latest HumanMessage's content was NOT sent to the summarizer.
    assert not any("LATEST_USER_INPUT_UNIQUE_MARKER" in p for p in received), (
        "latest human input must not be summarized away"
    )


def test_summarize_failure_propagates() -> None:
    class BoomFake(FakeListChatModel):
        def invoke(self, *args: Any, **kwargs: Any) -> Any:  # type: ignore[override]
            raise RuntimeError("boom")

    history = build_text_history(n_turns=10, text_per_turn="x" * 200)
    mw = DimosCompactionMiddleware(
        summarizer=BoomFake(responses=["never used"]),
        threshold_tokens=400,
        target_tokens=200,
        summary_size_tokens=20,
    )
    with pytest.raises(RuntimeError, match="boom"):
        mw.before_model(state(history), runtime=None)


def test_recompaction_folds_prior_summary() -> None:
    history = build_text_history(n_turns=10, text_per_turn="x" * 200)
    fake, received = make_counting_fake(["[s1]", "[s2]"])
    mw = DimosCompactionMiddleware(
        summarizer=fake,
        threshold_tokens=400,
        target_tokens=200,
        summary_size_tokens=20,
    )
    r1 = mw.before_model(state(history), runtime=None)
    assert r1 is not None
    after1 = r1["messages"][1:]
    # Add more turns and run again.
    n_so_far = max(
        (
            (m.additional_kwargs or {}).get("dimos_turn", 0)
            for m in after1
            if isinstance((m.additional_kwargs or {}).get("dimos_turn"), int)
        ),
        default=0,
    )
    extended = list(after1)
    for i in range(n_so_far + 1, n_so_far + 11):
        extended.append(make_human(f"q{i} " + "x" * 200, i))
        extended.append(make_ai(f"a{i} " + "x" * 200, i))

    r2 = mw.before_model(state(extended), runtime=None)
    assert r2 is not None
    after2 = r2["messages"][1:]
    compacted = [
        m
        for m in after2
        if isinstance(m, SystemMessage) and (m.additional_kwargs or {}).get("dimos_compacted")
    ]
    # Still exactly one compacted summary (the new one rolled the old in).
    assert len(compacted) == 1
    # And [s1] was visible to the second summarizer call.
    assert any("[s1]" in p for p in received[1:])


class RecordingFakeAgent(FakeMessagesListChatModel):
    """Fake agent chat model that records each `.invoke()`'s input messages.

    Subclassing FakeMessagesListChatModel keeps tool_call response support
    while letting us inspect what the agent node saw at every step. The base
    class raises on `bind_tools()`; we no-op it because the fake doesn't
    actually need tool schemas — it returns predetermined responses.
    """

    received_inputs: list[list[BaseMessage]] = Field(default_factory=list)

    def bind_tools(self, tools: Any, **kwargs: Any) -> Any:  # type: ignore[override]
        return self

    def invoke(self, input: Any, *args: Any, **kwargs: Any) -> Any:  # type: ignore[override]
        if isinstance(input, list):
            self.received_inputs.append(list(input))
        return super().invoke(input, *args, **kwargs)


@tool
def echo(text: str) -> str:
    """Echo back the given text."""
    return text


@tool
def get_big_result() -> str:
    """Return a chunk of text big enough to push history over a small threshold."""
    return "BIG_RESULT_LINE " + ("x" * 800)  # ~270 tokens with the chars/3 placeholder


def test_full_loop_compaction_fires_inside_create_agent() -> None:
    """Real `create_agent` loop: middleware fires, langgraph reducer honors
    `RemoveMessage(REMOVE_ALL_MESSAGES)`, and the final state contains the
    summary plus the agent's appended response.
    """
    agent_model = RecordingFakeAgent(responses=[AIMessage(content="Acknowledged.")])
    summarizer = CountingFake(responses=["FAKE_SUMMARY"])

    history: list[BaseMessage] = [SystemMessage(content="You are a test agent.")]
    for i in range(1, 9):
        history.append(make_human(f"q{i} " + "x" * 150, i))
        history.append(make_ai(f"a{i} " + "x" * 150, i))
    history.append(make_human("now please respond", 9))

    mw = DimosCompactionMiddleware(
        summarizer=summarizer,
        threshold_tokens=400,
        target_tokens=200,
        summary_size_tokens=40,
        system_prompt="test agent",
    )
    graph: Any = create_agent(
        model=agent_model,
        tools=[echo],
        middleware=[mw],
    )

    result = graph.invoke(cast("Any", {"messages": history}))
    final_messages = result["messages"]

    # 1. The agent's response was appended (loop ran to completion).
    assert any(isinstance(m, AIMessage) and m.content == "Acknowledged." for m in final_messages)

    # 2. A compaction summary message exists in the final state.
    summaries = [
        m
        for m in final_messages
        if isinstance(m, SystemMessage) and (m.additional_kwargs or {}).get("dimos_compacted")
    ]
    assert len(summaries) == 1
    assert "FAKE_SUMMARY" in summaries[0].content

    # 3. The summarizer was invoked exactly once (proves compaction actually fired).
    assert len(summarizer.received) == 1

    # 4. The agent node received a *compacted* prompt — early turns are gone
    #    from what the model saw, summary is present, current turn is intact.
    assert len(agent_model.received_inputs) == 1
    prompt_seen = agent_model.received_inputs[0]
    contents = " | ".join(
        m.content if isinstance(m.content, str) else str(m.content) for m in prompt_seen
    )
    assert "q1 " not in contents, "earliest turn should have been summarized away"
    assert "FAKE_SUMMARY" in contents, "summary should be in the agent's input"
    assert "now please respond" in contents, "current turn must reach the model"

    # 5. Old turns are gone from the final state too (reducer wiped them via
    #    the REMOVE_ALL_MESSAGES sentinel).
    final_contents = " | ".join(
        m.content if isinstance(m.content, str) else str(m.content) for m in final_messages
    )
    assert "q1 " not in final_contents
    assert "q2 " not in final_contents


def test_compaction_fires_between_tool_call_and_final_answer() -> None:
    """Multi-step turn: model → tool_call → tool result → model again.

    The pre-tool state is under threshold (no compaction on first `before_model`).
    The tool returns a chunk big enough that the SECOND `before_model` is over
    threshold and must compact. Proves the "fires before every model call"
    invariant — the property that motivates doing this as middleware at all.
    """
    agent_model = RecordingFakeAgent(
        responses=[
            AIMessage(
                content="",
                tool_calls=[{"name": "get_big_result", "args": {}, "id": "call_x"}],
            ),
            AIMessage(content="Tool reply received."),
        ]
    )
    summarizer = CountingFake(responses=["MID_TURN_SUMMARY"])

    # Pre-load enough older history that we're CLOSE to threshold but under it.
    # Adding the tool result will push us over and force compaction on the 2nd call.
    history: list[BaseMessage] = [SystemMessage(content="You are a test agent.")]
    for i in range(1, 4):
        history.append(make_human(f"q{i} " + "x" * 100, i))
        history.append(make_ai(f"a{i} " + "x" * 100, i))
    history.append(make_human("call the tool", 4))

    mw = DimosCompactionMiddleware(
        summarizer=summarizer,
        threshold_tokens=350,
        target_tokens=180,
        summary_size_tokens=40,
        system_prompt="test agent",
    )
    graph: Any = create_agent(
        model=agent_model,
        tools=[get_big_result],
        middleware=[mw],
    )

    result = graph.invoke(cast("Any", {"messages": history}))
    final_messages = result["messages"]

    # 1. Model was called twice (tool_call round-trip + final answer).
    assert len(agent_model.received_inputs) == 2

    # 2. First call: NO compaction yet. The agent's first prompt should still
    #    contain "q1" because we haven't crossed threshold yet.
    first_contents = " | ".join(
        m.content if isinstance(m.content, str) else str(m.content)
        for m in agent_model.received_inputs[0]
    )
    assert "q1 " in first_contents, "first model call should see uncompacted history"

    # 3. Second call: compaction DID fire (after the tool result inflated state).
    second_contents = " | ".join(
        m.content if isinstance(m.content, str) else str(m.content)
        for m in agent_model.received_inputs[1]
    )
    assert "MID_TURN_SUMMARY" in second_contents, (
        "second model call should see the summary — compaction must fire between "
        "tool result and next model call"
    )
    assert "q1 " not in second_contents, "second call should not see compacted-away turn 1"

    # 4. Summarizer was invoked exactly once (the second before_model triggered it).
    assert len(summarizer.received) == 1

    # 5. Final answer was appended.
    assert any(
        isinstance(m, AIMessage) and m.content == "Tool reply received." for m in final_messages
    )
