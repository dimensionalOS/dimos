from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from dimos.agents.utils import estimate_tokens
from langchain_core.messages import trim_messages

def test_estimate_tokens():
    msgs = [
        SystemMessage(content="system prompt"),
        HumanMessage(content="Hello world!"),  # length 12
        AIMessage(content="", tool_calls=[{"name": "test_tool", "args": {}, "id": "call_123"}]) # 1 tool call
    ]
    
    # 13 char / 4 = 3 + 10 = 13 (System)
    # 12 char / 4 = 3 + 10 = 13 (Human)
    # 0 char / 4 = 0 + 10 = 10 + 50 (ToolCall) = 60 (AI)
    # Total ~ 86
    tokens = estimate_tokens(msgs)
    assert tokens > 0
    assert tokens < 150 # Roughly sane heuristic test

def test_trim_history_preserves_system_prompt():
    msgs = [
        SystemMessage(content="System"),
        HumanMessage(content="A" * 100),
        AIMessage(content="B" * 100),
        HumanMessage(content="C" * 100),
    ]
    
    # Need to find a threshold that drops something but keeps system
    # Each block of 100 chars is ~35 tokens (100 / 4 + 10). System is ~11 tokens.
    # Total is ~ 116.
    # Let's set limit to 60. Should keep System and "C".
    trimmed = trim_messages(
        msgs,
        max_tokens=60,
        strategy="last",
        token_counter=estimate_tokens,
        include_system=True,
        allow_partial=False
    )
    
    assert isinstance(trimmed, list)
    assert len(trimmed) == 2
    assert trimmed[0].content == "System"
    assert trimmed[1].content == "C" * 100

def test_trim_history_does_not_break_toolcalls():
    msgs = [
        SystemMessage(content="System"),
        HumanMessage(content="trigger tool"),
        # Tool call sequence
        AIMessage(content="", tool_calls=[{"name": "test_tool", "args": {}, "id": "call_123"}]),
        ToolMessage(content="tool result", tool_call_id="call_123"),
        # New prompt
        HumanMessage(content="D" * 200),
    ]
    
    # Total is around: 
    # System: ~11, Human: ~13, AI: ~60, Tool: ~12, Human: ~60. Total ~156.
    # What if we set max to 75?
    # It must keep "D" (60 tokens). It needs 15 more, but the ToolMessage + AI block is ~72 tokens,
    # so it should drop both. It should not keep just the ToolMessage or just the AIMessage.
    trimmed = trim_messages(
        msgs,
        max_tokens=75,
        strategy="last",
        token_counter=estimate_tokens,
        include_system=True,
        allow_partial=False
    )
    
    assert len(trimmed) == 2
    assert trimmed[0].content == "System"
    assert trimmed[1].content == "D" * 200
