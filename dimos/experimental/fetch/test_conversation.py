from __future__ import annotations

import asyncio
import types as builtin_types
from types import SimpleNamespace
from unittest.mock import patch

import conversation


_LOOP: asyncio.AbstractEventLoop | None = None


def _run(coro):
    """Run a coroutine on a single reused module loop.

    A sibling module (test_tts.py) uses asyncio.get_event_loop(), which raises if
    the current loop was closed/cleared. We keep one loop for this module, reuse
    it across calls, and leave it set as the current loop so siblings still work
    without orphaning a loop on every call.
    """
    global _LOOP
    if _LOOP is None or _LOOP.is_closed():
        _LOOP = asyncio.new_event_loop()
    asyncio.set_event_loop(_LOOP)
    return _LOOP.run_until_complete(coro)


def _make_session(**kwargs):
    emitted: list[dict] = []
    robot_calls: list[dict] = []

    async def emit(message):
        emitted.append(message)

    async def robot(payload):
        robot_calls.append(payload)
        return {"enabled": True, "ok": True}

    session = conversation.LiveConversationSession(
        emit=emit, robot_action=robot, **kwargs
    )
    return session, emitted, robot_calls


class _MockLiveSession:
    def __init__(self, messages):
        self._messages = messages
        self.tool_responses: list[dict] = []
        self.client_contents: list[dict] = []
        self.realtime: list[dict] = []

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        return None

    async def receive(self):
        for message in self._messages:
            yield message

    async def send_tool_response(self, **kwargs):
        self.tool_responses.append(kwargs)

    async def send_client_content(self, **kwargs):
        self.client_contents.append(kwargs)

    async def send_realtime_input(self, **kwargs):
        self.realtime.append(kwargs)


def _audio_msg(pcm):
    inline = SimpleNamespace(data=pcm)
    part = SimpleNamespace(inline_data=inline)
    model_turn = SimpleNamespace(parts=[part])
    server_content = SimpleNamespace(
        model_turn=model_turn,
        interrupted=False,
        output_transcription=None,
        input_transcription=None,
        turn_complete=False,
    )
    return SimpleNamespace(server_content=server_content, tool_call=None)


def _transcript_msg(speaker, text):
    out = SimpleNamespace(text=text) if speaker == "dog" else None
    inp = SimpleNamespace(text=text) if speaker == "user" else None
    server_content = SimpleNamespace(
        model_turn=None,
        interrupted=False,
        output_transcription=out,
        input_transcription=inp,
        turn_complete=(speaker == "dog"),
    )
    return SimpleNamespace(server_content=server_content, tool_call=None)


def _interrupt_msg():
    server_content = SimpleNamespace(
        model_turn=None,
        interrupted=True,
        output_transcription=None,
        input_transcription=None,
        turn_complete=False,
    )
    return SimpleNamespace(server_content=server_content, tool_call=None)


def _toolcall_msg(name, args, call_id):
    call = SimpleNamespace(id=call_id, name=name, args=args)
    tool_call = SimpleNamespace(function_calls=[call])
    return SimpleNamespace(server_content=None, tool_call=tool_call)


# -- tool dispatch -----------------------------------------------------------


def test_take_order_clamps_quantity_and_skips_hardware() -> None:
    session, _emitted, robot = _make_session()
    result, scheduling, finish = _run(session._dispatch_tool("take_order", {"quantity": 9}))

    assert result["order_recorded"] is True
    assert result["quantity"] == 4
    assert "back" in result["instructions"]
    assert scheduling == "WHEN_IDLE"
    assert finish is False
    assert robot == []


def test_take_order_defaults_to_one() -> None:
    session, _emitted, _robot = _make_session()
    result, _, _ = _run(session._dispatch_tool("take_order", {"quantity": "not a number"}))
    assert result["quantity"] == 1


def test_take_photo_emits_conversation_state() -> None:
    session, emitted, _robot = _make_session()
    result, scheduling, finish = _run(session._dispatch_tool("take_photo", {}))

    assert result["photo_taken"] is True
    assert scheduling == "WHEN_IDLE"
    assert finish is False
    assert any(
        m.get("type") == "conversation_state" and m.get("state") == "take_photo"
        for m in emitted
    )


def test_do_trick_invokes_robot() -> None:
    session, _emitted, robot = _make_session()
    result, _, _ = _run(session._dispatch_tool("do_trick", {"trick": "wave"}))

    assert robot == [{"action": "wave"}]
    assert result["performed"] is True
    assert result["trick"] == "wave"


def test_do_trick_rejects_unknown_trick() -> None:
    session, _emitted, robot = _make_session()
    result, _, _ = _run(session._dispatch_tool("do_trick", {"trick": "backflip"}))

    assert result["performed"] is False
    assert robot == []


def test_celebrate_dances_emits_and_signals_finish() -> None:
    session, emitted, robot = _make_session()
    result, scheduling, finish = _run(session._dispatch_tool("celebrate", {"goodbye_line": "bye"}))

    assert robot == [{"action": "dance"}]
    assert scheduling == "INTERRUPT"
    assert result["celebrated"] is True
    # _dispatch_tool requests finish via the flag but does NOT set _done itself,
    # so the caller can send the final tool response first.
    assert finish is True
    assert session.finished is False
    assert any(m.get("state") == "celebrate" for m in emitted)


def test_stop_and_reset_signals_finish() -> None:
    session, emitted, _robot = _make_session()
    result, scheduling, finish = _run(session._dispatch_tool("stop_and_reset", {"reason": "left"}))

    assert result["reset"] is True
    assert scheduling == "INTERRUPT"
    assert finish is True
    assert session.finished is False
    assert any(m.get("state") == "reset" for m in emitted)


def test_unknown_tool_returns_failure() -> None:
    session, _emitted, _robot = _make_session()
    result, scheduling, finish = _run(session._dispatch_tool("frobnicate", {}))

    assert result["success"] is False
    assert scheduling is None
    assert finish is False


def test_handle_tool_calls_finishes_after_send() -> None:
    session, emitted, _robot = _make_session()
    session._types = SimpleNamespace(FunctionResponse=lambda **kw: kw)
    mock = _MockLiveSession([])
    session._session = mock
    call = SimpleNamespace(id="c1", name="celebrate", args={"goodbye_line": "bye"})

    _run(session._handle_tool_calls([call]))

    # The terminal response was sent, and only then was the session finished.
    assert mock.tool_responses
    assert mock.tool_responses[0]["function_responses"][0]["name"] == "celebrate"
    assert session.finished is True
    assert any(m.get("state") == "celebrate" for m in emitted)


def test_run_emits_terminal_reset_when_not_finished_by_tool() -> None:
    session, emitted, _robot = _make_session(idle_timeout_s=100.0)
    session._types = SimpleNamespace()
    mock = _MockLiveSession([])  # empty receive -> recv_loop returns immediately

    async def scenario():
        session._session = mock
        await asyncio.wait_for(session.run(), timeout=3.0)

    _run(scenario())

    assert any(
        m.get("type") == "conversation_state" and m.get("state") == "reset"
        for m in emitted
    )


# -- receive loop ------------------------------------------------------------


def test_recv_loop_routes_audio_transcript_interrupt_and_tools() -> None:
    session, emitted, _robot = _make_session()
    session._types = SimpleNamespace(FunctionResponse=lambda **kw: kw)
    mock = _MockLiveSession([
        _audio_msg(b"\x01\x02\x03\x04"),
        _transcript_msg("dog", "ice cold Coke coming up"),
        _transcript_msg("user", "two please"),
        _interrupt_msg(),
        _toolcall_msg("take_order", {"quantity": 2}, "c1"),
    ])
    session._session = mock

    _run(session._recv_loop())

    types_seen = [m["type"] for m in emitted]
    assert "audio_out" in types_seen
    assert "interrupted" in types_seen
    assert any(m["type"] == "transcript" and m["speaker"] == "dog" for m in emitted)
    assert any(m["type"] == "transcript" and m["speaker"] == "user" for m in emitted)

    assert mock.tool_responses
    response = mock.tool_responses[0]["function_responses"][0]
    assert response["name"] == "take_order"
    assert response["response"]["quantity"] == 2
    assert response["scheduling"] == "WHEN_IDLE"


def test_handle_tool_calls_retries_without_scheduling() -> None:
    session, _emitted, _robot = _make_session()
    session._types = SimpleNamespace(FunctionResponse=lambda **kw: kw)

    class _PickySession:
        def __init__(self):
            self.calls: list[dict] = []

        async def send_tool_response(self, **kwargs):
            self.calls.append(kwargs)
            if any("scheduling" in fr for fr in kwargs["function_responses"]):
                raise RuntimeError("scheduling not supported on this model")
            return None

    picky = _PickySession()
    session._session = picky
    call = SimpleNamespace(id="c1", name="take_order", args={"quantity": 2})

    _run(session._handle_tool_calls([call]))

    assert len(picky.calls) == 2  # rejected with scheduling, retried without
    first = picky.calls[0]["function_responses"][0]
    second = picky.calls[1]["function_responses"][0]
    assert "scheduling" in first
    assert "scheduling" not in second
    assert second["response"]["quantity"] == 2


# -- framing injection -------------------------------------------------------


def test_push_frame_state_injects_framing_hint() -> None:
    session, _emitted, _robot = _make_session()
    session._types = SimpleNamespace(
        Content=lambda **kw: kw,
        Part=SimpleNamespace(from_text=lambda text: text),
    )
    mock = _MockLiveSession([])

    async def scenario():
        session._session = mock
        session._loop = asyncio.get_running_loop()
        session.push_frame_state({"state": "photo_ready", "line": ""})
        await asyncio.sleep(0.05)

    _run(scenario())

    assert mock.client_contents
    assert "[FRAMING]" in str(mock.client_contents)


def test_push_frame_state_ignores_irrelevant_state() -> None:
    session, _emitted, _robot = _make_session()
    session._types = SimpleNamespace(
        Content=lambda **kw: kw,
        Part=SimpleNamespace(from_text=lambda text: text),
    )
    mock = _MockLiveSession([])

    async def scenario():
        session._session = mock
        session._loop = asyncio.get_running_loop()
        session.push_frame_state({"state": "search", "line": ""})
        await asyncio.sleep(0.05)

    _run(scenario())
    assert mock.client_contents == []


# -- watchdog ----------------------------------------------------------------


def test_watchdog_idle_timeout_resets() -> None:
    session, emitted, _robot = _make_session(idle_timeout_s=0.05)

    async def scenario():
        session._loop = asyncio.get_running_loop()
        aged = session._loop.time() - 5.0
        session._last_activity_at = aged
        session._last_user_at = aged
        session._last_model_at = aged
        await asyncio.wait_for(session._watchdog(), timeout=3.0)

    _run(scenario())

    assert session.finished is True
    assert any(
        m.get("type") == "conversation_state"
        and m.get("state") == "reset"
        and m.get("data", {}).get("reason") == "idle_timeout"
        for m in emitted
    )


def test_watchdog_silence_nudge() -> None:
    session, _emitted, _robot = _make_session(idle_timeout_s=100.0)
    session._types = SimpleNamespace(
        Content=lambda **kw: kw,
        Part=SimpleNamespace(from_text=lambda text: text),
    )
    mock = _MockLiveSession([])

    async def scenario():
        session._session = mock
        session._loop = asyncio.get_running_loop()
        now = session._loop.time()
        session._last_activity_at = now
        session._last_user_at = now - 9.0
        session._last_model_at = now - 20.0
        task = asyncio.create_task(session._watchdog())
        await asyncio.sleep(1.3)
        session._finish()
        await task

    _run(scenario())
    assert any("[SYSTEM]" in str(c) for c in mock.client_contents)


# -- session open ------------------------------------------------------------


class _Any:
    def __call__(self, *args, **kwargs):
        return self

    def __getattr__(self, name):
        return self


class _AnyModule(builtin_types.ModuleType):
    def __getattr__(self, name):
        return _Any()


def _patch_genai(mock_session, captured):
    def connect(**kwargs):
        captured.update(kwargs)
        return mock_session

    mock_client = SimpleNamespace(
        aio=SimpleNamespace(live=SimpleNamespace(connect=connect)),
    )
    genai_module = builtin_types.ModuleType("google.genai")
    genai_module.Client = lambda **kw: mock_client
    types_module = _AnyModule("google.genai.types")
    genai_module.types = types_module
    google_module = builtin_types.ModuleType("google")
    google_module.genai = genai_module

    return patch.dict("sys.modules", {
        "google": google_module,
        "google.genai": genai_module,
        "google.genai.types": types_module,
    })


def test_open_connects_with_configured_model(monkeypatch) -> None:
    monkeypatch.setenv("GEMINI_API_KEY", "test-key")
    session, _emitted, _robot = _make_session(model="my-live-model", system_context="")
    mock = _MockLiveSession([])
    captured: dict = {}

    with _patch_genai(mock, captured):
        _run(session.open())

    assert captured["model"] == "my-live-model"
    assert session._session is mock


def test_open_requires_api_key(monkeypatch) -> None:
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    monkeypatch.delenv("GOOGLE_API_KEY", raising=False)
    session, _emitted, _robot = _make_session()
    mock = _MockLiveSession([])
    captured: dict = {}

    with _patch_genai(mock, captured):
        try:
            _run(session.open())
        except RuntimeError as exc:
            assert "GEMINI_API_KEY" in str(exc)
        else:
            raise AssertionError("expected RuntimeError for missing key")
