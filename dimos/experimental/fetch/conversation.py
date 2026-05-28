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

"""Persistent Gemini Live conversation for the Fetch Coke vendor.

This module owns a single bidirectional Gemini Live session per customer
interaction: it streams the customer's microphone audio in, streams the dog's
voice out, drives robot actions via tool calls, and uses the existing vision
policy's framing output (injected as context) to time the photo. It is the
conversational counterpart to the output-only ``tts.gemini_live_tts``.
"""

from __future__ import annotations

import asyncio
import base64
from datetime import datetime
import logging
from typing import Any, Awaitable, Callable

try:
    from dimos.experimental.fetch.tts import (
        DEFAULT_GEMINI_TTS_MODEL,
        _gemini_api_key,
        map_voice,
    )
except ModuleNotFoundError:
    from tts import (  # type: ignore[no-redef]
        DEFAULT_GEMINI_TTS_MODEL,
        _gemini_api_key,
        map_voice,
    )
try:
    from dimos.experimental.fetch.conversation_prompt import (
        build_system_instruction,
        build_tools,
    )
except ModuleNotFoundError:
    from conversation_prompt import (  # type: ignore[no-redef]
        build_system_instruction,
        build_tools,
    )

logger = logging.getLogger("fetch.conversation")

DEFAULT_IDLE_TIMEOUT_S = 30.0
SILENCE_NUDGE_S = 8.0
FRAMING_INJECT_MIN_INTERVAL_S = 2.0
MIC_QUEUE_MAXSIZE = 256

EmitCallable = Callable[[dict[str, Any]], Awaitable[None]]
RobotActionCallable = Callable[[dict[str, Any]], Awaitable[dict[str, Any]]]


class LiveConversationSession:
    """A persistent, bidirectional Gemini Live session for one interaction."""

    def __init__(
        self,
        *,
        emit: EmitCallable,
        robot_action: RobotActionCallable | None = None,
        voice: str = "Kore",
        model: str = DEFAULT_GEMINI_TTS_MODEL,
        system_context: str = "",
        idle_timeout_s: float = DEFAULT_IDLE_TIMEOUT_S,
    ) -> None:
        self._emit_cb = emit
        self._robot_action_cb = robot_action
        self.voice = map_voice(voice, "gemini")
        self.model = model
        self.idle_timeout_s = idle_timeout_s
        self._system_context = system_context
        self._system_instruction = build_system_instruction()

        self._mic_queue: asyncio.Queue[bytes | None] = asyncio.Queue(maxsize=MIC_QUEUE_MAXSIZE)
        self._done = asyncio.Event()
        self._session: Any | None = None
        self._cm: Any | None = None
        self._types: Any | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

        self._seq = 0
        self._dog_speaking = False
        self._last_activity_at = 0.0
        self._last_user_at = 0.0
        self._last_model_at = 0.0
        self._nudged = False
        self._last_framing_key: tuple[str, str] | None = None
        self._last_framing_at = 0.0
        self._terminal_emitted = False

    # -- lifecycle -----------------------------------------------------------

    @property
    def finished(self) -> bool:
        return self._done.is_set()

    async def open(self) -> None:
        from google import genai

        try:
            from google.genai import types
        except ModuleNotFoundError:  # pragma: no cover - real SDK always present
            import google.genai.types as types  # type: ignore[no-redef]

        self._types = types
        key_name, api_key = _gemini_api_key()
        if not api_key:
            raise RuntimeError(f"{key_name} is not set")

        self._loop = asyncio.get_running_loop()
        self._now()  # seed activity timestamps

        client = genai.Client(api_key=api_key)
        config = self._build_live_config(types)
        self._cm = client.aio.live.connect(model=self.model, config=config)
        self._session = await self._cm.__aenter__()

        if self._system_context:
            await self._inject_text(
                "You just walked up to a customer. The scene and your opening "
                f"joke: {self._system_context} Greet them and start the bit."
            )

    async def run(self) -> None:
        if self._session is None:
            await self.open()
        tasks = [
            asyncio.create_task(self._send_loop()),
            asyncio.create_task(self._recv_loop()),
            asyncio.create_task(self._watchdog()),
        ]
        done_waiter = asyncio.create_task(self._done.wait())
        try:
            await asyncio.wait([*tasks, done_waiter], return_when=asyncio.FIRST_COMPLETED)
        finally:
            for task in (*tasks, done_waiter):
                task.cancel()
            await asyncio.gather(*tasks, done_waiter, return_exceptions=True)
            if not self._terminal_emitted:
                # The session ended without a tool/idle reset (e.g. an exception
                # in a loop) — tell the browser so it leaves conversation mode.
                await self._emit_terminal("reset", {"reason": "ended"})
            await self.close()

    async def close(self) -> None:
        self._done.set()
        try:
            self._mic_queue.put_nowait(None)
        except asyncio.QueueFull:
            pass
        if self._cm is not None:
            try:
                await self._cm.__aexit__(None, None, None)
            except Exception:  # pragma: no cover - SDK teardown best effort
                logger.exception("conversation session close failed")
        self._cm = None
        self._session = None

    # -- inputs from the middleware -----------------------------------------

    def push_mic(self, pcm: bytes) -> None:
        """Enqueue a base64-decoded PCM chunk (16 kHz int16) from the browser."""
        if self._done.is_set() or not pcm:
            return
        try:
            self._mic_queue.put_nowait(pcm)
        except asyncio.QueueFull:
            logger.debug("mic queue full; dropping chunk")

    def push_frame_state(self, decision: dict[str, Any]) -> None:
        """Inject the vision policy's framing result so the dog can coach."""
        if self._done.is_set() or self._session is None or self._loop is None:
            return
        state = str(decision.get("state") or "")
        line = str(decision.get("line") or "")
        if state == "photo_ready":
            hint = (
                "good - the customer is clearly holding the Coke and the shot "
                "is centered and well framed; take the photo now"
            )
        elif state == "wait_for_coke":
            coach = line or "ask them to hold the Coke up and center themselves"
            hint = f"not ready yet - {coach}"
        else:
            return
        now = self._loop.time()
        key = (state, line)
        if key == self._last_framing_key and now - self._last_framing_at < FRAMING_INJECT_MIN_INTERVAL_S:
            return
        self._last_framing_key = key
        self._last_framing_at = now
        asyncio.create_task(self._inject_text(f"[FRAMING] {hint}"))

    # -- internal loops ------------------------------------------------------

    async def _send_loop(self) -> None:
        types = self._types
        while True:
            chunk = await self._mic_queue.get()
            if chunk is None:
                return
            if self._session is None:
                continue
            try:
                await self._session.send_realtime_input(
                    audio=types.Blob(data=chunk, mime_type="audio/pcm;rate=16000"),
                )
            except Exception:
                logger.exception("conversation mic forward failed")

    async def _recv_loop(self) -> None:
        if self._session is None:
            return
        async for message in self._session.receive():
            tool_call = getattr(message, "tool_call", None)
            if tool_call is not None and getattr(tool_call, "function_calls", None):
                await self._handle_tool_calls(tool_call.function_calls)
                self._mark_activity()
                continue

            server_content = getattr(message, "server_content", None)
            if server_content is None:
                continue

            if getattr(server_content, "interrupted", False):
                self._dog_speaking = False
                await self._emit({"type": "interrupted"})

            model_turn = getattr(server_content, "model_turn", None)
            if model_turn is not None:
                for part in getattr(model_turn, "parts", None) or []:
                    inline = getattr(part, "inline_data", None)
                    data = getattr(inline, "data", None) if inline is not None else None
                    if data:
                        self._dog_speaking = True
                        self._mark_model_activity()
                        self._seq += 1
                        await self._emit({
                            "type": "audio_out",
                            "data": base64.b64encode(data).decode("ascii"),
                            "seq": self._seq,
                        })

            out_text = _transcription_text(getattr(server_content, "output_transcription", None))
            if out_text:
                self._mark_model_activity()
                await self._emit({
                    "type": "transcript",
                    "speaker": "dog",
                    "text": out_text,
                    "final": bool(getattr(server_content, "turn_complete", False)),
                })

            in_text = _transcription_text(getattr(server_content, "input_transcription", None))
            if in_text:
                self._mark_user_activity()
                await self._emit({
                    "type": "transcript",
                    "speaker": "user",
                    "text": in_text,
                    "final": False,
                })

            if getattr(server_content, "turn_complete", False):
                self._dog_speaking = False

    async def _watchdog(self) -> None:
        while not self._done.is_set():
            await asyncio.sleep(1.0)
            if self._loop is None:
                continue
            now = self._loop.time()
            if now - self._last_activity_at > self.idle_timeout_s:
                await self._emit_terminal("reset", {"reason": "idle_timeout"})
                self._finish()
                return
            if (
                self._last_user_at > self._last_model_at
                and now - self._last_user_at > SILENCE_NUDGE_S
                and not self._nudged
            ):
                self._nudged = True
                await self._inject_text(
                    "[SYSTEM] The customer spoke but you went quiet. Respond now."
                )

    # -- tool calls ----------------------------------------------------------

    async def _handle_tool_calls(self, function_calls: Any) -> None:
        if self._session is None:
            return
        pending: list[tuple[Any, str, dict[str, Any], str | None]] = []
        should_finish = False
        for call in function_calls:
            name = str(getattr(call, "name", "") or "")
            args = dict(getattr(call, "args", None) or {})
            result, scheduling, finish = await self._dispatch_tool(name, args)
            should_finish = should_finish or finish
            pending.append((getattr(call, "id", None), name, result, scheduling))
        if not await self._send_tool_responses(pending, with_scheduling=True):
            # Scheduling is an async-function-calling hint; a synchronous-only
            # model may reject it. Retry without it so the result is delivered
            # and the conversation never hangs waiting on a tool response.
            await self._send_tool_responses(pending, with_scheduling=False)
        # Finish only after the final response is sent, so done_waiter cannot
        # cancel _recv_loop mid-send and drop the terminal tool response.
        if should_finish:
            self._finish()

    async def _send_tool_responses(
        self,
        pending: list[tuple[Any, str, dict[str, Any], str | None]],
        with_scheduling: bool,
    ) -> bool:
        responses = [
            self._function_response(
                call_id, name, result, scheduling if with_scheduling else None
            )
            for call_id, name, result, scheduling in pending
        ]
        try:
            await self._session.send_tool_response(function_responses=responses)
            return True
        except Exception:
            logger.exception("send_tool_response failed (with_scheduling=%s)", with_scheduling)
            return False

    async def _dispatch_tool(
        self, name: str, args: dict[str, Any]
    ) -> tuple[dict[str, Any], str | None, bool]:
        """Returns (response, scheduling, finish). ``finish`` is applied by the
        caller only after the tool response is sent, so the terminal response is
        never dropped by an early session shutdown."""
        if name == "take_order":
            quantity = _coerce_int(args.get("quantity"), default=1, low=1, high=4)
            noun = "Coke" if quantity == 1 else "Cokes"
            return (
                {
                    "order_recorded": True,
                    "quantity": quantity,
                    "instructions": (
                        f"Tell them to reach back and grab {quantity} ice-cold "
                        f"{noun} from the pouch on your back."
                    ),
                },
                "WHEN_IDLE",
                False,
            )

        if name == "take_photo":
            await self._emit({"type": "conversation_state", "state": "take_photo", "data": {}})
            return (
                {"photo_taken": True, "ts": datetime.now().isoformat(timespec="seconds")},
                "WHEN_IDLE",
                False,
            )

        if name == "do_trick":
            trick = str(args.get("trick") or "wave").strip().lower()
            if trick not in ("wave", "dance"):
                return ({"performed": False, "error": f"unknown trick {trick!r}"}, "WHEN_IDLE", False)
            robot = await self._robot_action({"action": trick})
            return (
                {"performed": bool(robot.get("ok", False)), "trick": trick, "robot_state": robot},
                "WHEN_IDLE",
                False,
            )

        if name == "celebrate":
            robot = await self._robot_action({"action": "dance"})
            await self._emit_terminal("celebrate", {"goodbye_line": str(args.get("goodbye_line") or "")})
            return ({"celebrated": True, "robot_state": robot}, "INTERRUPT", True)

        if name == "stop_and_reset":
            reason = str(args.get("reason") or "")
            await self._emit_terminal("reset", {"reason": reason})
            return ({"reset": True, "reason": reason}, "INTERRUPT", True)

        return ({"success": False, "error": f"unknown tool {name!r}"}, None, False)

    async def _robot_action(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._robot_action_cb is None:
            return {"enabled": False, "ok": False, "message": "robot not connected"}
        try:
            return await self._robot_action_cb(payload)
        except Exception as exc:
            logger.exception("robot action failed during conversation")
            return {"enabled": True, "ok": False, "message": str(exc)}

    def _function_response(self, call_id: Any, name: str, result: dict[str, Any], scheduling: str | None) -> Any:
        types = self._types
        kwargs: dict[str, Any] = {"name": name, "response": result}
        if call_id is not None:
            kwargs["id"] = call_id
        if scheduling is not None:
            # The Live API accepts scheduling as a string ("WHEN_IDLE",
            # "INTERRUPT", "SILENT"); the SDK enum is a str subclass so this
            # coerces cleanly without depending on the enum's attribute name.
            kwargs["scheduling"] = scheduling
        return types.FunctionResponse(**kwargs)

    # -- config + helpers ----------------------------------------------------

    def _build_live_config(self, types: Any) -> Any:
        kwargs: dict[str, Any] = dict(
            response_modalities=["AUDIO"],
            system_instruction=types.Content(
                parts=[types.Part.from_text(text=self._system_instruction)],
            ),
            tools=[build_tools(types)],
            speech_config=types.SpeechConfig(
                voice_config=types.VoiceConfig(
                    prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=self.voice),
                ),
            ),
        )
        optional = (
            ("input_audio_transcription", lambda: types.AudioTranscriptionConfig()),
            ("output_audio_transcription", lambda: types.AudioTranscriptionConfig()),
            (
                "realtime_input_config",
                lambda: types.RealtimeInputConfig(
                    automatic_activity_detection=types.AutomaticActivityDetection(),
                ),
            ),
            ("session_resumption", lambda: types.SessionResumptionConfig()),
            (
                "context_window_compression",
                lambda: types.ContextWindowCompressionConfig(
                    sliding_window=types.SlidingWindow(),
                ),
            ),
        )
        for field, factory in optional:
            try:
                kwargs[field] = factory()
            except Exception:
                logger.warning("conversation config: skipping unsupported field %s", field)
        return types.LiveConnectConfig(**kwargs)

    async def _inject_text(self, text: str) -> None:
        if self._session is None:
            return
        types = self._types
        try:
            await self._session.send_client_content(
                turns=[types.Content(role="user", parts=[types.Part.from_text(text=text)])],
                turn_complete=True,
            )
        except Exception:
            logger.exception("conversation context inject failed")

    async def _emit(self, message: dict[str, Any]) -> None:
        try:
            await self._emit_cb(message)
        except Exception:
            logger.exception("conversation emit failed")

    async def _emit_terminal(self, state: str, data: dict[str, Any] | None = None) -> None:
        """Emit a terminal conversation_state and record that the browser was told
        the interaction ended, so run()'s finally won't send a duplicate reset."""
        self._terminal_emitted = True
        await self._emit({"type": "conversation_state", "state": state, "data": data or {}})

    def _finish(self) -> None:
        self._done.set()

    def _now(self) -> float:
        now = self._loop.time() if self._loop is not None else 0.0
        self._last_activity_at = now
        self._last_user_at = now
        self._last_model_at = now
        return now

    def _mark_activity(self) -> None:
        if self._loop is not None:
            self._last_activity_at = self._loop.time()

    def _mark_user_activity(self) -> None:
        if self._loop is not None:
            now = self._loop.time()
            self._last_user_at = now
            self._last_activity_at = now

    def _mark_model_activity(self) -> None:
        if self._loop is not None:
            now = self._loop.time()
            self._last_model_at = now
            self._last_activity_at = now
            self._nudged = False


def _transcription_text(transcription: Any) -> str:
    if transcription is None:
        return ""
    return str(getattr(transcription, "text", "") or "")


def _coerce_int(value: Any, *, default: int, low: int, high: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        parsed = default
    return max(low, min(high, parsed))
