# Copyright 2025-2026 Dimensional Inc.
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

from threading import Thread
from typing import TYPE_CHECKING, Any

import reactivex as rx
import reactivex.operators as ops

from dimos.agents.annotation import skill
from dimos.agents.skills.seat_guide import (
    SeatGuideRequestSpec,
    is_seat_guide_preview_request,
    parse_seat_guide_intent,
)
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.transport import pLCMTransport
from dimos.stream.audio.node_normalizer import AudioNormalizer
from dimos.utils.logging_config import setup_logger
from dimos.web.robot_web_interface import RobotWebInterface

if TYPE_CHECKING:
    from dimos.stream.audio.base import AudioEvent

logger = setup_logger()


def _create_whisper_node():
    # Do not force English here. SeatGuide's primary demo phrase is Chinese, and
    # Whisper can auto-detect language when `language` is omitted.
    from dimos.stream.audio.stt.node_whisper import WhisperNode

    return WhisperNode(modelopts={"fp16": False})


class WebInput(Module):
    _web_interface: RobotWebInterface | None = None
    _thread: Thread | None = None
    _human_transport: pLCMTransport[str] | None = None
    _seat_guide: SeatGuideRequestSpec | None = None
    _agent_responses: rx.subject.Subject[str] | None = None
    _stt_node: Any | None = None
    _stt_error: str | None = None

    @rpc
    def start(self) -> None:
        super().start()

        self._human_transport = pLCMTransport("/human_input")

        audio_subject: rx.subject.Subject[AudioEvent] = rx.subject.Subject()

        self._agent_responses = rx.subject.Subject()
        self._web_interface = RobotWebInterface(
            port=5555,
            text_streams={"agent_responses": self._agent_responses},
            audio_subject=audio_subject,
        )

        unsub = self._web_interface.query_stream.subscribe(self._route_text)
        self.register_disposable(unsub)

        try:
            normalizer = AudioNormalizer()
            stt_node = _create_whisper_node()
            self._stt_node = stt_node
            self._stt_error = None

            normalizer.consume_audio(audio_subject.pipe(ops.share()))
            stt_node.consume_audio(normalizer.emit_audio())

            unsub = stt_node.emit_text().subscribe(self._route_text)
            self.register_disposable(unsub)
        except Exception as exc:
            self._stt_node = None
            self._stt_error = f"{type(exc).__name__}: {exc}"
            logger.exception("WebInput speech-to-text pipeline unavailable")

        self._thread = Thread(target=self._web_interface.run, daemon=True)
        self._thread.start()

        logger.info("Web interface started at http://localhost:5555")

    @rpc
    def stop(self) -> None:
        if self._web_interface:
            self._web_interface.shutdown()
        if self._thread:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        if self._human_transport:
            self._human_transport.lcm.stop()
        super().stop()

    @skill
    def web_input_status(self) -> str:
        """Report WebInput voice and text routing readiness.

        Use this during Go2 bring-up to confirm the browser microphone/text
        entry point is running, SeatGuide direct routing is connected, and
        SeatGuide responses can be streamed back to the web UI.
        """
        web_state = "started" if self._web_interface is not None else "not_started"
        thread_state = (
            "running"
            if self._thread is not None and self._thread.is_alive()
            else "not_running"
        )
        seat_route = (
            "seat_guide_direct" if self._seat_guide is not None else "agent_only"
        )
        response_stream = (
            "connected" if self._agent_responses is not None else "missing"
        )
        voice_upload = (
            "connected"
            if self._web_interface is not None
            and getattr(self._web_interface, "audio_subject", None) is not None
            else "missing"
        )
        if self._stt_node is not None:
            stt_state = "connected"
        elif getattr(self, "_stt_error", None):
            stt_state = f"error({self._stt_error})"
        else:
            stt_state = "missing"
        human_transport = (
            "connected" if self._human_transport is not None else "missing"
        )
        url = (
            f"http://localhost:{self._web_interface.port}"
            if self._web_interface is not None
            else "unavailable"
        )
        return (
            f"WebInput status: web={web_state}; thread={thread_state}; "
            f"seat_route={seat_route}; responses={response_stream}; "
            f"voice_upload={voice_upload}; stt={stt_state}; "
            f"human_transport={human_transport}; url={url}."
        )

    def _route_text(self, text: str) -> None:
        logger.info("WebInput received text", text=text)
        if parse_seat_guide_intent(text).should_find_seat and self._seat_guide is not None:
            try:
                if is_seat_guide_preview_request(text):
                    logger.info("WebInput routing text to SeatGuide preview", text=text)
                    response = self._seat_guide.preview_seat_request(text)
                else:
                    logger.info("WebInput routing text to SeatGuide live request", text=text)
                    response = self._seat_guide.handle_seat_request(text)
                self._publish_agent_response(response)
                return
            except Exception:
                logger.exception(
                    "SeatGuide direct route failed; publishing text to normal agent path"
                )

        if self._human_transport is None:
            logger.warning("Dropping human input because human transport is not initialized")
            return
        logger.info("WebInput routing text to agent path", text=text)
        self._human_transport.publish(text)

    def _publish_agent_response(self, text: str) -> None:
        if self._agent_responses is None:
            return
        self._agent_responses.on_next(text)
