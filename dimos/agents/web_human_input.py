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

import json
from threading import Thread
from typing import TYPE_CHECKING

import reactivex as rx
import reactivex.operators as ops

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


class WebInput(Module):
    _web_interface: RobotWebInterface | None = None
    _thread: Thread | None = None
    _human_transport: pLCMTransport[str] | None = None
    _agent_transport: pLCMTransport | None = None

    @rpc
    def start(self) -> None:
        super().start()

        self._human_transport = pLCMTransport("/human_input")

        audio_subject: rx.subject.Subject[AudioEvent] = rx.subject.Subject()

        agent_responses: rx.subject.Subject[str] = rx.subject.Subject()
        self._web_interface = RobotWebInterface(
            port=5555,
            text_streams={"agent_responses": agent_responses},
            audio_subject=audio_subject,
        )

        # Forward the LLM agent's replies (published on LCM "/agent") to the
        # agent_responses SSE stream so the web UI can display them.
        self._agent_transport = pLCMTransport("/agent")

        def _on_agent_message(msg: object) -> None:
            kind = getattr(msg, "type", None)  # "human" | "ai" | "tool" | "system"
            if kind == "human":
                return  # skip the echoed user input
            content = getattr(msg, "content", None)
            if isinstance(content, list):
                content = " ".join(
                    str(part.get("text", "")) if isinstance(part, dict) else str(part)
                    for part in content
                )
            if content:
                # Emit a typed envelope so the UI can read out only the agent's
                # spoken replies (kind == "ai") and treat tool output as status.
                agent_responses.on_next(json.dumps({"kind": kind, "text": str(content)}))

        self._agent_transport.subscribe(_on_agent_message)

        normalizer = AudioNormalizer()

        # Here to prevent unwanted imports in the file.
        from dimos.stream.audio.stt.node_whisper import WhisperNode

        stt_node = WhisperNode()

        # Connect audio pipeline: browser audio → normalizer → whisper
        normalizer.consume_audio(audio_subject.pipe(ops.share()))
        stt_node.consume_audio(normalizer.emit_audio())

        # Subscribe to both text input sources
        # 1. Direct text from web interface
        unsub = self._web_interface.query_stream.subscribe(self._human_transport.publish)
        self.register_disposable(unsub)

        # 2. Transcribed text from STT
        unsub = stt_node.emit_text().subscribe(self._human_transport.publish)
        self.register_disposable(unsub)

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
        if self._agent_transport:
            self._agent_transport.lcm.stop()
        super().stop()
