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

"""MiniMax TTS node.

Calls MiniMax's ``/v1/t2a_v2`` HTTP endpoint to synthesize speech. The endpoint
returns hex-encoded audio chunks via SSE (when ``stream=true``) or a single hex
payload in JSON. We default to non-streaming JSON for simplicity — the audio is
small enough per utterance that we don't need SSE plumbing here, and ``node_openai``
follows the same request-and-decode pattern.

Reference: https://platform.minimax.io/docs/api-reference/speech-t2a-http
"""

from enum import Enum
import io
import json
import threading
import time
import urllib.error
import urllib.request

from reactivex import Observable, Subject
import soundfile as sf  # type: ignore[import-untyped]

from dimos.stream.audio.base import (
    AbstractAudioEmitter,
    AudioEvent,
)
from dimos.stream.audio.text.base import AbstractTextConsumer, AbstractTextEmitter
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Default MiniMax TTS API base URL. Override with ``MINIMAX_BASE_URL``.
DEFAULT_MINIMAX_BASE_URL = "https://api.minimax.io"

# Recommended default TTS model — best quality / naturalness.
DEFAULT_MINIMAX_TTS_MODEL = "speech-2.8-hd"

# Default voice — English, female, graceful.
DEFAULT_MINIMAX_VOICE = "English_Graceful_Lady"


class MiniMaxTTSVoice(str, Enum):
    """A small subset of MiniMax system voices.

    See https://platform.minimax.io/faq/system-voice-id for the full list.
    """

    GRACEFUL_LADY = "English_Graceful_Lady"
    INSIGHTFUL_SPEAKER = "English_Insightful_Speaker"
    RADIANT_GIRL = "English_radiant_girl"
    PERSUASIVE_MAN = "English_Persuasive_Man"
    LUCKY_ROBOT = "English_Lucky_Robot"
    EXPRESSIVE_NARRATOR = "English_expressive_narrator"


class MiniMaxTTSNode(AbstractTextConsumer, AbstractAudioEmitter, AbstractTextEmitter):
    """Text-to-speech node backed by MiniMax's t2a_v2 API.

    The node consumes text from an observable, calls MiniMax to synthesize
    audio, and emits AudioEvent frames on ``emit_audio()`` and the source text
    on ``emit_text()``.

    Args:
        api_key: MiniMax API key (defaults to ``MINIMAX_API_KEY`` env var).
        voice: Voice ID (see :class:`MiniMaxTTSVoice`).
        model: TTS model (default ``speech-2.8-hd``).
        base_url: Override the MiniMax API base URL.
        speed: Speech rate in ``[0.5, 2]`` (default 1.0).
        sample_rate: Output sample rate in Hz (default 32000).
        buffer_size: Reserved for API parity with the OpenAI node.
    """

    def __init__(
        self,
        api_key: str | None = None,
        voice: MiniMaxTTSVoice = MiniMaxTTSVoice.GRACEFUL_LADY,
        model: str = DEFAULT_MINIMAX_TTS_MODEL,
        base_url: str | None = None,
        speed: float = 1.0,
        sample_rate: int = 32000,
        buffer_size: int = 1024,
    ) -> None:
        self.api_key = api_key
        self.voice = voice
        self.model = model
        self.base_url = base_url or DEFAULT_MINIMAX_BASE_URL
        self.speed = speed
        self.sample_rate = sample_rate
        self.buffer_size = buffer_size

        self.audio_subject = Subject()  # type: ignore[var-annotated]
        self.text_subject = Subject()  # type: ignore[var-annotated]
        self.subscription = None
        self.processing_thread = None
        self.is_running = True
        self.text_queue: list[str] = []  # type: ignore[var-annotated]
        self.queue_lock = threading.Lock()

    def _resolve_api_key(self) -> str:
        import os

        key = self.api_key or os.getenv("MINIMAX_API_KEY")
        if not key:
            raise ValueError(
                "MiniMax API key must be provided or set in MINIMAX_API_KEY environment variable"
            )
        return key

    def emit_audio(self) -> Observable:  # type: ignore[type-arg]
        return self.audio_subject

    def emit_text(self) -> Observable:  # type: ignore[type-arg]
        return self.text_subject

    def consume_text(self, text_observable: Observable) -> "AbstractTextConsumer":  # type: ignore[type-arg]
        logger.info("Starting MiniMaxTTSNode")

        self.processing_thread = threading.Thread(target=self._process_queue, daemon=True)  # type: ignore[assignment]
        self.processing_thread.start()  # type: ignore[attr-defined]

        self.subscription = text_observable.subscribe(  # type: ignore[assignment]
            on_next=self._queue_text,
            on_error=lambda e: logger.error(f"Error in MiniMaxTTSNode: {e}"),
        )

        return self

    def _queue_text(self, text: str) -> None:
        if not text.strip():
            return
        with self.queue_lock:
            self.text_queue.append(text)

    def _process_queue(self) -> None:
        while self.is_running:
            text_to_process = None
            with self.queue_lock:
                if self.text_queue:
                    text_to_process = self.text_queue.pop(0)

            if text_to_process:
                self._synthesize_speech(text_to_process)
            else:
                time.sleep(0.1)

    def _synthesize_speech(self, text: str) -> None:
        """Call MiniMax t2a_v2 and emit the resulting audio frame."""
        try:
            audio_bytes = self._call_t2a(text)
        except Exception as e:
            logger.error(f"Error synthesizing speech with MiniMax: {e}")
            return

        try:
            audio_data = io.BytesIO(audio_bytes)
            with sf.SoundFile(audio_data, "r") as sound_file:
                actual_sample_rate = sound_file.samplerate
                audio_array = sound_file.read()
        except Exception as e:
            logger.error(f"Error decoding MiniMax audio: {e}")
            return

        logger.debug(f"MiniMax audio sample rate: {actual_sample_rate}Hz")

        self.text_subject.on_next(text)

        audio_event = AudioEvent(
            data=audio_array,
            sample_rate=actual_sample_rate,
            timestamp=time.time(),
            channels=1 if audio_array.ndim == 1 else audio_array.shape[1],
        )
        self.audio_subject.on_next(audio_event)

    def _call_t2a(self, text: str) -> bytes:
        """Call MiniMax t2a_v2 (non-streaming) and return raw mp3 bytes."""
        url = f"{self.base_url.rstrip('/')}/v1/t2a_v2"
        payload = {
            "model": self.model,
            "text": text,
            "stream": False,
            "voice_setting": {
                "voice_id": self.voice.value,
                "speed": self.speed,
                "vol": 1,
                "pitch": 0,
            },
            "audio_setting": {
                "sample_rate": self.sample_rate,
                "bitrate": 128000,
                "format": "mp3",
                "channel": 1,
            },
        }

        req = urllib.request.Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self._resolve_api_key()}",
            },
            method="POST",
        )

        with urllib.request.urlopen(req, timeout=30) as response:  # noqa: S310 - URL is configurable
            body = json.loads(response.read().decode("utf-8"))

        # MiniMax returns a status_code; non-zero means failure.
        status_code = body.get("base_resp", {}).get("status_code", 0)
        if status_code != 0:
            status_msg = body.get("base_resp", {}).get("status_msg", "unknown error")
            raise RuntimeError(f"MiniMax TTS failed (status_code={status_code}): {status_msg}")

        hex_audio = body.get("data", {}).get("audio", "")
        if not hex_audio:
            raise RuntimeError("MiniMax TTS returned empty audio payload")

        # MiniMax returns hex-encoded audio, not base64.
        return bytes.fromhex(hex_audio)

    def dispose(self) -> None:
        """Clean up resources."""
        logger.info("Disposing MiniMaxTTSNode")

        self.is_running = False

        with self.queue_lock:
            self.text_queue.clear()

        if self.processing_thread and self.processing_thread.is_alive():
            self.processing_thread.join(timeout=2.0)

        if self.subscription:
            self.subscription.dispose()
            self.subscription = None

        self.audio_subject.on_completed()
        self.text_subject.on_completed()


if __name__ == "__main__":
    import os

    from reactivex import Subject

    from dimos.stream.audio.node_output import SounddeviceAudioOutput
    from dimos.stream.audio.text.node_stdout import TextPrinterNode
    from dimos.stream.audio.utils import keepalive

    if not os.getenv("MINIMAX_API_KEY"):
        raise SystemExit("Set MINIMAX_API_KEY to run the MiniMax TTS demo.")

    text_subject = Subject()  # type: ignore[var-annotated]

    tts_node = MiniMaxTTSNode(voice=MiniMaxTTSVoice.GRACEFUL_LADY)
    tts_node.consume_text(text_subject)

    audio_output = SounddeviceAudioOutput(sample_rate=tts_node.sample_rate)
    audio_output.consume_audio(tts_node.emit_audio())

    stdout = TextPrinterNode(prefix="[Spoken Text] ")
    stdout.consume_text(tts_node.emit_text())

    for message in [
        "Hello!",
        "This is a test of the MiniMax text to speech system.",
    ]:
        text_subject.on_next(message)

    keepalive()
