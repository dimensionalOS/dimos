#!/usr/bin/env python3
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

import os
import threading
import time

from google import genai
from google.genai import types
import numpy as np
from reactivex import Observable, Subject

from dimos.stream.audio.base import (
    AbstractAudioEmitter,
    AudioEvent,
)
from dimos.stream.audio.text.base import AbstractTextConsumer, AbstractTextEmitter
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Gemini TTS returns single-channel 16-bit PCM at 24 kHz, little-endian.
_SAMPLE_RATE = 24000


class GeminiTTSNode(AbstractTextConsumer, AbstractAudioEmitter, AbstractTextEmitter):
    """
    A text-to-speech node that consumes text, emits audio using Google's Gemini TTS API, and passes through text.

    This node implements AbstractTextConsumer to receive text input, AbstractAudioEmitter
    to provide audio output, and AbstractTextEmitter to pass through the text being spoken,
    allowing it to be inserted into a text-to-audio pipeline with text passthrough capabilities.

    Mirrors OpenAITTSNode but uses the Gemini API directly (google-genai), so it
    needs no OpenAI key and reuses the GOOGLE_API_KEY already used for the LLM
    and embeddings. Only the ``*-preview-tts`` models support audio output.
    """

    def __init__(
        self,
        api_key: str | None = None,
        voice: str = "Kore",
        model: str = "gemini-2.5-flash-preview-tts",
        instruction: str = "Read the following text aloud verbatim",
    ) -> None:
        """
        Initialize GeminiTTSNode.

        Args:
            api_key: Gemini API key (if None, reads GOOGLE_API_KEY / GEMINI_API_KEY)
            voice: Prebuilt Gemini voice name (e.g. "Kore", "Puck", "Charon")
            model: Gemini TTS model (must be a ``*-preview-tts`` model)
            instruction: Style directive prepended to each utterance. Gemini
                narrates only the text after it (not the directive itself); it
                also stops the model from "answering" short prompts instead of
                reading them. Set to "" to send the raw text unmodified.
        """
        self.voice = voice
        self.model = model
        self.instruction = instruction

        api_key = api_key or os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("Gemini TTS requires GEMINI_API_KEY or GOOGLE_API_KEY to be set")
        self.client = genai.Client(api_key=api_key)

        # Initialize state
        self.audio_subject = Subject()  # type: ignore[var-annotated]
        self.text_subject = Subject()  # type: ignore[var-annotated]
        self.subscription = None
        self.processing_thread = None
        self.is_running = True
        self.text_queue = []  # type: ignore[var-annotated]
        self.queue_lock = threading.Lock()

    def emit_audio(self) -> Observable:  # type: ignore[type-arg]
        """
        Returns an observable that emits audio frames.

        Returns:
            Observable emitting AudioEvent objects
        """
        return self.audio_subject

    def emit_text(self) -> Observable:  # type: ignore[type-arg]
        """
        Returns an observable that emits the text being spoken.

        Returns:
            Observable emitting text strings
        """
        return self.text_subject

    def consume_text(self, text_observable: Observable) -> "AbstractTextConsumer":  # type: ignore[type-arg]
        """
        Start consuming text from the observable source.

        Args:
            text_observable: Observable source of text strings

        Returns:
            Self for method chaining
        """
        logger.info("Starting GeminiTTSNode")

        # Start the processing thread
        self.processing_thread = threading.Thread(target=self._process_queue, daemon=True)  # type: ignore[assignment]
        self.processing_thread.start()  # type: ignore[attr-defined]

        # Subscribe to the text observable
        self.subscription = text_observable.subscribe(  # type: ignore[assignment]
            on_next=self._queue_text,
            on_error=lambda e: logger.error(f"Error in GeminiTTSNode: {e}"),
        )

        return self

    def _queue_text(self, text: str) -> None:
        """
        Add text to the processing queue.

        Args:
            text: The text to synthesize
        """
        if not text.strip():
            return

        with self.queue_lock:
            self.text_queue.append(text)

    def _process_queue(self) -> None:
        """Background thread to process the text queue."""
        while self.is_running:
            # Check if there's text to process
            text_to_process = None
            with self.queue_lock:
                if self.text_queue:
                    text_to_process = self.text_queue.pop(0)

            if text_to_process:
                self._synthesize_speech(text_to_process)
            else:
                # Sleep a bit to avoid busy-waiting
                time.sleep(0.1)

    def _synthesize_speech(self, text: str) -> None:
        """
        Convert text to speech using the Gemini TTS API.

        Args:
            text: The text to synthesize
        """
        try:
            contents = f"{self.instruction}: {text}" if self.instruction else text
            response = self.client.models.generate_content(
                model=self.model,
                contents=contents,
                config=types.GenerateContentConfig(
                    response_modalities=["AUDIO"],
                    speech_config=types.SpeechConfig(
                        voice_config=types.VoiceConfig(
                            prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=self.voice)
                        )
                    ),
                ),
            )
            self.text_subject.on_next(text)

            # Gemini returns raw 16-bit PCM bytes (24 kHz, mono, little-endian).
            pcm_bytes = response.candidates[0].content.parts[0].inline_data.data
            audio_array = np.frombuffer(pcm_bytes, dtype=np.int16)

            audio_event = AudioEvent(
                data=audio_array,
                sample_rate=_SAMPLE_RATE,
                timestamp=time.time(),
                channels=1,
            )

            self.audio_subject.on_next(audio_event)

        except Exception as e:
            logger.error(f"Error synthesizing speech: {e}")

    def dispose(self) -> None:
        """Clean up resources."""
        logger.info("Disposing GeminiTTSNode")

        self.is_running = False

        # Clear pending items so the thread doesn't start new synthesis.
        with self.queue_lock:
            self.text_queue.clear()

        if self.processing_thread and self.processing_thread.is_alive():
            self.processing_thread.join(timeout=2.0)

        if self.subscription:
            self.subscription.dispose()
            self.subscription = None

        # Complete the subjects
        self.audio_subject.on_completed()
        self.text_subject.on_completed()


if __name__ == "__main__":
    from dimos.stream.audio.node_output import SounddeviceAudioOutput
    from dimos.stream.audio.text.node_stdout import TextPrinterNode
    from dimos.stream.audio.utils import keepalive

    # Create a simple text subject that we can push values to
    text_subject = Subject()  # type: ignore[var-annotated]

    tts_node = GeminiTTSNode(voice="Kore")
    tts_node.consume_text(text_subject)

    # Create and connect an audio output node - match Gemini's 24 kHz output
    audio_output = SounddeviceAudioOutput(sample_rate=_SAMPLE_RATE)
    audio_output.consume_audio(tts_node.emit_audio())

    stdout = TextPrinterNode(prefix="[Spoken Text] ")
    stdout.consume_text(tts_node.emit_text())

    test_messages = [
        "Hello!",
        "This is a test of the Gemini text to speech system.",
    ]

    print("Starting Gemini TTS test...")
    print("-" * 60)

    for message in test_messages:
        text_subject.on_next(message)

    keepalive()
