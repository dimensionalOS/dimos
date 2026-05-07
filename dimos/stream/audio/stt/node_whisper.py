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

from typing import Any

from reactivex import Observable, create, disposable

from dimos.stream.audio.base import (
    AbstractAudioConsumer,
    AudioEvent,
)
from dimos.stream.audio.text.base import AbstractTextEmitter
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

try:
    import whisper  # type: ignore[import-untyped]

    _USE_FASTER_WHISPER = False
except ImportError:
    try:
        from faster_whisper import WhisperModel  # type: ignore[import-untyped]

        logger.warn(
            "openai-whisper not installed, falling back to faster-whisper. "
            "Install openai-whisper for the full backend: pip install openai-whisper",
        )
        _USE_FASTER_WHISPER = True
    except ImportError:
        raise ImportError(
            "No whisper backend found. "
            "Install faster-whisper (pip install faster-whisper) "
            "or openai-whisper (pip install dimos[whisper])."
        )


class WhisperNode(AbstractAudioConsumer, AbstractTextEmitter):
    """
    A node that transcribes audio using OpenAI Whisper or faster-whisper and emits
    the transcribed text. Prefers openai-whisper if installed, falls back to faster-whisper.
    """

    def __init__(
        self,
        model: str = "base",
        modelopts: dict[str, Any] | None = None,
    ) -> None:
        if modelopts is None:
            modelopts = {"language": "en", "fp16": False}
        self.audio_observable = None

        if _USE_FASTER_WHISPER:
            compute_type = "float16" if modelopts.get("fp16", False) else "int8"
            modelopts = {k: v for k, v in modelopts.items() if k != "fp16"}
            self.modelopts = modelopts
            self.model = WhisperModel(model, device="auto", compute_type=compute_type)
        else:
            self.modelopts = modelopts
            self.model = whisper.load_model(model)

    def consume_audio(self, audio_observable: Observable) -> "WhisperNode":  # type: ignore[type-arg]
        """
        Set the audio source observable to consume.

        Args:
            audio_observable: Observable emitting AudioEvent objects

        Returns:
            Self for method chaining
        """
        self.audio_observable = audio_observable  # type: ignore[assignment]
        return self

    def emit_text(self) -> Observable:  # type: ignore[type-arg]
        """
        Create an observable that emits transcribed text from audio.

        Returns:
            Observable emitting transcribed text from audio recordings
        """
        if self.audio_observable is None:
            raise ValueError("No audio source provided. Call consume_audio() first.")

        def on_subscribe(observer, scheduler):
            logger.info("Starting Whisper transcription service")

            # Subscribe to the audio source
            def on_audio_event(event: AudioEvent) -> None:
                try:
                    if _USE_FASTER_WHISPER:
                        segments, _info = self.model.transcribe(
                            event.data.flatten(), **self.modelopts
                        )
                        text = " ".join(seg.text.strip() for seg in segments)
                    else:
                        result = self.model.transcribe(event.data.flatten(), **self.modelopts)
                        text = result["text"].strip()
                    observer.on_next(text)
                except Exception as e:
                    logger.error(f"Error processing audio event: {e}")
                    observer.on_error(e)

            # Set up subscription to audio source
            subscription = self.audio_observable.subscribe(
                on_next=on_audio_event,
                on_error=lambda e: observer.on_error(e),
                on_completed=lambda: observer.on_completed(),
            )

            # Return a disposable to clean up resources
            def dispose() -> None:
                subscription.dispose()

            return disposable.Disposable(dispose)

        return create(on_subscribe)


if __name__ == "__main__":
    from dimos.stream.audio.node_key_recorder import KeyRecorder
    from dimos.stream.audio.node_microphone import (
        SounddeviceAudioSource,
    )
    from dimos.stream.audio.node_normalizer import AudioNormalizer
    from dimos.stream.audio.node_output import SounddeviceAudioOutput
    from dimos.stream.audio.node_volume_monitor import monitor
    from dimos.stream.audio.text.node_stdout import TextPrinterNode
    from dimos.stream.audio.tts.node_openai import OpenAITTSNode
    from dimos.stream.audio.utils import keepalive

    # Create microphone source, recorder, and audio output
    mic = SounddeviceAudioSource()
    normalizer = AudioNormalizer()
    recorder = KeyRecorder()
    whisper_node = WhisperNode()
    output = SounddeviceAudioOutput(sample_rate=24000)

    normalizer.consume_audio(mic.emit_audio())
    recorder.consume_audio(normalizer.emit_audio())
    monitor(recorder.emit_audio())
    whisper_node.consume_audio(recorder.emit_recording())

    # Create and connect the text printer node
    text_printer = TextPrinterNode(prefix="USER: ")
    text_printer.consume_text(whisper_node.emit_text())

    tts_node = OpenAITTSNode()
    tts_node.consume_text(whisper_node.emit_text())

    output.consume_audio(tts_node.emit_audio())

    keepalive()
