# Copyright 2025-2026 Dimensional Inc.

import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
from faster_whisper import WhisperModel
from reactivex import Observable, create

from dimos.msgs.sensor_msgs.Audio import Audio


@dataclass
class TranscriptionResult:
    """Transcription result container."""

    text: str
    segments: list[Any]  # faster_whisper segments (Segment objects)
    ts: float


class RealtimeTranscriber:
    """Realtime transcriber using faster-whisper.

    Accumulates audio chunks and transcribes when buffer exceeds threshold.
    """

    def __init__(
        self,
        model_size: str = "tiny.en",
        device: str = "cpu",
        compute_type: str = "float32",
        min_buffer_seconds: float = 3.0,
        sample_rate: int = 16000,
    ) -> None:
        """Initialize the transcriber.

        Args:
            model_size: faster-whisper model size (e.g., "tiny", "base", "small").
            device: Device to run on ("cpu", "cuda").
            compute_type: Compute type ("float32", "float16", "int8").
            min_buffer_seconds: Minimum audio duration to trigger transcription.
            sample_rate: Expected sample rate (Whisper requires 16000).
        """
        self.model = WhisperModel(model_size, device=device, compute_type=compute_type)
        self.min_buffer_seconds = min_buffer_seconds
        self.sample_rate = sample_rate

        self._buffer = np.array([], dtype=np.float32)
        self._buffer_lock = threading.Lock()
        self._observer: Any = None

    @property
    def transcription_stream(self) -> Observable[TranscriptionResult]:
        """Observable stream of transcription results."""

        def subscribe(observer: Any, scheduler: Any = None) -> Any:
            self._observer = observer
            return lambda: setattr(self, "_observer", None)

        return create(subscribe)

    def on_audio(self, msg: Audio) -> None:
        """Process incoming audio message."""
        if msg.sample_rate != self.sample_rate:
            # Basic warning - resampling would be needed here for robustness
            print(f"Warning: Audio sample rate {msg.sample_rate} != {self.sample_rate}")

        with self._buffer_lock:
            # Append new data
            self._buffer = np.concatenate((self._buffer, msg.data))

            # Check if we have enough data
            buffer_duration = len(self._buffer) / self.sample_rate
            if buffer_duration >= self.min_buffer_seconds:
                self._transcribe()

    def _transcribe(self) -> None:
        """Transcribe the current buffer."""
        try:
            # Run transcription
            # beam_size=5 is standard. vad_filter=True helps ignore silence.
            segments, info = self.model.transcribe(
                self._buffer,
                beam_size=5,
                vad_filter=True,
                language="en" if "en" in self.model.model.is_multilingual else None,
            )

            # Consume generator to get text
            segment_list = list(segments)
            full_text = " ".join([seg.text for seg in segment_list]).strip()

            if full_text and self._observer:
                result = TranscriptionResult(
                    text=full_text,
                    segments=segment_list,
                    ts=time.time(),
                )
                self._observer.on_next(result)

        except Exception as e:
            if self._observer:
                self._observer.on_error(e)
            else:
                print(f"Transcription error: {e}")
        finally:
            # Clear buffer after transcription (naive approach)
            self._buffer = np.array([], dtype=np.float32)
