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

"""Local text-to-speech using pyttsx3."""

import tempfile
import time
from pathlib import Path
from typing import Optional

import numpy as np
import soundfile as sf
from reactivex import Observable, create

from dimos.stream.audio2.types import AudioEvent, RawAudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.input.tts_pyttsx3")

try:
    import pyttsx3

    PYTTSX3_AVAILABLE = True
except ImportError:
    PYTTSX3_AVAILABLE = False
    logger.warning("pyttsx3 not installed - pyttsx3_tts() will not be available")


def pyttsx3_tts(
    text: str,
    rate: int = 200,
    volume: float = 1.0,
    voice: Optional[str] = None,
) -> Observable[AudioEvent]:
    """Create a text-to-speech source using local pyttsx3 (offline).

    Synthesizes text using the system's local TTS engine (espeak on Linux,
    SAPI5 on Windows, NSSpeechSynthesizer on macOS). Quality is lower than
    OpenAI but works completely offline.

    Args:
        text: Text to synthesize
        rate: Speech rate in words per minute (default: 200)
        volume: Volume level 0.0-1.0 (default: 1.0)
        voice: Optional voice ID (None = system default)

    Returns:
        Observable that emits a single RawAudioEvent containing the synthesized speech

    Examples:
        # Basic local TTS
        pyttsx3_tts("Hello world").pipe(speaker()).run()

        # Slow speech with effects
        pyttsx3_tts("I am a robot", rate=150).pipe(
            robotize(),
            speaker()
        ).run()

        # Custom voice (list available with pyttsx3.init().getProperty('voices'))
        pyttsx3_tts("Custom voice", voice="english-us").pipe(speaker()).run()
    """

    def subscribe(observer, scheduler=None):
        """Subscribe to the TTS source."""
        if not PYTTSX3_AVAILABLE:
            observer.on_error(
                ImportError("pyttsx3 is not installed. Install it with: pip install pyttsx3")
            )
            return

        temp_file = None
        try:
            logger.info(f"Synthesizing text with pyttsx3 (rate={rate}, volume={volume})")

            # Initialize pyttsx3 engine
            engine = pyttsx3.init()
            engine.setProperty("rate", rate)
            engine.setProperty("volume", volume)

            # Set voice if specified
            if voice is not None:
                voices = engine.getProperty("voices")
                # Try to find matching voice
                for v in voices:
                    if voice.lower() in v.id.lower() or voice.lower() in v.name.lower():
                        engine.setProperty("voice", v.id)
                        logger.info(f"Using voice: {v.name}")
                        break

            # Create temporary WAV file
            temp_file = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
            temp_path = temp_file.name
            temp_file.close()

            # Synthesize to file
            engine.save_to_file(text, temp_path)
            engine.runAndWait()

            # Read the generated audio file
            audio_array, sample_rate = sf.read(temp_path, dtype=np.float32)

            logger.info(
                f"Synthesized {len(audio_array) / sample_rate:.2f}s of audio "
                f"at {sample_rate}Hz ({len(audio_array)} samples)"
            )

            # Clean up temp file
            Path(temp_path).unlink()

            # Create and emit audio event
            timestamp = time.time()
            channels = 1 if audio_array.ndim == 1 else audio_array.shape[1]

            event = RawAudioEvent(
                data=audio_array,
                sample_rate=sample_rate,
                channels=channels,
                timestamp=timestamp,
            )

            observer.on_next(event)
            observer.on_completed()

        except Exception as e:
            logger.error(f"Error synthesizing speech: {e}")
            # Clean up temp file on error
            if temp_file is not None:
                try:
                    Path(temp_file.name).unlink(missing_ok=True)
                except Exception:
                    pass
            observer.on_error(e)

    return create(subscribe)
