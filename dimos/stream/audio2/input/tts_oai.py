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

"""Text-to-speech input sources for audio pipeline."""

import io
import time
from enum import Enum
from typing import Optional

import numpy as np
import soundfile as sf
from openai import OpenAI
from reactivex import Observable, create

from dimos.stream.audio2.types import AudioEvent, RawAudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.input.tts")


class Voice(str, Enum):
    """Available voices in OpenAI TTS API."""

    ALLOY = "alloy"
    ECHO = "echo"
    FABLE = "fable"
    ONYX = "onyx"
    NOVA = "nova"
    SHIMMER = "shimmer"


def openai_tts(
    text: str,
    voice: Voice = Voice.ECHO,
    model: str = "tts-1",
    speed: float = 1.0,
    api_key: Optional[str] = None,
) -> Observable[AudioEvent]:
    """Create a text-to-speech source using OpenAI's TTS API.

    Synthesizes the provided text to speech and emits it as raw PCM audio.
    OpenAI TTS returns audio at 24kHz sample rate.

    Args:
        text: Text to synthesize
        voice: Voice to use (default: ECHO)
        model: TTS model to use (default: "tts-1", can use "tts-1-hd" for higher quality)
        speed: Speech speed multiplier 0.25-4.0 (default: 1.0)
        api_key: OpenAI API key (if None, uses environment variable OPENAI_API_KEY)

    Returns:
        Observable that emits a single RawAudioEvent containing the synthesized speech

    Examples:
        # Basic TTS
        openai_tts("Hello world").pipe(speaker()).run()

        # Custom voice and effects
        openai_tts(
            "I am a robot",
            voice=Voice.ONYX,
            speed=0.9
        ).pipe(
            robotize(),
            speaker()
        ).run()

        # High quality TTS with pitch shift
        openai_tts(
            "Testing high quality audio",
            model="tts-1-hd",
            voice=Voice.NOVA
        ).pipe(
            pitch_shift(1.2),
            speaker()
        ).run()
    """

    def subscribe(observer, scheduler=None):
        """Subscribe to the TTS source."""
        try:
            logger.info(f"Synthesizing text with OpenAI TTS (voice={voice.value}, model={model})")

            # Initialize OpenAI client
            client = OpenAI(api_key=api_key)

            # Call OpenAI TTS API
            response = client.audio.speech.create(
                model=model, voice=voice.value, input=text, speed=speed
            )

            # Convert response to audio data
            audio_data = io.BytesIO(response.content)

            # Read with soundfile (OpenAI returns MP3/Opus format)
            with sf.SoundFile(audio_data, "r") as sound_file:
                sample_rate = sound_file.samplerate
                audio_array = sound_file.read(dtype=np.float32)

            logger.info(
                f"Synthesized {len(audio_array) / sample_rate:.2f}s of audio "
                f"at {sample_rate}Hz ({len(audio_array)} samples)"
            )

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
            observer.on_error(e)

    return create(subscribe)
