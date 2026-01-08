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

"""Tests for audio2 package __init__.py imports."""

import time


def test_all_imports():
    """Test that all exports from audio2 package can be imported."""
    from dimos.stream.audio2 import (
        AudioEvent,
        AudioFormat,
        AudioSpec,
        CompressedAudioEvent,
        RawAudioEvent,
        RemoteSpeechModule,
        SpeechModule,
        Voice,
        file_input,
        microphone,
        network_output,
        normalizer,
        openai_tts,
        pitch_shift,
        pyttsx3_tts,
        ring_modulator,
        robotize,
        signal,
        speaker,
        vumeter,
    )

    # Verify modules exist
    assert SpeechModule is not None
    assert RemoteSpeechModule is not None

    # Verify input functions exist
    assert callable(file_input)
    assert callable(microphone)
    assert callable(signal)
    assert callable(openai_tts)
    assert callable(pyttsx3_tts)

    # Verify operator functions exist
    assert callable(normalizer)
    assert callable(vumeter)
    assert callable(robotize)
    assert callable(pitch_shift)
    assert callable(ring_modulator)

    # Verify output functions exist
    assert callable(speaker)
    assert callable(network_output)

    # Verify types exist
    assert AudioEvent is not None
    assert RawAudioEvent is not None
    assert CompressedAudioEvent is not None
    assert AudioFormat is not None
    assert AudioSpec is not None
    assert Voice is not None


def test_simplified_api():
    """Test that simplified API works for common use cases."""
    from dimos.stream.audio2 import pitch_shift, robotize, signal, speaker

    # Create a simple pipeline (don't run it)
    pipeline = signal(frequency=440, duration=0.1).pipe(robotize(), pitch_shift(1.2))

    assert pipeline is not None


def test_types_usage():
    """Test that types can be used correctly."""
    from dimos.stream.audio2 import AudioFormat, AudioSpec, Voice

    # Test AudioFormat enum (value is just the format string, not full caps)
    assert AudioFormat.PCM_F32LE.value == "F32LE"
    assert AudioFormat.PCM_S16LE.value == "S16LE"

    # Test AudioSpec creation
    spec = AudioSpec(format=AudioFormat.PCM_F32LE, sample_rate=44100, channels=1)
    assert spec.sample_rate == 44100
    assert spec.channels == 1
    assert spec.format == AudioFormat.PCM_F32LE

    # Test Voice enum
    assert Voice.ECHO.value == "echo"
    assert "alloy" in [v.value for v in Voice]
