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

"""Tests for pitch shift effect."""

import time

from dimos.stream.audio2.input.signal import WaveformType, signal
from dimos.stream.audio2.operators.effects import pitch_shift
from dimos.stream.audio2.output.soundcard import speaker
from dimos.stream.audio2.types import AudioFormat, AudioSpec


def test_pitch_shift_basic():
    """Test basic pitch shift with speaker output."""
    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.3,
        duration=1.0,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(
        pitch_shift(1.5),  # Shift up by a fifth
        speaker(),
    ).run()

    # Give cleanup threads time to finish
    time.sleep(0.2)


def test_pitch_shift_down():
    """Test pitch shift down."""
    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.3,
        duration=1.0,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(
        pitch_shift(0.5),  # Shift down an octave
        speaker(),
    ).run()

    # Give cleanup threads time to finish
    time.sleep(0.2)


def test_pitch_shift_no_change():
    """Test pitch shift with no change (pitch=1.0)."""
    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.3,
        duration=0.5,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(
        pitch_shift(1.0),  # No change
        speaker(),
    ).run()

    # Give cleanup threads time to finish
    time.sleep(0.2)
