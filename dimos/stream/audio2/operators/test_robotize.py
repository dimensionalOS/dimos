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

"""Tests for robotize effect."""

import time

from dimos.stream.audio2.input.signal import WaveformType, signal
from dimos.stream.audio2.operators import robotize
from dimos.stream.audio2.output.soundcard import speaker
from dimos.stream.audio2.types import AudioFormat, AudioSpec


def test_robotize_basic():
    """Test basic robotize effect."""
    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.5,
        duration=0.5,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(robotize(), speaker()).run()

    # Give cleanup threads time to finish
    time.sleep(1.0)


def test_robotize_custom_params():
    """Test robotize with custom parameters."""
    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.5,
        duration=0.5,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(
        robotize(pitch=1.5, carrier_freq=50.0, carrier_waveform="sine", ring_mix=0.8),
        speaker(),
    ).run()

    # Give cleanup threads time to finish
    time.sleep(1.0)


def test_robotize_low_pitch():
    """Test robotize with low pitch (deep robot voice)."""
    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.5,
        duration=0.5,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(
        robotize(pitch=0.8, carrier_freq=20.0, carrier_waveform="square", ring_mix=0.6),
        speaker(),
    ).run()

    # Give cleanup threads time to finish
    time.sleep(1.0)
