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

"""Tests for soundcard output."""

import threading
import time

import pytest

from dimos.stream.audio2.input.file import file_input
from dimos.stream.audio2.input.signal import WaveformType, signal
from dimos.stream.audio2.output.soundcard import speaker
from dimos.stream.audio2.types import AudioFormat, AudioSpec
from dimos.utils.data import get_data


def test_speaker_with_signal():
    # Clean API: pipe speaker and run to block
    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.5,  # Moderate volume for testing
        duration=0.5,  # Short duration for testing
        output=AudioSpec(format=AudioFormat.PCM_F32LE),  # Raw audio
    ).pipe(speaker()).run()


def test_speaker_with_signal_encoded():
    # Clean API: pipe speaker and run to block
    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.5,  # Moderate volume for testing
        duration=0.5,  # Short duration for testing
    ).pipe(speaker()).run()


def test_speaker_with_file():
    # Clean API: pipe speaker and run to block
    file_input(
        file_path=str(get_data("audio_bender") / "out_of_date.wav"),
        realtime=True,  # Real-time playback
        output=AudioSpec(format=AudioFormat.PCM_F32LE),  # Output raw audio
    ).pipe(speaker()).run()
