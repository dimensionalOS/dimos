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

"""Tests for network audio output."""

import time

import pytest

from dimos.stream.audio2.input import microphone
from dimos.stream.audio2.input.file import file_input
from dimos.stream.audio2.input.signal import WaveformType, signal
from dimos.stream.audio2.input.tts_oai import Voice, openai_tts
from dimos.stream.audio2.operators import normalizer, raw_normalizer, robotize, vumeter
from dimos.stream.audio2.output.network import network_output
from dimos.stream.audio2.output.soundcard import speaker
from dimos.stream.audio2.types import AudioFormat, AudioSpec
from dimos.utils.data import get_data

host = "127.0.0.1"


@pytest.mark.tool
def test_network_output_to_server_signal():
    """Test streaming to actual server with a robotic melody (requires manual setup)."""

    # To run this test:
    # 1. Start the server: ./gstreamer_scripts/gstreamer.sh
    # 2. Run this test with: pytest test_network.py::test_network_output_to_server_signal

    from reactivex import concat

    # Simple robotic melody: C-E-G-C-G-E-C (arpeggio)
    # Note frequencies: C4=261.63, E4=329.63, G4=392.00, C5=523.25
    base_len = 0.2
    melody_notes = [
        (261.63, base_len),  # C4
        (329.63, base_len),  # E4
        (392.00, base_len),  # G4
        (523.25, base_len),  # C5
        (392.00, base_len),  # G4
        (329.63, base_len),  # E4
        (261.63, base_len * 2),  # C4 (longer)
    ]

    # Build list of signal observables
    notes = [
        signal(
            waveform=WaveformType.SINE,
            frequency=freq,
            volume=0.2,
            duration=dur,
            realtime=False,
            output=AudioSpec(format=AudioFormat.PCM_F32LE),
        )
        for freq, dur in melody_notes
    ]

    # Concatenate all notes into a melody and apply robotic effect
    concat(*notes).pipe(normalizer(), network_output(host=host, port=5002, codec="opus")).run()

    time.sleep(2.5)


@pytest.mark.tool
def test_network_output_to_server_file():
    """Test streaming to actual server (requires manual setup)."""

    # To run this test:
    # 1. Start the server: ./gstreamer_scripts/gstreamer.sh
    # 2. Run this test with: pytest test_network.py::test_network_output_to_server_file

    file_input(
        file_path=str(get_data("audio_bender") / "out_of_date.wav"),
        realtime=False,  # Fast playback for testing
    ).pipe(
        #        robotize(),
        vumeter(),
        normalizer(),
        network_output(host=host, port=5002, codec="opus"),
    ).run()

    time.sleep(0.2)


@pytest.mark.tool
def test_network_mic():
    """Test streaming to actual server (requires manual setup)."""

    # To run this test:
    # 1. Start the server: ./gstreamer_scripts/gstreamer.sh
    # 2. Run this test with: pytest test_network.py::test_network_output_to_server_file

    microphone().pipe(
        robotize(),
        normalizer(),
        vumeter(),
        network_output(host=host, port=5002, codec="opus"),
    ).run()

    time.sleep(0.2)


@pytest.mark.tool
def test_network_tts():
    """Test streaming to actual server (requires manual setup)."""

    # To run this test:
    # 1. Start the server: ./gstreamer_scripts/gstreamer.sh
    # 2. Run this test with: pytest test_network.py::test_network_output_to_server_file

    openai_tts("Hello from OpenAI").pipe(
        normalizer(),
        vumeter(),
        network_output(host=host, port=5002, codec="opus"),
    ).run()

    time.sleep(2.2)
