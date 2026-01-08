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

"""Tests for VU meter operator."""

import time
import pytest
from reactivex import operators as ops

from dimos.stream.audio2.input.signal import WaveformType, signal
from dimos.stream.audio2.operators.raw_vumeter import raw_vumeter
from dimos.stream.audio2.types import AudioFormat, AudioSpec


def test_vumeter_passthrough():
    """Test that vumeter passes audio through unchanged."""

    event_count = 0
    completed = False

    def on_next(value):
        nonlocal event_count
        event_count += 1
        # Verify event has expected properties
        assert hasattr(value, "data")
        assert hasattr(value, "sample_rate")
        assert hasattr(value, "channels")

    def on_completed():
        nonlocal completed
        completed = True

    # Process audio through VU meter
    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.5,
        duration=0.3,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(
        raw_vumeter(bar_length=30),
        ops.do_action(on_next=on_next, on_completed=on_completed),
    ).run()

    assert event_count > 0, "Expected events from raw VU meter"
    assert completed, "Observable did not complete"

    # Give cleanup threads time to finish
    time.sleep(0.2)


def test_vumeter_with_rms():
    """Test VU meter with RMS volume calculation."""
    from dimos.stream.audio2.operators.utils import calculate_rms_volume

    event_count = 0

    def on_next(value):
        nonlocal event_count
        event_count += 1

    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.5,
        duration=0.3,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(
        raw_vumeter(volume_func=calculate_rms_volume),
        ops.do_action(on_next=on_next),
    ).run()

    assert event_count > 0, "Expected events from RMS raw VU meter"

    # Give cleanup threads time to finish
    time.sleep(0.2)


def test_vumeter_minimal():
    """Test VU meter with minimal display (no percentage or activity)."""

    event_count = 0

    def on_next(value):
        nonlocal event_count
        event_count += 1

    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.5,
        duration=0.3,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(
        raw_vumeter(show_percentage=False, show_activity=False),
        ops.do_action(on_next=on_next),
    ).run()

    assert event_count > 0, "Expected events from minimal raw VU meter"

    # Give cleanup threads time to finish
    time.sleep(0.2)
