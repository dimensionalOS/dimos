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

"""Tests for audio normalizer operator."""

import time
import numpy as np
import pytest
from reactivex import operators as ops

from dimos.stream.audio2.input.signal import WaveformType, signal
from dimos.stream.audio2.operators.raw_normalizer import raw_normalizer
from dimos.stream.audio2.types import AudioFormat, AudioSpec


def test_normalizer_basic():
    """Test that normalizer processes audio without errors."""

    event_count = 0
    completed = False

    def on_next(value):
        nonlocal event_count
        event_count += 1

    def on_completed():
        nonlocal completed
        completed = True

    # Generate low volume signal and normalize it
    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.1,  # Low volume
        duration=0.5,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(
        raw_normalizer(target_level=0.8),
        ops.do_action(on_next=on_next, on_completed=on_completed),
    ).run()

    assert event_count > 0, "Expected events from raw_normalizer"
    assert completed, "Observable did not complete"

    # Give cleanup threads time to finish
    time.sleep(0.2)


def test_normalizer_amplifies_quiet_audio():
    """Test that normalizer actually amplifies quiet audio."""

    volumes_before = []
    volumes_after = []

    def measure_before(event):
        # Measure volume before normalization
        peak = np.max(np.abs(event.data))
        volumes_before.append(peak)
        return event

    def measure_after(event):
        # Measure volume after normalization
        peak = np.max(np.abs(event.data))
        volumes_after.append(peak)
        return event

    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.1,  # Low volume
        duration=0.5,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(
        ops.map(measure_before),
        raw_normalizer(target_level=0.8, adapt_speed=0.5),  # Fast adaptation for testing
        ops.map(measure_after),
    ).run()

    # Check that we collected data
    assert len(volumes_before) > 0
    assert len(volumes_after) > 0

    # Average volume after normalization should be higher
    avg_before = np.mean(volumes_before)
    avg_after = np.mean(volumes_after)

    assert avg_after > avg_before, f"Volume not increased: {avg_before} -> {avg_after}"
    print(
        f"Normalization: {avg_before:.3f} -> {avg_after:.3f} (gain: {avg_after / avg_before:.2f}x)"
    )

    # Give cleanup threads time to finish
    time.sleep(0.2)


def test_normalizer_with_rms():
    """Test that normalizer works with RMS volume calculation."""
    from dimos.stream.audio2.operators.utils import calculate_rms_volume

    event_count = 0

    def on_next(value):
        nonlocal event_count
        event_count += 1

    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.2,
        duration=0.3,
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(
        raw_normalizer(volume_func=calculate_rms_volume),
        ops.do_action(on_next=on_next),
    ).run()

    assert event_count > 0, "Expected events from RMS raw_normalizer"

    # Give cleanup threads time to finish
    time.sleep(0.2)
