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

"""Tests for signal input source."""

import threading
import time

import pytest
from reactivex import operators as ops

from dimos.stream.audio2.input.signal import WaveformType, signal
from dimos.stream.audio2.types import AudioFormat, AudioSpec


def test_signal_input_completes():
    """Test that signal() emits events and properly completes the observable."""

    # Track events and completion
    event_count = 0
    completed = False

    def on_next(value):
        nonlocal event_count
        event_count += 1

    def on_completed():
        nonlocal completed
        completed = True

    # Subscribe and wait with run() - blocks until completion
    signal(
        waveform=WaveformType.SINE,
        frequency=440.0,
        volume=0.5,
        duration=0.5,  # Short duration for testing
        output=AudioSpec(format=AudioFormat.PCM_F32LE),
    ).pipe(ops.do_action(on_next=on_next, on_completed=on_completed)).run()

    # Check that we received events
    assert event_count > 0, f"Expected events but got {event_count}"

    # Check that the observable completed
    assert completed, "Observable did not complete"

    # Wait for threads to clean up
    max_wait = 2.0
    start = time.time()
    while time.time() - start < max_wait:
        signal_threads = [
            t
            for t in threading.enumerate()
            if "TestSignal" in t.name or "GStreamerMainLoop" in t.name
        ]
        if not signal_threads:
            break
        time.sleep(0.1)
