#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

import pytest
import numpy as np
import time

from dimos.stream.audio.output.mock import MockAudioOutput
from dimos.stream.audio.input.simulated import SimulatedAudioSource


def test_mock_output_collection():
    """Test that MockAudioOutput correctly collects events."""
    source = SimulatedAudioSource(sample_rate=8000, frame_length=256, volume_oscillation=False)

    mock = MockAudioOutput()
    mock.consume_audio(source.emit_audio())

    # Collect some events
    assert mock.wait_for_events(5, timeout=2.0)
    mock.stop()

    # Verify collection
    events = mock.get_received_events()
    assert len(events) == 5
    assert all(len(e.data) == 256 for e in events)
    assert mock.get_total_samples() == 5 * 256


def test_mock_output_timing():
    """Test that MockAudioOutput records timing information."""
    source = SimulatedAudioSource(sample_rate=10000, frame_length=100)

    mock = MockAudioOutput()
    start_time = time.time()
    mock.consume_audio(source.emit_audio(fps=50))  # Fast generation

    assert mock.wait_for_events(3, timeout=1.0)
    mock.stop()

    # Check timing
    assert len(mock.receive_times) == 3
    assert all(t >= start_time for t in mock.receive_times)
    assert all(
        mock.receive_times[i] <= mock.receive_times[i + 1]
        for i in range(len(mock.receive_times) - 1)
    )


def test_mock_output_clear():
    """Test clearing collected events."""
    source = SimulatedAudioSource()
    mock = MockAudioOutput()

    mock.consume_audio(source.emit_audio())
    assert mock.wait_for_events(2, timeout=1.0)

    # Clear and verify
    mock.clear()
    assert len(mock.get_received_events()) == 0
    assert len(mock.receive_times) == 0
    assert mock.get_total_samples() == 0

    mock.stop()


def test_mock_output_passthrough():
    """Test that MockAudioOutput can pass through audio."""
    source = SimulatedAudioSource(frame_length=128)
    mock1 = MockAudioOutput()
    mock2 = MockAudioOutput()

    # Chain: source -> mock1 -> mock2
    mock1.consume_audio(source.emit_audio())
    mock2.consume_audio(mock1.emit_audio())

    assert mock2.wait_for_events(2, timeout=1.0)
    mock1.stop()
    mock2.stop()

    # Both should have received same events
    assert len(mock1.get_received_events()) == len(mock2.get_received_events())
    assert mock1.get_total_samples() == mock2.get_total_samples()
