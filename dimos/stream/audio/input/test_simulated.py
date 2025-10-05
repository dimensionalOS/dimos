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

from dimos.stream.audio.input.simulated import SimulatedAudioSource
from dimos.stream.audio.output.mock import MockAudioOutput


def test_simulated_audio_source_basic():
    """Test basic functionality of SimulatedAudioSource."""
    # Create simulated source
    source = SimulatedAudioSource(
        sample_rate=16000, frame_length=1024, frequency=440.0, volume_oscillation=False
    )

    # Create mock output to capture events
    mock = MockAudioOutput()
    mock.consume_audio(source.emit_audio())

    # Wait for a few events
    assert mock.wait_for_events(3, timeout=2.0)

    # Stop and verify
    mock.stop()

    # Check format
    format_info = mock.get_format_info()
    assert format_info == (16000, 1, np.float32)

    # Check we received events
    assert len(mock.get_received_events()) >= 3
    assert mock.get_total_samples() >= 3 * 1024


def test_simulated_audio_waveforms():
    """Test different waveform types."""
    waveforms = ["sine", "square", "triangle", "sawtooth"]

    for waveform in waveforms:
        source = SimulatedAudioSource(waveform=waveform, frame_length=512, volume_oscillation=False)

        mock = MockAudioOutput()
        mock.consume_audio(source.emit_audio())

        # Get one frame
        assert mock.wait_for_events(1, timeout=1.0)
        mock.stop()

        # Verify we got audio
        events = mock.get_received_events()
        assert len(events) == 1
        assert len(events[0].data) == 512

        # Basic sanity check - audio should be in range [-1, 1]
        assert np.all(np.abs(events[0].data) <= 1.0)


def test_simulated_audio_stereo():
    """Test stereo audio generation."""
    source = SimulatedAudioSource(channels=2, frame_length=256, volume_oscillation=False)

    mock = MockAudioOutput()
    mock.consume_audio(source.emit_audio())

    assert mock.wait_for_events(1, timeout=1.0)
    mock.stop()

    # Check stereo format
    events = mock.get_received_events()
    assert events[0].channels == 2
    assert events[0].data.shape == (256, 2)


def test_simulated_audio_continuity():
    """Test that audio is generated continuously."""
    source = SimulatedAudioSource(sample_rate=8000, frame_length=100, volume_oscillation=False)

    mock = MockAudioOutput()
    mock.consume_audio(source.emit_audio(fps=100))  # High FPS for fast test

    # Collect several frames
    assert mock.wait_for_events(10, timeout=2.0)
    mock.stop()

    # Verify continuity (with loose tolerance for simulated source)
    assert mock.verify_continuous(tolerance_ms=100.0)
