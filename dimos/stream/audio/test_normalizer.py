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

from dimos.stream.audio.node_normalizer import AudioNormalizer
from dimos.stream.audio.input.simulated import SimulatedAudioSource
from dimos.stream.audio.output.mock import MockAudioOutput


def test_normalizer_basic():
    """Test basic audio normalization."""
    # Create quiet source
    source = SimulatedAudioSource(frame_length=1024, volume_oscillation=False)

    # Create normalizer
    normalizer = AudioNormalizer(target_level=0.8, max_gain=10.0)

    # Create mock output
    mock = MockAudioOutput()

    # Connect pipeline
    normalizer.consume_audio(source.emit_audio())
    mock.consume_audio(normalizer.emit_audio())

    # Collect some frames
    assert mock.wait_for_events(5, timeout=2.0)
    mock.stop()

    # Verify we got normalized audio
    events = mock.get_received_events()
    assert len(events) >= 5

    # Check that audio was processed (format preserved)
    assert mock.get_format_info() == (16000, 1, np.float32)


def test_normalizer_gain_limits():
    """Test that normalizer respects gain limits."""
    # Create a very quiet sine wave
    source = SimulatedAudioSource(frame_length=512, volume_oscillation=False)

    # Create normalizer with limited gain
    normalizer = AudioNormalizer(
        target_level=1.0,
        max_gain=2.0,  # Low max gain
    )

    # Create two mocks - one for source, one for normalized
    mock_source = MockAudioOutput()
    mock_normalized = MockAudioOutput()

    # Split the source to both mocks
    mock_source.consume_audio(source.emit_audio())
    normalizer.consume_audio(mock_source.emit_audio())
    mock_normalized.consume_audio(normalizer.emit_audio())

    # Collect frames
    assert mock_normalized.wait_for_events(3, timeout=2.0)
    mock_source.stop()
    mock_normalized.stop()

    # Compare volumes
    source_events = mock_source.get_received_events()
    normalized_events = mock_normalized.get_received_events()

    if len(source_events) > 0 and len(normalized_events) > 0:
        # Calculate RMS of first event
        source_rms = np.sqrt(np.mean(source_events[0].data ** 2))
        normalized_rms = np.sqrt(np.mean(normalized_events[0].data ** 2))

        # Gain should not exceed max_gain
        if source_rms > 0:
            actual_gain = normalized_rms / source_rms
            assert actual_gain <= 2.1  # Allow small tolerance


def test_normalizer_passthrough():
    """Test that normalizer passes through audio events."""
    source = SimulatedAudioSource(sample_rate=24000, channels=2, frame_length=2048)

    normalizer = AudioNormalizer()
    mock = MockAudioOutput()

    # Connect pipeline
    normalizer.consume_audio(source.emit_audio())
    mock.consume_audio(normalizer.emit_audio())

    # Get one frame
    assert mock.wait_for_events(1, timeout=1.0)
    mock.stop()

    # Check format is preserved
    assert mock.get_format_info() == (24000, 2, np.float32)
