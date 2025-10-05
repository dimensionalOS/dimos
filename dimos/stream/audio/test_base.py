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

from dimos.stream.audio.base import AudioEvent


def test_audio_event_creation():
    """Test AudioEvent creation and properties."""
    data = np.random.randn(1024).astype(np.float32)
    event = AudioEvent(data=data, sample_rate=16000, timestamp=1234567890.0, channels=1)

    assert event.sample_rate == 16000
    assert event.timestamp == 1234567890.0
    assert event.channels == 1
    assert event.dtype == np.float32
    assert event.shape == (1024,)
    assert np.array_equal(event.data, data)


def test_audio_event_to_float32():
    """Test conversion to float32."""
    # Start with int16 data
    data_int16 = np.array([0, 16384, -16384, 32767, -32768], dtype=np.int16)
    event = AudioEvent(data_int16, sample_rate=8000, timestamp=0.0)

    # Convert to float32
    float_event = event.to_float32()

    # Check conversion
    assert float_event.dtype == np.float32
    expected = np.array([0.0, 0.5, -0.5, 32767 / 32768, -1.0], dtype=np.float32)
    np.testing.assert_allclose(float_event.data, expected, rtol=1e-4)

    # Converting float32 to float32 should return same object
    float_event2 = float_event.to_float32()
    assert float_event2 is float_event


def test_audio_event_to_int16():
    """Test conversion to int16."""
    # Start with float32 data
    data_float = np.array([0.0, 0.5, -0.5, 1.0, -1.0], dtype=np.float32)
    event = AudioEvent(data_float, sample_rate=8000, timestamp=0.0)

    # Convert to int16
    int_event = event.to_int16()

    # Check conversion
    assert int_event.dtype == np.int16
    expected = np.array([0, 16383, -16383, 32767, -32767], dtype=np.int16)
    np.testing.assert_allclose(int_event.data, expected, atol=1)

    # Converting int16 to int16 should return same object
    int_event2 = int_event.to_int16()
    assert int_event2 is int_event


def test_audio_event_multichannel():
    """Test multichannel audio event."""
    # Create stereo data
    data = np.random.randn(1024, 2).astype(np.float32)
    event = AudioEvent(data, sample_rate=44100, timestamp=0.0, channels=2)

    assert event.channels == 2
    assert event.shape == (1024, 2)
    assert event.dtype == np.float32
