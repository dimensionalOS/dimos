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

try:
    from dimos.stream.audio.input.gstreamer import GstreamerInput
    from dimos.stream.audio.output.gstreamer import GstreamerOutput

    GSTREAMER_AVAILABLE = True
except ImportError:
    GSTREAMER_AVAILABLE = False

from dimos.stream.audio.output.mock import MockAudioOutput
from dimos.stream.audio.input.simulated import SimulatedAudioSource


@pytest.mark.skipif(not GSTREAMER_AVAILABLE, reason="GStreamer not available")
def test_gstreamer_input_basic():
    """Test basic GStreamer input with test source."""
    # Create GStreamer input with audiotestsrc
    gst_input = GstreamerInput(
        pipeline_str="audiotestsrc wave=sine freq=440 num-buffers=10 ! audioconvert ! audioresample",
        sample_rate=16000,
        channels=1,
    )

    # Create mock output
    mock = MockAudioOutput()
    mock.consume_audio(gst_input.emit_audio())

    # Wait for events (audiotestsrc with num-buffers=10)
    assert mock.wait_for_events(5, timeout=3.0)
    mock.stop()

    # Verify format
    assert mock.get_format_info() == (16000, 1, np.float32)

    # Verify we got audio
    assert mock.get_total_samples() > 0


@pytest.mark.skipif(not GSTREAMER_AVAILABLE, reason="GStreamer not available")
def test_gstreamer_output_basic():
    """Test basic GStreamer output with simulated input."""
    # Create simulated source
    source = SimulatedAudioSource(sample_rate=16000, frame_length=1024, volume_oscillation=False)

    # Create GStreamer output (fakesink for testing)
    gst_output = GstreamerOutput(
        sample_rate=16000,
        device="fakesink",  # Use fakesink instead of real audio
    )

    # Connect pipeline
    gst_output.consume_audio(source.emit_audio())

    # Let it run briefly
    import time

    time.sleep(0.5)

    # Stop
    gst_output.stop()

    # If we get here without errors, the pipeline worked
    assert True


@pytest.mark.skipif(not GSTREAMER_AVAILABLE, reason="GStreamer not available")
def test_gstreamer_passthrough():
    """Test passing audio through mock to GStreamer output."""
    # Use simulated source instead of another GStreamer input
    source = SimulatedAudioSource(sample_rate=16000, frame_length=512, volume_oscillation=False)

    # Create mock to capture the audio
    mock = MockAudioOutput()

    # Create GStreamer output with fakesink
    gst_output = GstreamerOutput(sample_rate=16000, device="fakesink")

    # Connect pipeline: source -> mock -> gst_output
    mock.consume_audio(source.emit_audio())
    gst_output.consume_audio(mock.emit_audio())

    # Wait for some events
    assert mock.wait_for_events(5, timeout=2.0)

    # Stop pipeline
    mock.stop()
    gst_output.stop()

    # Verify we captured audio
    events = mock.get_received_events()
    assert len(events) >= 5
    assert all(event.sample_rate == 16000 for event in events)
