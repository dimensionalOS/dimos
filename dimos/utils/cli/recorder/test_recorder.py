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

import shutil
import time
from contextlib import contextmanager
from pathlib import Path

import pytest

from dimos import core
from dimos_lcm.sensor_msgs import Image
from dimos.utils.data import _get_data_dir

from .recorder import RecorderService


@contextmanager
def temp_recording_dir(recording_name: str):
    """Context manager for temporary recording directories that auto-delete on exit.

    Args:
        recording_name: Name for the recording subdirectory within data/

    Yields:
        Path: Path to the temporary recording directory

    Example:
        with temp_recording_dir("my_test_recording") as record_dir:
            lidar_store = TimedSensorStorage(f"{recording_name}/lidar")
            # ... do recording ...
        # Directory is automatically deleted after exiting the context
    """
    record_dir = _get_data_dir(recording_name)

    # Clean up any existing directory
    if record_dir.exists():
        shutil.rmtree(record_dir)

    try:
        yield record_dir
    finally:
        # Clean up on exit
        if record_dir.exists():
            shutil.rmtree(record_dir)


@pytest.mark.lcm
def test_recorder_service_basic():
    """Test basic recorder service functionality."""
    # Start recorder service
    recorder = RecorderService(autoconf=True)
    recorder.start()

    # Let it discover topics for a moment
    time.sleep(0.5)

    # Should have discovered some topics (at least from this test)
    # Note: might be empty if no other LCM publishers are running
    initial_topic_count = len(recorder.topics)

    # Publish a test message
    test_topic = "/test/recorder"
    pub = core.LCMTransport(test_topic, Image)
    # Create a simple test image with minimal data
    test_image = Image()
    test_image.width = 640
    test_image.height = 480
    test_image.encoding = "rgb8"
    test_image.step = 640 * 3  # 3 bytes per pixel for RGB
    test_image.data = bytes(10)  # Just a small amount of data for testing
    pub.publish(test_image)

    # Give it time to receive
    time.sleep(0.1)

    # Should have discovered the test topic (with type suffix)
    # Find the topic that starts with our test topic name
    found_topics = [t for t in recorder.topics if t.startswith(test_topic)]
    assert len(found_topics) > 0, f"Topic {test_topic} not found in {list(recorder.topics.keys())}"

    actual_topic_name = found_topics[0]
    assert len(recorder.topics) >= initial_topic_count + 1

    # Check topic stats
    topic_info = recorder.topics[actual_topic_name]
    assert topic_info.freq(1.0) > 0
    assert topic_info.total_traffic() > 0

    recorder.stop()


@pytest.mark.lcm
def test_topic_selection():
    """Test topic selection functionality."""
    recorder = RecorderService(autoconf=True)
    recorder.start()

    # Publish some test messages
    topics = ["/test/topic1", "/test/topic2", "/test/topic3"]
    publishers = []
    for topic in topics:
        pub = core.LCMTransport(topic, Image)
        img = Image()
        img.width = 100
        img.height = 100
        img.encoding = "rgb8"
        img.step = 100 * 3
        img.data = bytes(10)
        pub.publish(img)
        publishers.append(pub)

    time.sleep(0.1)

    # All topics should be unselected by default
    # Find actual topic names (with type suffixes)
    actual_topics = {}
    for topic in topics:
        found = [t for t in recorder.topics if t.startswith(topic)]
        assert len(found) > 0, f"Topic {topic} not found"
        actual_topics[topic] = found[0]
        assert not recorder.topics[actual_topics[topic]].selected

    # Test individual selection
    recorder.toggle_topic_selection(actual_topics[topics[0]])
    assert recorder.topics[actual_topics[topics[0]]].selected
    assert not recorder.topics[actual_topics[topics[1]]].selected

    # Test select all
    recorder.select_all_topics(True)
    for topic in actual_topics.values():
        assert recorder.topics[topic].selected

    # Test deselect all
    recorder.select_all_topics(False)
    for topic in actual_topics.values():
        assert not recorder.topics[topic].selected

    # Test get selected topics
    recorder.toggle_topic_selection(actual_topics[topics[1]])
    recorder.toggle_topic_selection(actual_topics[topics[2]])
    selected = recorder.get_selected_topics()
    assert len(selected) == 2
    assert actual_topics[topics[1]] in selected
    assert actual_topics[topics[2]] in selected

    recorder.stop()


@pytest.mark.lcm
def test_recording():
    """Test recording functionality."""
    # Clean up any existing test recordings
    test_recording_dir = _get_data_dir("recordings")
    if test_recording_dir.exists():
        shutil.rmtree(test_recording_dir)

    recorder = RecorderService(autoconf=True)
    recorder.start()

    # Set up test topics
    topic1 = "/test/record1"
    topic2 = "/test/record2"

    pub1 = core.LCMTransport(topic1, Image)
    pub2 = core.LCMTransport(topic2, Image)

    # Publish initial messages to discover topics
    img1 = Image()
    img1.width = 200
    img1.height = 200
    img1.encoding = "rgb8"
    img1.step = 200 * 3
    img1.data = bytes(10)
    pub1.publish(img1)

    img2 = Image()
    img2.width = 300
    img2.height = 300
    img2.encoding = "rgb8"
    img2.step = 300 * 3
    img2.data = bytes(10)
    pub2.publish(img2)
    time.sleep(0.1)

    # Find actual topic names and select topics for recording
    actual_topic1 = [t for t in recorder.topics if t.startswith(topic1)][0]
    actual_topic2 = [t for t in recorder.topics if t.startswith(topic2)][0]
    recorder.toggle_topic_selection(actual_topic1)
    recorder.toggle_topic_selection(actual_topic2)

    # Start recording
    assert recorder.start_recording()
    assert recorder.recording
    assert recorder.recording_dir is not None
    assert recorder.recording_dir.exists()

    # Publish more messages while recording
    for i in range(5):
        img1 = Image()
        img1.width = 200
        img1.height = 200
        img1.encoding = "rgb8"
        img1.step = 200 * 3
        img1.data = bytes([i] * 10)
        pub1.publish(img1)

        img2 = Image()
        img2.width = 300
        img2.height = 300
        img2.encoding = "rgb8"
        img2.step = 300 * 3
        img2.data = bytes([i] * 10)
        pub2.publish(img2)
        time.sleep(0.1)

    # Check recording duration
    duration = recorder.get_recording_duration()
    assert duration > 0.5  # Should be at least 0.5 seconds

    # Stop recording
    recording_dir = recorder.stop_recording()
    assert recording_dir is not None
    assert recording_dir.exists()
    assert not recorder.recording

    # Check that files were created
    # Topics should be saved with / replaced by _ and type suffix removed
    # The actual directory names will include the type suffix
    topic1_dir = recording_dir / actual_topic1.replace("/", "_").strip("_")
    topic2_dir = recording_dir / actual_topic2.replace("/", "_").strip("_")

    assert topic1_dir.exists()
    assert topic2_dir.exists()

    # Check that pickle files were created
    topic1_files = list(topic1_dir.glob("*.pickle"))
    topic2_files = list(topic2_dir.glob("*.pickle"))

    assert len(topic1_files) >= 5  # At least 5 messages recorded
    assert len(topic2_files) >= 5

    # Clean up
    recorder.stop()
    if test_recording_dir.exists():
        shutil.rmtree(test_recording_dir)


@pytest.mark.lcm
def test_recording_with_temp_dir():
    """Test recording using temp_recording_dir context manager."""
    with temp_recording_dir("test_temp_recording") as record_dir:
        recorder = RecorderService(autoconf=True, recording_base_dir="test_temp_recording")
        recorder.start()

        # Publish test message
        test_topic = "/test/temp"
        pub = core.LCMTransport(test_topic, Image)
        img = Image()
        img.width = 100
        img.height = 100
        img.encoding = "rgb8"
        img.step = 100 * 3
        img.data = bytes(10)
        pub.publish(img)
        time.sleep(0.1)

        # Find actual topic name and select for recording
        actual_topic = [t for t in recorder.topics if t.startswith(test_topic)][0]
        recorder.toggle_topic_selection(actual_topic)
        assert recorder.start_recording()

        # Record some messages
        for i in range(3):
            img = Image()
            img.width = 100
            img.height = 100
            img.encoding = "rgb8"
            img.step = 100 * 3
            img.data = bytes([i] * 10)
            pub.publish(img)
            time.sleep(0.1)

        recording_path = recorder.stop_recording()
        assert recording_path is not None
        assert recording_path.exists()

        # Directory should still exist inside context
        assert record_dir.exists()

        recorder.stop()

    # Directory should be cleaned up after context
    assert not record_dir.exists()
