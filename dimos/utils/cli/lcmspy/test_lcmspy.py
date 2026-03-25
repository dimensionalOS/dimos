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

import time

import pytest

from dimos.protocol.pubsub.impl.lcmpubsub import PickleLCM, Topic
from dimos.utils.cli.lcmspy.lcmspy import GraphLCMSpy, GraphTopic, LCMSpy, Topic as TopicSpy


@pytest.fixture
def pickle_lcm():
    lcm = PickleLCM()
    lcm.start()
    yield lcm
    lcm.stop()


@pytest.fixture
def lcmspy_instance():
    spy = LCMSpy()
    spy.start()
    yield spy
    spy.stop()


@pytest.fixture
def graph_lcmspy_instance():
    spy = GraphLCMSpy(graph_log_window=0.1)
    spy.start()
    time.sleep(0.2)  # Wait for thread to start
    yield spy
    spy.stop()


def test_spy_basic(pickle_lcm, lcmspy_instance) -> None:
    video_topic = Topic(topic="/video")
    odom_topic = Topic(topic="/odom")

    for i in range(5):
        pickle_lcm.publish(video_topic, f"video frame {i}")
        time.sleep(0.1)
        if i % 2 == 0:
            pickle_lcm.publish(odom_topic, f"odometry data {i / 2}")

    # Wait a bit for messages to be processed
    time.sleep(0.5)

    # Test statistics for video topic
    video_topic_spy = lcmspy_instance.topic["/video"]
    assert video_topic_spy is not None

    # Test frequency (should be around 10 Hz for 5 messages in ~0.5 seconds)
    freq = video_topic_spy.freq(1.0)
    assert freq > 0
    print(f"Video topic frequency: {freq:.2f} Hz")

    # Test bandwidth
    kbps = video_topic_spy.kbps(1.0)
    assert kbps > 0
    print(f"Video topic bandwidth: {kbps:.2f} kbps")

    # Test average message size
    avg_size = video_topic_spy.size(1.0)
    assert avg_size > 0
    print(f"Video topic average message size: {avg_size:.2f} bytes")

    # Test statistics for odom topic
    odom_topic_spy = lcmspy_instance.topic["/odom"]
    assert odom_topic_spy is not None

    freq = odom_topic_spy.freq(1.0)
    assert freq > 0
    print(f"Odom topic frequency: {freq:.2f} Hz")

    kbps = odom_topic_spy.kbps(1.0)
    assert kbps > 0
    print(f"Odom topic bandwidth: {kbps:.2f} kbps")

    avg_size = odom_topic_spy.size(1.0)
    assert avg_size > 0
    print(f"Odom topic average message size: {avg_size:.2f} bytes")

    print(f"Video topic: {video_topic_spy}")
    print(f"Odom topic: {odom_topic_spy}")


def test_topic_statistics_direct() -> None:
    """Test Topic statistics directly without LCM"""

    topic = TopicSpy("/test")

    # Add some test messages
    test_data = [b"small", b"medium sized message", b"very long message for testing purposes"]

    for _i, data in enumerate(test_data):
        topic.msg(data)
        time.sleep(0.1)  # Simulate time passing

    # Test statistics over 1 second window
    freq = topic.freq(1.0)
    kbps = topic.kbps(1.0)
    avg_size = topic.size(1.0)

    assert freq > 0
    assert kbps > 0
    assert avg_size > 0

    print(f"Direct test - Frequency: {freq:.2f} Hz")
    print(f"Direct test - Bandwidth: {kbps:.2f} kbps")
    print(f"Direct test - Avg size: {avg_size:.2f} bytes")


def test_topic_cleanup() -> None:
    """Test that old messages are properly cleaned up"""

    topic = TopicSpy("/test")

    # Add a message
    topic.msg(b"test message")
    initial_count = len(topic.message_history)
    assert initial_count == 1

    # Simulate time passing by manually adding old timestamps
    old_time = time.time() - 70  # 70 seconds ago
    topic.message_history.appendleft((old_time, 10))

    # Trigger cleanup
    topic._cleanup_old_messages(max_age=60.0)

    # Should only have the recent message
    assert len(topic.message_history) == 1
    assert topic.message_history[0][0] > time.time() - 10  # Recent message


def test_graph_topic_basic() -> None:
    """Test GraphTopic basic functionality"""
    topic = GraphTopic("/test_graph")

    # Add some messages and update graphs
    topic.msg(b"test message")
    topic.update_graphs(1.0)

    # Should have history data
    assert len(topic.freq_history) == 1
    assert len(topic.bandwidth_history) == 1
    assert topic.freq_history[0] > 0
    assert topic.bandwidth_history[0] > 0


def test_graph_lcmspy_basic(graph_lcmspy_instance) -> None:
    """Test GraphLCMSpy basic functionality"""
    # Simulate a message
    graph_lcmspy_instance.msg("/test", b"test data")
    time.sleep(0.5)  # Wait for graph update — macOS needs longer for thread scheduling

    # Should create GraphTopic with history
    topic = graph_lcmspy_instance.topic["/test"]
    assert isinstance(topic, GraphTopic)
    assert len(topic.freq_history) > 0
    assert len(topic.bandwidth_history) > 0


def test_lcmspy_global_totals(lcmspy_instance) -> None:
    """Test that LCMSpy tracks global totals as a Topic itself"""
    # Send messages to different topics
    lcmspy_instance.msg("/video", b"video frame data")
    lcmspy_instance.msg("/odom", b"odometry data")
    lcmspy_instance.msg("/imu", b"imu data")

    # Verify each test topic received exactly one message (ignore LCM discovery packets)
    for t in ("/video", "/odom", "/imu"):
        assert len(lcmspy_instance.topic[t].message_history) == 1

    # Check global statistics
    global_freq = lcmspy_instance.freq(1.0)
    global_kbps = lcmspy_instance.kbps(1.0)
    global_size = lcmspy_instance.size(1.0)

    assert global_freq > 0
    assert global_kbps > 0
    assert global_size > 0

    print(f"Global frequency: {global_freq:.2f} Hz")
    print(f"Global bandwidth: {lcmspy_instance.kbps_hr(1.0)}")
    print(f"Global avg message size: {global_size:.0f} bytes")


def test_graph_lcmspy_global_totals(graph_lcmspy_instance) -> None:
    """Test that GraphLCMSpy tracks global totals with history"""
    # Send messages
    graph_lcmspy_instance.msg("/video", b"video frame data")
    graph_lcmspy_instance.msg("/odom", b"odometry data")
    time.sleep(0.2)  # Wait for graph update

    # Update global graphs
    graph_lcmspy_instance.update_graphs(1.0)

    # Should have global history
    assert len(graph_lcmspy_instance.freq_history) == 1
    assert len(graph_lcmspy_instance.bandwidth_history) == 1
    assert graph_lcmspy_instance.freq_history[0] > 0
    assert graph_lcmspy_instance.bandwidth_history[0] > 0

    print(f"Global frequency history: {graph_lcmspy_instance.freq_history[0]:.2f} Hz")
    print(f"Global bandwidth history: {graph_lcmspy_instance.bandwidth_history[0]:.2f} kB/s")
