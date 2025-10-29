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

import time

import pytest

from dimos.protocol.pubsub.lcmpubsub import PickleLCM, Topic
from dimos.utils.cli.lcmspy.lcmspy import GraphLCMSpy, GraphTopic, LCMSpy, Topic as TopicSpy


@pytest.mark.lcm
def test_spy_basic() -> None:
    lcm = PickleLCM(autoconf=True)
    lcm.start()

    lcmspy = LCMSpy(autoconf=True)
    lcmspy.start()

    video_topic = Topic(topic="/video")
    odom_topic = Topic(topic="/odom")

    for i in range(5):
        lcm.publish(video_topic, f"video frame {i}")
        time.sleep(0.1)
        if i % 2 == 0:
            lcm.publish(odom_topic, f"odometry data {i / 2}")

    # Wait a bit for messages to be processed
    time.sleep(0.5)

    # Test statistics for video topic
    video_topic_spy = lcmspy.topic["/video"]
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
    odom_topic_spy = lcmspy.topic["/odom"]
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


@pytest.mark.lcm
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


@pytest.mark.lcm
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


@pytest.mark.lcm
def test_graph_lcmspy_basic() -> None:
    """Test GraphLCMSpy basic functionality"""
    spy = GraphLCMSpy(autoconf=True, graph_log_window=0.1)
    spy.start()
    time.sleep(0.2)  # Wait for thread to start

    # Simulate a message
    spy.msg("/test", b"test data")
    time.sleep(0.2)  # Wait for graph update

    # Should create GraphTopic with history
    topic = spy.topic["/test"]
    assert isinstance(topic, GraphTopic)
    assert len(topic.freq_history) > 0
    assert len(topic.bandwidth_history) > 0

    spy.stop()


@pytest.mark.lcm
def test_lcmspy_global_totals() -> None:
    """Test that LCMSpy tracks global totals as a Topic itself"""
    spy = LCMSpy(autoconf=True)
    spy.start()

    # Send messages to different topics
    spy.msg("/video", b"video frame data")
    spy.msg("/odom", b"odometry data")
    spy.msg("/imu", b"imu data")

    # The spy itself should have accumulated all messages
    assert len(spy.message_history) == 3

    # Check global statistics
    global_freq = spy.freq(1.0)
    global_kbps = spy.kbps(1.0)
    global_size = spy.size(1.0)

    assert global_freq > 0
    assert global_kbps > 0
    assert global_size > 0

    print(f"Global frequency: {global_freq:.2f} Hz")
    print(f"Global bandwidth: {spy.kbps_hr(1.0)}")
    print(f"Global avg message size: {global_size:.0f} bytes")

    spy.stop()


@pytest.mark.lcm
def test_graph_lcmspy_global_totals() -> None:
    """Test that GraphLCMSpy tracks global totals with history"""
    spy = GraphLCMSpy(autoconf=True, graph_log_window=0.1)
    spy.start()
    time.sleep(0.2)

    # Send messages
    spy.msg("/video", b"video frame data")
    spy.msg("/odom", b"odometry data")
    time.sleep(0.2)  # Wait for graph update

    # Update global graphs
    spy.update_graphs(1.0)

    # Should have global history
    assert len(spy.freq_history) == 1
    assert len(spy.bandwidth_history) == 1
    assert spy.freq_history[0] > 0
    assert spy.bandwidth_history[0] > 0

    print(f"Global frequency history: {spy.freq_history[0]:.2f} Hz")
    print(f"Global bandwidth history: {spy.bandwidth_history[0]:.2f} kB/s")

    spy.stop()
