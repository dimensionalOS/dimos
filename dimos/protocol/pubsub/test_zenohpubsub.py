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

"""Tests for Zenoh pub/sub implementation."""

import threading
import time

import pytest

from dimos.protocol.pubsub.zenohpubsub import (
    CongestionControl,
    PickleZenoh,
    Priority,
    Reliability,
    ZenohConfig,
    ZenohPubSubBase,
    ZenohQoS,
)


class TestZenohQoS:
    """Tests for ZenohQoS configuration."""

    def test_default_qos(self) -> None:
        """Default QoS should be reliable with block congestion control."""
        qos = ZenohQoS()
        assert qos.reliability == Reliability.RELIABLE
        assert qos.congestion_control == CongestionControl.BLOCK
        assert qos.priority == Priority.DATA
        assert qos.express is False

    def test_custom_qos(self) -> None:
        """Custom QoS settings should be preserved."""
        qos = ZenohQoS(
            reliability=Reliability.BEST_EFFORT,
            congestion_control=CongestionControl.DROP,
            priority=Priority.REAL_TIME,
            express=True,
        )
        assert qos.reliability == Reliability.BEST_EFFORT
        assert qos.congestion_control == CongestionControl.DROP
        assert qos.priority == Priority.REAL_TIME
        assert qos.express is True


class TestZenohConfig:
    """Tests for ZenohConfig configuration."""

    def test_default_config(self) -> None:
        """Default config should have SHM enabled and peer mode."""
        config = ZenohConfig()
        assert config.shm_enabled is True
        assert config.mode == "peer"
        assert config.locators == []
        assert config.listen_endpoints == []

    def test_custom_config(self) -> None:
        """Custom config should be preserved."""
        config = ZenohConfig(
            locators=["tcp/192.168.1.1:7447"],
            shm_enabled=False,
            mode="client",
        )
        assert config.locators == ["tcp/192.168.1.1:7447"]
        assert config.shm_enabled is False
        assert config.mode == "client"


class TestZenohPubSubBase:
    """Tests for ZenohPubSubBase lifecycle."""

    def test_start_stop(self) -> None:
        """Session should start and stop cleanly."""
        pubsub = ZenohPubSubBase()
        pubsub.start()
        assert pubsub._started is True
        assert pubsub._session is not None

        pubsub.stop()
        assert pubsub._started is False
        assert pubsub._session is None

    def test_context_manager(self) -> None:
        """Context manager should handle start/stop."""
        with ZenohPubSubBase() as pubsub:
            assert pubsub._started is True
        assert pubsub._started is False

    def test_double_start_is_noop(self) -> None:
        """Calling start() twice should be safe."""
        pubsub = ZenohPubSubBase()
        pubsub.start()
        session1 = pubsub._session
        pubsub.start()  # Should be no-op
        assert pubsub._session is session1
        pubsub.stop()

    def test_double_stop_is_safe(self) -> None:
        """Calling stop() twice should be safe."""
        pubsub = ZenohPubSubBase()
        pubsub.start()
        pubsub.stop()
        pubsub.stop()  # Should not raise
        assert pubsub._started is False


class TestPickleZenoh:
    """Tests for PickleZenoh pub/sub."""

    def test_publish_subscribe_dict(self) -> None:
        """Should be able to publish and subscribe to dictionaries."""
        received: list[dict] = []  # type: ignore[type-arg]
        event = threading.Event()

        def callback(msg: dict, _topic: str) -> None:  # type: ignore[type-arg]
            received.append(msg)
            event.set()

        with PickleZenoh() as pubsub:
            pubsub.subscribe("test/dict", callback)
            time.sleep(0.1)  # Allow subscription to propagate

            test_data = {"key": "value", "number": 42, "nested": {"a": 1}}
            pubsub.publish("test/dict", test_data)

            assert event.wait(timeout=2.0), "Timeout waiting for message"
            assert len(received) == 1
            assert received[0] == test_data

    def test_publish_subscribe_list(self) -> None:
        """Should be able to publish and subscribe to lists."""
        received: list[list] = []  # type: ignore[type-arg]
        event = threading.Event()

        def callback(msg: list, _topic: str) -> None:  # type: ignore[type-arg]
            received.append(msg)
            event.set()

        with PickleZenoh() as pubsub:
            pubsub.subscribe("test/list", callback)
            time.sleep(0.1)

            test_data = [1, 2, 3, "four", {"five": 5}]
            pubsub.publish("test/list", test_data)

            assert event.wait(timeout=2.0), "Timeout waiting for message"
            assert received[0] == test_data

    def test_multiple_subscribers(self) -> None:
        """Multiple subscribers should all receive messages."""
        received1: list[str] = []
        received2: list[str] = []
        event1 = threading.Event()
        event2 = threading.Event()

        def callback1(msg: str, _topic: str) -> None:
            received1.append(msg)
            event1.set()

        def callback2(msg: str, _topic: str) -> None:
            received2.append(msg)
            event2.set()

        with PickleZenoh() as pubsub:
            pubsub.subscribe("test/multi", callback1)
            pubsub.subscribe("test/multi", callback2)
            time.sleep(0.1)

            pubsub.publish("test/multi", "hello")

            assert event1.wait(timeout=2.0), "Timeout waiting for subscriber 1"
            assert event2.wait(timeout=2.0), "Timeout waiting for subscriber 2"
            assert received1 == ["hello"]
            assert received2 == ["hello"]

    def test_unsubscribe(self) -> None:
        """Unsubscribe should stop receiving messages."""
        received: list[str] = []

        def callback(msg: str, _topic: str) -> None:
            received.append(msg)

        with PickleZenoh() as pubsub:
            unsubscribe = pubsub.subscribe("test/unsub", callback)
            time.sleep(0.1)

            pubsub.publish("test/unsub", "first")
            time.sleep(0.1)
            assert len(received) == 1

            unsubscribe()
            time.sleep(0.1)

            pubsub.publish("test/unsub", "second")
            time.sleep(0.2)
            # Should still only have first message
            assert len(received) == 1
            assert received[0] == "first"

    def test_qos_best_effort(self) -> None:
        """Best effort QoS should work."""
        received: list[str] = []
        event = threading.Event()

        def callback(msg: str, _topic: str) -> None:
            received.append(msg)
            event.set()

        qos = ZenohQoS(reliability=Reliability.BEST_EFFORT)
        with PickleZenoh(qos=qos) as pubsub:
            pubsub.subscribe("test/besteffort", callback)
            time.sleep(0.1)

            pubsub.publish("test/besteffort", "fast")
            assert event.wait(timeout=2.0)
            assert received == ["fast"]


class TestZenohPubSubErrors:
    """Tests for error handling."""

    def test_publish_without_start_raises(self) -> None:
        """Publishing without starting session should raise."""
        pubsub = ZenohPubSubBase()
        with pytest.raises(RuntimeError, match="session not started"):
            pubsub.publish("test", b"data")

    def test_subscribe_without_start_raises(self) -> None:
        """Subscribing without starting session should raise."""
        pubsub = ZenohPubSubBase()
        with pytest.raises(RuntimeError, match="session not started"):
            pubsub.subscribe("test", lambda msg, topic: None)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
