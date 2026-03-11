# Copyright 2026 Dimensional Inc.
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

"""Tests for RerunBridgeModule rate limiting."""

from __future__ import annotations

import time
from unittest.mock import MagicMock, patch

import pytest


class TestBridgeRateLimiter:
    """Verify per-entity-path rate limiting in _on_message."""

    @pytest.fixture()
    def bridge(self):
        """Create a RerunBridgeModule with rate limiting, no actual viewer."""
        from dimos.visualization.rerun.bridge import RerunBridgeModule

        b = RerunBridgeModule(min_interval_sec=0.1)
        yield b
        b.stop()

    @pytest.fixture()
    def mock_rerun(self):
        """Patch rerun.log so we can count calls without a viewer."""
        with patch("rerun.log") as mock_log, patch("rerun.init"):
            yield mock_log

    def _make_msg(self):
        """Create a minimal RerunConvertible message."""
        from dimos.visualization.rerun.bridge import RerunConvertible

        msg = MagicMock(spec=RerunConvertible)
        archetype = MagicMock()
        archetype.__class__.__name__ = "MockArchetype"
        msg.to_rerun.return_value = archetype
        return msg

    def test_first_message_always_passes(self, bridge, mock_rerun):
        """First message on any entity path should always be logged."""
        msg = self._make_msg()
        topic = MagicMock()
        topic.name = "/color_image#sensor_msgs.Image"

        bridge._on_message(msg, topic)

        assert mock_rerun.call_count == 1

    def test_rapid_messages_are_dropped(self, bridge, mock_rerun):
        """Messages faster than min_interval_sec should be dropped."""
        msg = self._make_msg()
        topic = MagicMock()
        topic.name = "/color_image#sensor_msgs.Image"

        for _ in range(30):
            bridge._on_message(msg, topic)

        # Only the first should pass (all others within 0.1s window)
        assert mock_rerun.call_count == 1

    def test_messages_pass_after_interval(self, bridge, mock_rerun):
        """Messages should pass again after min_interval_sec elapses."""
        msg = self._make_msg()
        topic = MagicMock()
        topic.name = "/color_image#sensor_msgs.Image"

        bridge._on_message(msg, topic)
        assert mock_rerun.call_count == 1

        time.sleep(0.15)

        bridge._on_message(msg, topic)
        assert mock_rerun.call_count == 2

    def test_different_topics_have_independent_limits(self, bridge, mock_rerun):
        """Each entity path should have its own rate limit."""
        msg = self._make_msg()
        topic_a = MagicMock()
        topic_a.name = "/color_image#sensor_msgs.Image"
        topic_b = MagicMock()
        topic_b.name = "/depth_image#sensor_msgs.Image"

        bridge._on_message(msg, topic_a)
        bridge._on_message(msg, topic_b)

        assert mock_rerun.call_count == 2

    def test_rate_limiting_disabled_when_zero(self, mock_rerun):
        """Setting min_interval_sec=0 should disable rate limiting."""
        from dimos.visualization.rerun.bridge import RerunBridgeModule

        bridge = RerunBridgeModule(min_interval_sec=0.0)

        msg = self._make_msg()
        topic = MagicMock()
        topic.name = "/color_image#sensor_msgs.Image"

        for _ in range(30):
            bridge._on_message(msg, topic)

        bridge.stop()
        assert mock_rerun.call_count == 30

    def test_effective_rate_over_time(self, bridge, mock_rerun):
        """Simulate 1 second of 30fps and verify ~10 messages get through."""
        msg = self._make_msg()
        topic = MagicMock()
        topic.name = "/color_image#sensor_msgs.Image"

        start = time.monotonic()

        while time.monotonic() - start < 1.0:
            bridge._on_message(msg, topic)
            time.sleep(1.0 / 30)

        logged = mock_rerun.call_count

        # With 0.1s interval, expect ~10 messages in 1 second
        assert 8 <= logged <= 12, f"Expected ~10 messages in 1s at 10Hz, got {logged}"
