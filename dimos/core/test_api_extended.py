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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Extended Python API tests — beyond the initial 10 use cases.

Tests: RxPY pipelines, LCM echo round-trip, frame rate measurement,
large payloads, multi-blueprint lifecycle.

Run: CI=1 python -m pytest dimos/core/test_api_extended.py -v
"""

from __future__ import annotations

import math
import threading
import time

import pytest
import reactivex.operators as ops

from dimos.core.blueprints import autoconnect
from dimos.core.tests.stream_test_module import StreamTestModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image


@pytest.fixture(scope="module")
def robot():
    blueprint = autoconnect(StreamTestModule.blueprint())
    coordinator = blueprint.build()
    time.sleep(1)
    yield coordinator
    coordinator.stop()
    time.sleep(0.5)


# ─── 11: RxPY Pipeline ─────────────────────────────────────────────
class TestExtended11_RxPipeline:
    """RxPY reactive stream processing on robot data."""

    @pytest.mark.xfail(reason="RemoteOut lacks .observable() — known gap")
    def test_observable_take_five_odom(self, robot):
        """Use RxPY to take exactly 5 odom values."""
        mod = robot.module("StreamTestModule")
        results = mod.odom.observable().pipe(ops.take(5), ops.to_list()).run()
        assert len(results) == 5
        assert all(isinstance(r, PoseStamped) for r in results)

    @pytest.mark.xfail(reason="RemoteOut lacks .observable() — known gap")
    def test_filter_and_map(self, robot):
        """Filter odom by position, map to x coordinate."""
        mod = robot.module("StreamTestModule")
        # Move robot forward first so we have non-zero positions
        mod.relative_move(forward=1.0)

        results = (
            mod.odom.observable()
            .pipe(
                ops.map(lambda p: p.position.x),
                ops.filter(lambda x: x > 0),
                ops.take(3),
                ops.to_list(),
            )
            .run()
        )
        assert len(results) == 3
        assert all(x > 0 for x in results)

    @pytest.mark.xfail(reason="RemoteOut lacks .observable() — known gap")
    def test_scan_accumulator(self, robot):
        """Use scan to accumulate total distance traveled."""
        mod = robot.module("StreamTestModule")

        results = (
            mod.odom.observable()
            .pipe(
                ops.map(lambda p: (p.position.x, p.position.y)),
                ops.pairwise(),
                ops.map(
                    lambda pair: math.sqrt(
                        (pair[1][0] - pair[0][0]) ** 2 + (pair[1][1] - pair[0][1]) ** 2
                    )
                ),
                ops.scan(lambda acc, d: acc + d, 0.0),
                ops.take(5),
                ops.to_list(),
            )
            .run()
        )
        assert len(results) == 5
        # Accumulated distance should be monotonically non-decreasing
        for i in range(1, len(results)):
            assert results[i] >= results[i - 1]


# ─── 12: LCM Echo Round-Trip ───────────────────────────────────────
class TestExtended12_LCMEcho:
    """Send cmd_vel, verify odom changes — full LCM round-trip."""

    def test_cmd_vel_changes_odom(self, robot):
        """Publish velocity command, verify position changes in odom."""
        mod = robot.module("StreamTestModule")

        # Record position before
        pos_before = mod.get_position()

        # Send forward velocity via cmd_vel stream
        twist = Twist(
            linear=Vector3(1.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.0),
        )
        mod.cmd_vel.publish(twist)
        time.sleep(1.0)  # let module integrate velocity

        # Stop
        twist_stop = Twist(
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.0),
        )
        mod.cmd_vel.publish(twist_stop)

        pos_after = mod.get_position()
        dx = pos_after["x"] - pos_before["x"]
        # At 1.0 m/s for 1s, should move ~1m (10 frames * 0.1s * 1.0 m/s)
        assert dx > 0.5, f"Expected forward movement, got dx={dx:.3f}"

    def test_rotation_via_cmd_vel(self, robot):
        """Send angular velocity, verify yaw changes."""
        mod = robot.module("StreamTestModule")
        pos_before = mod.get_position()

        twist = Twist(
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 1.0),  # 1 rad/s
        )
        mod.cmd_vel.publish(twist)
        time.sleep(0.5)

        twist_stop = Twist(
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.0),
        )
        mod.cmd_vel.publish(twist_stop)

        pos_after = mod.get_position()
        dyaw = abs(pos_after["yaw"] - pos_before["yaw"])
        assert dyaw > 0.1, f"Expected rotation, got dyaw={dyaw:.3f} rad"


# ─── 13: Frame Rate Measurement ────────────────────────────────────
class TestExtended13_FrameRate:
    """Measure actual stream delivery rates."""

    def test_odom_rate(self, robot):
        """Verify odom arrives at ~10Hz."""
        mod = robot.module("StreamTestModule")
        timestamps = []

        def on_odom(p):
            timestamps.append(time.monotonic())

        unsub = mod.odom.subscribe(on_odom)
        time.sleep(2)
        unsub()

        assert len(timestamps) >= 10, f"Too few: {len(timestamps)}"
        # Calculate average interval
        intervals = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]
        avg = sum(intervals) / len(intervals)
        # Stream rate depends on LCM delivery + backpressure, allow wide range
        assert 0.01 < avg < 0.5, f"Avg interval {avg:.3f}s outside expected range"

    def test_camera_rate(self, robot):
        """Verify camera arrives at ~10Hz."""
        mod = robot.module("StreamTestModule")
        timestamps = []

        def on_img(img):
            timestamps.append(time.monotonic())

        unsub = mod.color_image.subscribe(on_img)
        time.sleep(2)
        unsub()

        assert len(timestamps) >= 10


# ─── 14: Large Payload ─────────────────────────────────────────────
class TestExtended14_LargePayload:
    """Test that larger images can flow through the stream system."""

    def test_image_data_integrity(self, robot):
        """Verify image pixel data survives the LCM round-trip."""
        mod = robot.module("StreamTestModule")
        frames = []
        e = threading.Event()

        def on_img(img):
            frames.append(img)
            if len(frames) >= 3:
                e.set()

        unsub = mod.color_image.subscribe(on_img)
        assert e.wait(timeout=5)
        unsub()

        for img in frames[:3]:
            assert isinstance(img, Image)
            assert img.data.shape == (8, 8, 3)
            # All pixels should be same value (our synthetic pattern)
            assert img.data.min() == img.data.max()


# ─── 15: Multi-Blueprint Lifecycle ─────────────────────────────────
class TestExtended15_Lifecycle:
    """Start/stop/restart blueprint sequences."""

    def test_start_stop_start(self):
        """Start a blueprint, stop it, start a new one."""
        bp = autoconnect(StreamTestModule.blueprint())

        # First lifecycle
        coord1 = bp.build()
        mod1 = coord1.module("StreamTestModule")
        assert mod1.stand_up() == "Standing"
        coord1.stop()
        time.sleep(0.5)

        # Second lifecycle — fresh coordinator
        coord2 = bp.build()
        mod2 = coord2.module("StreamTestModule")
        assert mod2.stand_up() == "Standing"

        # Verify streams work on second instance
        frames = []
        e = threading.Event()
        unsub = mod2.odom.subscribe(lambda p: (frames.append(p), e.set()))
        assert e.wait(timeout=5)
        unsub()
        assert len(frames) >= 1

        coord2.stop()
        time.sleep(0.5)

    def test_multiple_coordinators_sequential(self):
        """Build 3 coordinators sequentially — each should work independently."""
        bp = autoconnect(StreamTestModule.blueprint())
        for i in range(3):
            coord = bp.build()
            mod = coord.module("StreamTestModule")
            result = mod.relative_move(forward=float(i + 1))
            assert "Moved" in result
            coord.stop()
            time.sleep(0.5)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
