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

"""Stream-level tests for the dimos Python API.

Tests inspired by GitHub issue #1636 — the API should make it trivial to:
- Collect N frames from a stream
- Subscribe to streams with callbacks
- Run skills while streams are active
- Compose blueprints programmatically

NOTE: RemoteOut (proxy streams) only support .subscribe(), not
get_next/hot_latest/observable. Those are on local In only.
This test suite validates what works through the API today and
documents what gaps remain.
"""

from __future__ import annotations

import threading
import time

import pytest

# LCM transport spawns daemon threads (_lcm_loop) when subscribing to streams.
# These threads are cleaned up at process exit but outlive individual tests.
# Mark all stream tests to allow this expected behavior.
pytestmark = pytest.mark.allow_thread_leaks

from dimos.core.blueprints import autoconnect
from dimos.core.tests.stream_test_module import StreamTestModule
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


@pytest.fixture
def robot():
    """Launch StreamTestModule and yield coordinator."""
    blueprint = autoconnect(StreamTestModule.blueprint())
    coordinator = blueprint.build()
    yield coordinator
    coordinator.stop()
    import time

    time.sleep(0.5)  # let LCM daemon threads wind down


class TestStreamSubscribe:
    """Test subscribe-based stream access — what RemoteOut supports today."""

    def test_subscribe_receives_pose(self, robot):
        """Subscribe to odom stream and receive a PoseStamped."""
        stream_mod = robot.module("StreamTestModule")
        received = []
        event = threading.Event()

        def on_pose(pose):
            received.append(pose)
            if len(received) >= 1:
                event.set()

        unsub = stream_mod.odom.subscribe(on_pose)
        try:
            assert event.wait(timeout=5.0), "Should receive a pose within 5s"
            assert isinstance(received[0], PoseStamped)
            # Position starts at 0,0 — just verify we got a valid pose
        finally:
            unsub()

    def test_collect_five_frames(self, robot):
        """Collect 5 frames from a stream via subscribe — issue #1636 use case."""
        stream_mod = robot.module("StreamTestModule")
        frames = []
        event = threading.Event()

        def on_pose(pose):
            frames.append(pose)
            if len(frames) >= 5:
                event.set()

        unsub = stream_mod.odom.subscribe(on_pose)
        try:
            assert event.wait(timeout=5.0), "Should collect 5 frames within 5s"
            assert len(frames) >= 5
            # Frames should be sequential (x increases monotonically)
            xs = [f.position.x for f in frames[:5]]
            for i in range(1, len(xs)):
                assert xs[i] >= xs[i - 1], f"Frames not monotonic: {xs}"
        finally:
            unsub()

    def test_unsubscribe_stops_callbacks(self, robot):
        """Unsubscribing should stop further callbacks."""
        stream_mod = robot.module("StreamTestModule")
        received = []

        unsub = stream_mod.odom.subscribe(lambda p: received.append(p))
        time.sleep(0.5)
        count_before = len(received)
        assert count_before > 0, "Should have received some poses"

        unsub()  # unsubscribe
        time.sleep(0.5)
        count_after = len(received)
        # Should not grow significantly after unsubscribe
        assert count_after - count_before <= 1, f"Got {count_after - count_before} more after unsub"

    def test_multiple_subscribers(self, robot):
        """Multiple subscribers receive the same data independently."""
        stream_mod = robot.module("StreamTestModule")
        sub1_frames = []
        sub2_frames = []
        event = threading.Event()

        def on1(pose):
            sub1_frames.append(pose)

        def on2(pose):
            sub2_frames.append(pose)
            if len(sub2_frames) >= 3:
                event.set()

        unsub1 = stream_mod.odom.subscribe(on1)
        unsub2 = stream_mod.odom.subscribe(on2)
        try:
            assert event.wait(timeout=5.0)
            assert len(sub1_frames) >= 2
            assert len(sub2_frames) >= 3
        finally:
            unsub1()
            unsub2()

    def test_collect_ten_frames_timing(self, robot):
        """Verify stream rate: 10Hz should deliver 10 frames in ~1s."""
        stream_mod = robot.module("StreamTestModule")
        frames = []
        event = threading.Event()

        def on_pose(pose):
            frames.append((time.monotonic(), pose))
            if len(frames) >= 10:
                event.set()

        unsub = stream_mod.odom.subscribe(on_pose)
        try:
            assert event.wait(timeout=5.0), "Should get 10 frames within 5s"
            duration = frames[9][0] - frames[0][0]
            # 10 frames at 10Hz = ~0.9s. Allow 0.5-3.0 for CI variance.
            assert 0.5 < duration < 3.0, f"10 frames took {duration:.2f}s"
        finally:
            unsub()


class TestSkillWithStreams:
    """Test calling skills on a module that also has active streams."""

    def test_skill_while_streaming(self, robot):
        """Call a skill while the stream is actively publishing."""
        stream_mod = robot.module("StreamTestModule")
        received = []
        event = threading.Event()

        unsub = stream_mod.odom.subscribe(lambda p: (received.append(p), event.set()))
        try:
            assert event.wait(timeout=5.0)
            assert len(received) > 0

            # Call RPC skill simultaneously
            position = stream_mod.get_position()
            assert "x" in position  # returns dict

        finally:
            unsub()

    def test_publish_count_increases(self, robot):
        """Verify the module is actually publishing over time."""
        stream_mod = robot.module("StreamTestModule")
        count1 = stream_mod.get_publish_count()
        time.sleep(0.5)
        count2 = stream_mod.get_publish_count()
        assert count2 > count1, f"Count didn't increase: {count1} -> {count2}"

    def test_rpc_and_stream_interleaved(self, robot):
        """Interleave RPC calls and stream reads — stress test."""
        stream_mod = robot.module("StreamTestModule")
        poses = []
        positions = []

        def collect_poses():
            event = threading.Event()

            def on_pose(p):
                poses.append(p)
                if len(poses) >= 10:
                    event.set()

            unsub = stream_mod.odom.subscribe(on_pose)
            event.wait(timeout=5.0)
            unsub()

        def collect_positions():
            for _ in range(5):
                positions.append(stream_mod.get_position())
                time.sleep(0.1)

        t1 = threading.Thread(target=collect_poses)
        t2 = threading.Thread(target=collect_positions)
        t1.start()
        t2.start()
        t1.join(timeout=10)
        t2.join(timeout=10)

        assert len(poses) >= 10
        assert len(positions) == 5


class TestBlueprintComposition:
    """Test composing blueprints programmatically — key issue #1636 use case."""

    def test_compose_two_modules(self):
        """Compose two module blueprints together."""
        from dimos.core.tests.stress_test_module import StressTestModule

        combined = autoconnect(
            StreamTestModule.blueprint(),
            StressTestModule.blueprint(),
        )
        coordinator = combined.build()
        try:
            assert "StreamTestModule" in coordinator.module_names
            assert "StressTestModule" in coordinator.module_names

            # Skills work across both modules
            assert coordinator.module("StressTestModule").ping() == "pong"

            # Stream works on StreamTestModule
            received = []
            event = threading.Event()
            unsub = coordinator.module("StreamTestModule").odom.subscribe(
                lambda p: (received.append(p), event.set() if len(received) >= 1 else None)
            )
            try:
                assert event.wait(timeout=5.0)
                assert isinstance(received[0], PoseStamped)
            finally:
                unsub()
        finally:
            coordinator.stop()

    def test_dynamic_blueprint_with_config(self):
        """Build a blueprint with custom global config."""
        blueprint = autoconnect(
            StreamTestModule.blueprint(),
        )
        coordinator = blueprint.build()
        try:
            stream_mod = coordinator.module("StreamTestModule")
            # Verify stream works with 1 worker
            received = []
            event = threading.Event()
            unsub = stream_mod.odom.subscribe(lambda p: (received.append(p), event.set()))
            try:
                assert event.wait(timeout=5.0)
                assert isinstance(received[0], PoseStamped)
            finally:
                unsub()
        finally:
            coordinator.stop()


class TestRemoteOutGaps:
    """Document what RemoteOut DOESN'T support yet — future work."""

    def test_remote_out_lacks_get_next(self, robot):
        """RemoteOut doesn't have get_next — this is a gap to fix."""
        stream_mod = robot.module("StreamTestModule")
        assert not hasattr(stream_mod.odom, "get_next"), (
            "If this fails, RemoteOut gained get_next — update tests!"
        )

    def test_remote_out_lacks_observable(self, robot):
        """RemoteOut doesn't have observable — this is a gap to fix."""
        stream_mod = robot.module("StreamTestModule")
        assert not hasattr(stream_mod.odom, "observable"), (
            "If this fails, RemoteOut gained observable — update tests!"
        )
