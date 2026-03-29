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

"""Tests for incremental module deployment — no full blueprint required.

Tests the pattern: start one module → use it → add another module later.
This is a key use case from issue #1636: users should be able to start
a single module and incrementally compose their robot.

Run: CI=1 python -m pytest dimos/core/test_api_incremental.py -v
"""

from __future__ import annotations

import threading
import time

import pytest

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import GlobalConfig
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.tests.stream_test_module import StreamTestModule
from dimos.core.tests.stress_test_module import StressTestModule


class TestSingleModuleNoBlueprintInit:
    """Start a single module without blueprint.build()."""

    def test_manual_coordinator_rpc_only(self):
        """Create coordinator manually, deploy one module, use RPC."""
        coord = ModuleCoordinator()
        coord.start()
        try:
            # Deploy just the StreamTestModule — RPC works, streams don't
            # (no transport wired without blueprint)
            proxy = coord.deploy(StreamTestModule, GlobalConfig())
            proxy.start()
            time.sleep(1)

            # RPC works without blueprint
            count = proxy.get_publish_count()
            assert count >= 0
            pos = proxy.get_position()
            assert "x" in pos

            # Skills work
            result = proxy.relative_move(forward=1.0)
            assert "Moved" in result
            pos2 = proxy.get_position()
            assert pos2["x"] > pos["x"]
        finally:
            coord.stop()
            time.sleep(0.5)

    def test_single_module_with_autoconnect(self):
        """Use autoconnect() for minimal blueprint — streams work."""
        bp = autoconnect(StreamTestModule.blueprint())
        coord = bp.build()
        try:
            proxy = coord.module("StreamTestModule")
            # Both RPC and streams work
            pos = proxy.get_position()
            assert "x" in pos

            frames = []
            e = threading.Event()
            unsub = proxy.odom.subscribe(
                lambda p: (frames.append(p), e.set() if len(frames) >= 3 else None)
            )
            assert e.wait(timeout=5), "No odom from single module"
            unsub()
            assert len(frames) >= 3
        finally:
            coord.stop()
            time.sleep(0.5)

    def test_pull_frame_from_single_module(self):
        """Start one module via autoconnect, pull a camera frame."""
        bp = autoconnect(StreamTestModule.blueprint())
        coord = bp.build()
        try:
            proxy = coord.module("StreamTestModule")
            time.sleep(0.5)

            frame = [None]
            e = threading.Event()
            unsub = proxy.color_image.subscribe(lambda img: (frame.__setitem__(0, img), e.set()))
            assert e.wait(timeout=5), "No camera frame"
            unsub()
            assert frame[0] is not None
            assert frame[0].data.shape == (8, 8, 3)
        finally:
            coord.stop()
            time.sleep(0.5)


class TestIncrementalDeploy:
    """Start with one module, add another later."""

    def test_add_module_after_first_is_running(self):
        """Deploy module A, use it, then deploy module B."""
        coord = ModuleCoordinator()
        coord.start()
        try:
            # Phase 1: Just StreamTestModule
            stream_proxy = coord.deploy(StreamTestModule, GlobalConfig())
            stream_proxy.start()
            time.sleep(0.5)

            # Use it
            pos = stream_proxy.get_position()
            assert "x" in pos

            # Phase 2: Add StressTestModule
            stress_proxy = coord.deploy(StressTestModule, GlobalConfig())
            stress_proxy.start()
            time.sleep(0.5)

            # Both work simultaneously
            assert stress_proxy.ping() == "pong"
            stream_proxy.relative_move(forward=1.0)
            pos2 = stream_proxy.get_position()
            assert pos2["x"] > pos["x"]
        finally:
            coord.stop()
            time.sleep(0.5)

    def test_add_second_blueprint_after_first(self):
        """Build two separate blueprints into the same coordinator.

        NOTE: Currently blueprints create their own coordinators.
        This tests the pattern of using two separate coordinators.
        """
        # First robot with streaming
        bp1 = autoconnect(StreamTestModule.blueprint())
        coord1 = bp1.build()
        try:
            proxy1 = coord1.module("StreamTestModule")

            # Start streaming
            frames = []
            unsub = proxy1.odom.subscribe(lambda p: frames.append(p))
            time.sleep(1)
            assert len(frames) > 0, "Stream should be active"

            # Can use a second coordinator simultaneously
            bp2 = autoconnect(StressTestModule.blueprint())
            coord2 = bp2.build()
            try:
                proxy2 = coord2.module("StressTestModule")
                assert proxy2.ping() == "pong"

                # First coordinator still streaming
                count_before = len(frames)
                time.sleep(0.5)
                assert len(frames) > count_before
            finally:
                coord2.stop()
                time.sleep(0.5)

            unsub()
        finally:
            coord1.stop()
            time.sleep(0.5)


class TestModuleWithoutCoordinator:
    """Can a module run completely standalone?"""

    def test_module_direct_instantiation(self):
        """Instantiate a module directly (no worker process)."""
        # This tests the raw module class — no RPC, no workers
        mod = StreamTestModule()
        # Module exists but isn't started (no process boundary)
        assert hasattr(mod, "odom")
        assert hasattr(mod, "color_image")
        assert hasattr(mod, "lidar")
        # Can call methods directly (no RPC)
        assert mod._publish_count == 0
        assert mod._x == 0.0


class TestCoordinatorModuleLookup:
    """Test module() accessor works with incremental deploys."""

    def test_module_names_updates_incrementally(self):
        """module_names should grow as modules are deployed."""
        coord = ModuleCoordinator()
        coord.start()
        try:
            assert len(coord.module_names) == 0

            stream_proxy = coord.deploy(StreamTestModule, GlobalConfig())
            stream_proxy.start()
            assert "StreamTestModule" in coord.module_names
            assert len(coord.module_names) == 1

            stress_proxy = coord.deploy(StressTestModule, GlobalConfig())
            stress_proxy.start()
            assert "StressTestModule" in coord.module_names
            assert len(coord.module_names) == 2
        finally:
            coord.stop()
            time.sleep(0.5)

    def test_module_accessor_with_manual_deploy(self):
        """robot.module('Name') works for manually deployed modules."""
        coord = ModuleCoordinator()
        coord.start()
        try:
            proxy = coord.deploy(StreamTestModule, GlobalConfig())
            proxy.start()
            time.sleep(0.5)

            # Access via name
            same_proxy = coord.module("StreamTestModule")
            assert same_proxy is not None
            pos = same_proxy.get_position()
            assert "x" in pos
        finally:
            coord.stop()
            time.sleep(0.5)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
