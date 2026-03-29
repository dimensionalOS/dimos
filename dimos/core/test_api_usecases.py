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

"""Python API E2E use case tests — all 10 examples from issue #1636.

Uses StreamTestModule (synthetic sensor data, no hardware needed).
Each test class = one use case from the examples list.

Run: CI=1 python -m pytest dimos/core/test_api_usecases.py -v -x
"""

from __future__ import annotations

import csv
import io
import math
import threading
import time

import numpy as np
import pytest

from dimos.core.blueprints import autoconnect
from dimos.core.tests.stream_test_module import StreamTestModule


@pytest.fixture(scope="module")
def robot():
    """Launch StreamTestModule, shared across all tests."""
    blueprint = autoconnect(StreamTestModule.blueprint())
    coordinator = blueprint.build()
    time.sleep(1)  # let streams start
    yield coordinator
    coordinator.stop()
    time.sleep(0.5)


# ─── Use Case 1: Stand Up and Walk Forward ─────────────────────────
class TestUseCase01_Walk:
    """Stand up, walk 2m forward, verify odom moved in +x."""

    def test_walk_forward(self, robot):
        mod = robot.module("StreamTestModule")

        pos0 = mod.get_position()
        mod.stand_up()
        mod.relative_move(forward=2.0)

        pos1 = mod.get_position()
        dx = pos1["x"] - pos0["x"]
        assert abs(dx - 2.0) < 0.1, f"Expected ~2m forward, got {dx:.3f}m"

        # Verify odom stream agrees
        odom = [None]
        e = threading.Event()
        unsub = mod.odom.subscribe(lambda p: (odom.__setitem__(0, p), e.set()))
        e.wait(timeout=5)
        unsub()
        assert odom[0].position.x > 1.5

        mod.stand_down()


# ─── Use Case 2: Take a Photo ──────────────────────────────────────
class TestUseCase02_Photo:
    """Capture a single frame from the camera stream."""

    def test_capture_frame(self, robot):
        mod = robot.module("StreamTestModule")
        frame = [None]
        e = threading.Event()
        unsub = mod.color_image.subscribe(
            lambda img, f=frame, ev=e: (f.__setitem__(0, img), ev.set())
        )
        assert e.wait(timeout=5), "No camera frame received"
        unsub()

        img = frame[0]
        assert img is not None
        assert hasattr(img, "data")
        assert img.data.shape == (8, 8, 3)  # 8x8 RGB


# ─── Use Case 3: Obstacle Distance Check ───────────────────────────
class TestUseCase03_Obstacle:
    """Get point cloud, check front sector for nearest obstacle."""

    def test_obstacle_distance(self, robot):
        mod = robot.module("StreamTestModule")
        cloud = [None]
        e = threading.Event()
        unsub = mod.lidar.subscribe(lambda pc: (cloud.__setitem__(0, pc), e.set()))
        assert e.wait(timeout=5), "No LiDAR data received"
        unsub()

        pts = cloud[0]._pcd_tensor.point["positions"].numpy()
        assert pts.shape[0] > 0, "Empty point cloud"
        # Front sector: x > 0, |y| < 0.5
        front = pts[(pts[:, 0] > 0) & (np.abs(pts[:, 1]) < 0.5)]
        assert len(front) > 0, "No points in front sector"
        min_dist = np.min(front[:, 0])
        assert min_dist > 0, f"Obstacle at negative distance: {min_dist}"
        assert min_dist < 5.0, f"Obstacle too far: {min_dist}"


# ─── Use Case 4: Walk a Square ─────────────────────────────────────
class TestUseCase04_Square:
    """Walk a 1m square (4 forward + 4 turns), end near start."""

    def test_walk_square(self, robot):
        mod = robot.module("StreamTestModule")

        pos0 = mod.get_position()
        mod.stand_up()

        for _ in range(4):
            mod.relative_move(forward=1.0)
            mod.relative_move(yaw=90.0)

        pos1 = mod.get_position()
        # Should be back near start (within tolerance for floating point)
        dist = math.sqrt((pos1["x"] - pos0["x"]) ** 2 + (pos1["y"] - pos0["y"]) ** 2)
        assert dist < 0.5, f"Didn't return to start: dist={dist:.3f}m"
        mod.stand_down()


# ─── Use Case 5: Stream Odometry to CSV ────────────────────────────
class TestUseCase05_OdomCSV:
    """Record 2 seconds of odometry, write to CSV."""

    def test_odom_csv(self, robot):
        mod = robot.module("StreamTestModule")
        rows = []

        def on_odom(pose):
            rows.append([time.time(), pose.position.x, pose.position.y])

        unsub = mod.odom.subscribe(on_odom)
        time.sleep(2)
        unsub()

        assert len(rows) >= 10, f"Expected ≥10 readings in 2s, got {len(rows)}"

        buf = io.StringIO()
        w = csv.writer(buf)
        w.writerow(["time", "x", "y"])
        w.writerows(rows)
        content = buf.getvalue()
        assert "time,x,y" in content
        lines = content.strip().split("\n")
        assert len(lines) >= 11  # header + 10+ data rows


# ─── Use Case 6: Person Follow (Skill Invocation) ──────────────────
class TestUseCase06_SkillInvocation:
    """Invoke skills programmatically — the person-follow pattern."""

    def test_skill_chain(self, robot):
        mod = robot.module("StreamTestModule")

        # Chain: stand up → move toward "person" → get position
        mod.stand_up()
        result = mod.relative_move(forward=1.5)
        assert "1.50" in result or "Moved" in result

        pos = mod.get_position_str()
        assert "x=" in pos


# ─── Use Case 7: 360° Photo Shoot ──────────────────────────────────
class TestUseCase07_OrbitPhotos:
    """Take 8 photos while rotating 360°."""

    def test_orbit_capture(self, robot):
        mod = robot.module("StreamTestModule")
        photos = []

        mod.stand_up()
        for _i in range(8):
            # Capture frame
            frame = [None]
            e = threading.Event()
            unsub = mod.color_image.subscribe(
                lambda img, f=frame, ev=e: (f.__setitem__(0, img), ev.set())
            )
            e.wait(timeout=5)
            unsub()
            photos.append(frame[0])

            # Rotate 45°
            mod.relative_move(yaw=45.0)

        assert len(photos) == 8
        assert all(p is not None for p in photos)
        # Verify we rotated ~360°
        pos = mod.get_position()
        # yaw should be near original (360° = 0 mod 2π)
        yaw_mod = pos["yaw"] % (2 * math.pi)
        assert yaw_mod < 0.5 or yaw_mod > (2 * math.pi - 0.5), (
            f"Expected ~0 or ~2π yaw after 360°, got {yaw_mod:.3f}"
        )


# ─── Use Case 8: Security Patrol ───────────────────────────────────
class TestUseCase08_Patrol:
    """Navigate between 4 waypoints, verify reaching each."""

    def test_patrol_waypoints(self, robot):
        mod = robot.module("StreamTestModule")

        mod.stand_up()
        visited = []
        # Walk a square: forward, left, forward, left, forward, left, forward
        for _i in range(4):
            mod.relative_move(forward=1.0)
            visited.append(mod.get_position())
            mod.relative_move(yaw=90.0)

        assert len(visited) == 4
        # Each waypoint should be at a different position
        positions = [(v["x"], v["y"]) for v in visited]
        # At least moved away from origin
        assert any(abs(x) > 0.5 or abs(y) > 0.5 for x, y in positions)


# ─── Use Case 9: Discover Skills ───────────────────────────────────
class TestUseCase09_Discover:
    """List all modules and their skills."""

    def test_list_modules(self, robot):
        names = robot.module_names
        assert "StreamTestModule" in names
        assert len(names) >= 1

    def test_list_skills(self, robot):
        mod = robot.module("StreamTestModule")
        skills = mod.get_skills()
        skill_names = [s.func_name for s in skills]
        assert "relative_move" in skill_names
        assert "stand_up" in skill_names
        assert "get_position_str" in skill_names

    def test_skill_args_schema(self, robot):
        mod = robot.module("StreamTestModule")
        skills = mod.get_skills()
        move_skill = next(s for s in skills if s.func_name == "relative_move")
        assert "forward" in str(move_skill.args_schema)


# ─── Use Case 10: Compose Blueprint ────────────────────────────────
class TestUseCase10_Compose:
    """Compose two modules into one blueprint."""

    def test_compose_stream_and_stress(self):
        from dimos.core.tests.stress_test_module import StressTestModule

        combined = autoconnect(
            StreamTestModule.blueprint(),
            StressTestModule.blueprint(),
        )
        coord = combined.build()
        try:
            assert "StreamTestModule" in coord.module_names
            assert "StressTestModule" in coord.module_names

            # Both work simultaneously
            assert coord.module("StressTestModule").ping() == "pong"

            frames = []
            e = threading.Event()
            unsub = coord.module("StreamTestModule").odom.subscribe(
                lambda p: (frames.append(p), e.set() if len(frames) >= 3 else None)
            )
            assert e.wait(timeout=5)
            unsub()
            assert len(frames) >= 3
        finally:
            coord.stop()
            time.sleep(0.5)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-x", "--tb=short"])
