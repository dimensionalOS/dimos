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

"""E2E smoke tests for sim-nav — verifies the full sensor + control pipeline.

Starts dimos sim-nav headless (GPU rendering), then checks that all LCM
topics are publishing and that cmd_vel actually moves the robot.

    pytest dimos/e2e_tests/test_dimsim_nav.py -v -s
"""

import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import pytest

from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

BRIDGE_PORT = 8090


def _force_kill_port(port: int) -> None:
    """Kill any process listening on the given port."""
    try:
        result = subprocess.run(
            ["lsof", "-ti", f":{port}"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        pids = result.stdout.strip().split()
        for pid in pids:
            if pid:
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except (ProcessLookupError, ValueError):
                    pass
    except Exception:
        pass


def _wait_for_port(port: int, timeout: float = 120) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("localhost", port), timeout=2):
                return True
        except OSError:
            time.sleep(1)
    return False


def _wait_for_port_free(port: int, timeout: float = 30) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("localhost", port), timeout=1):
                time.sleep(1)
        except OSError:
            return True
    return False


@pytest.fixture(scope="class")
def sim_nav():
    """Start dimos sim-nav headless with GPU rendering, tear down after."""
    # Kill any leftover processes from a previous crashed run
    _force_kill_port(BRIDGE_PORT)
    assert _wait_for_port_free(BRIDGE_PORT, timeout=10), (
        f"Port {BRIDGE_PORT} still in use after force-kill"
    )

    log_dir = os.environ.get("DIMSIM_EVAL_LOG_DIR", "")
    if log_dir:
        log_path = Path(log_dir)
        log_path.mkdir(parents=True, exist_ok=True)
        log_file = open(log_path / "dimos-sim-nav.log", "w")
        print(f"\n  dimos logs → {log_path}/dimos-sim-nav.log")
    else:
        log_file = None

    render = os.environ.get("DIMSIM_RENDER", "gpu")
    venv_bin = str(Path(sys.prefix) / "bin")
    env = {
        **os.environ,
        "DIMSIM_HEADLESS": "1",
        "DIMSIM_RENDER": render,
        "PATH": venv_bin + os.pathsep + os.environ.get("PATH", ""),
    }
    call = DimosCliCall()
    call.demo_args = ["sim-nav"]
    call.process = subprocess.Popen(
        ["dimos", "--simulation", "run", "sim-nav"],
        env=env,
        stdout=log_file or subprocess.DEVNULL,
        stderr=log_file or subprocess.DEVNULL,
        start_new_session=True,
    )

    try:
        assert _wait_for_port(BRIDGE_PORT, timeout=120), f"Bridge not ready on port {BRIDGE_PORT}"
        yield call
    finally:
        call.stop()
        if log_file:
            log_file.close()
        # Ensure port is freed even if stop() doesn't fully clean up
        _force_kill_port(BRIDGE_PORT)


@pytest.fixture(scope="class")
def spy(sim_nav):
    """LCM spy that subscribes to all topics and waits for initial sensor data."""
    s = LcmSpy()
    s.save_topic("/color_image#sensor_msgs.Image")
    s.save_topic("/depth_image#sensor_msgs.Image")
    s.save_topic("/odom#geometry_msgs.PoseStamped")
    s.save_topic("/lidar#sensor_msgs.PointCloud2")
    s.start()

    # Wait for at least one message on color_image before tests start
    s.wait_for_saved_topic("/color_image#sensor_msgs.Image", timeout=60.0)

    yield s

    s.stop()


# Runs in CI: spawns dimos sim-nav headless, verifies the sensor + control pipeline.
# Requires multicast routing, network egress (dimsim binary download), and headless Chrome.
class TestSimNav:
    """Smoke tests for the sim-nav pipeline — sensors, control, and data integrity."""

    def test_color_image_publishing(self, spy: LcmSpy) -> None:
        """Verify /color_image is publishing and decodable."""
        spy.wait_for_saved_topic("/color_image#sensor_msgs.Image", timeout=30)
        msgs = spy.messages.get("/color_image#sensor_msgs.Image", [])
        assert len(msgs) > 0, "No color_image messages received"

        # Decode the latest image — DimSim sends JPEG encoding
        img = Image.lcm_jpeg_decode(msgs[-1])
        assert img.width > 0, f"Image width is {img.width}"
        assert img.height > 0, f"Image height is {img.height}"
        print(f"  color_image: {img.width}x{img.height}, {len(msgs)} messages")

    def test_depth_image_publishing(self, spy: LcmSpy) -> None:
        """Verify /depth_image is publishing and decodable."""
        spy.wait_for_saved_topic("/depth_image#sensor_msgs.Image", timeout=30)
        msgs = spy.messages.get("/depth_image#sensor_msgs.Image", [])
        assert len(msgs) > 0, "No depth_image messages received"

        # Depth images use raw encoding (16UC1), not JPEG
        img = Image.lcm_decode(msgs[-1])
        assert img.width > 0, f"Depth image width is {img.width}"
        assert img.height > 0, f"Depth image height is {img.height}"
        print(f"  depth_image: {img.width}x{img.height}, {len(msgs)} messages")

    def test_odom_publishing(self, spy: LcmSpy) -> None:
        """Verify /odom is publishing with sane values."""
        spy.wait_for_saved_topic("/odom#geometry_msgs.PoseStamped", timeout=30)
        msgs = spy.messages.get("/odom#geometry_msgs.PoseStamped", [])
        assert len(msgs) > 0, "No odom messages received"

        pose = PoseStamped.lcm_decode(msgs[-1])
        pos = pose.position
        # Position should be finite (not NaN or inf)
        assert math.isfinite(pos.x), f"odom x is {pos.x}"
        assert math.isfinite(pos.y), f"odom y is {pos.y}"
        assert math.isfinite(pos.z), f"odom z is {pos.z}"
        # Position should be within reasonable bounds (scene is ~50m)
        assert abs(pos.x) < 100, f"odom x out of bounds: {pos.x}"
        assert abs(pos.y) < 100, f"odom y out of bounds: {pos.y}"
        assert abs(pos.z) < 100, f"odom z out of bounds: {pos.z}"
        print(f"  odom: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), {len(msgs)} messages")

    def test_lidar_publishing(self, spy: LcmSpy) -> None:
        """Verify /lidar is publishing with non-empty point cloud."""
        spy.wait_for_saved_topic("/lidar#sensor_msgs.PointCloud2", timeout=30)
        msgs = spy.messages.get("/lidar#sensor_msgs.PointCloud2", [])
        assert len(msgs) > 0, "No lidar messages received"

        cloud = PointCloud2.lcm_decode(msgs[-1])
        num_points = len(cloud)
        assert num_points > 0, "Lidar point cloud is empty"
        print(f"  lidar: {num_points} points, {len(msgs)} messages")

    def test_cmd_vel_moves_robot(self, spy: LcmSpy) -> None:
        """Publish cmd_vel and verify odom position changes."""
        # Record initial position
        spy.wait_for_saved_topic("/odom#geometry_msgs.PoseStamped", timeout=30)
        initial_msgs = spy.messages.get("/odom#geometry_msgs.PoseStamped", [])
        assert len(initial_msgs) > 0, "No initial odom"
        initial_pose = PoseStamped.lcm_decode(initial_msgs[-1])
        initial_pos = initial_pose.position
        print(
            f"  initial position: ({initial_pos.x:.2f}, {initial_pos.y:.2f}, {initial_pos.z:.2f})"
        )

        # Send forward velocity for 3 seconds
        twist = Twist(
            linear=Vector3(0.5, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.0),
        )
        deadline = time.time() + 3.0
        while time.time() < deadline:
            spy.publish("/cmd_vel#geometry_msgs.Twist", twist)
            time.sleep(0.05)  # 20 Hz

        # Wait a bit for physics to settle and odom to update
        time.sleep(1.0)

        # Check position changed
        final_msgs = spy.messages.get("/odom#geometry_msgs.PoseStamped", [])
        final_pose = PoseStamped.lcm_decode(final_msgs[-1])
        final_pos = final_pose.position

        dx = final_pos.x - initial_pos.x
        dy = final_pos.y - initial_pos.y
        distance = math.sqrt(dx * dx + dy * dy)
        print(f"  final position: ({final_pos.x:.2f}, {final_pos.y:.2f}, {final_pos.z:.2f})")
        print(f"  distance moved: {distance:.2f}m")

        assert distance > 0.1, (
            f"Robot didn't move after cmd_vel: distance={distance:.3f}m "
            f"(initial=({initial_pos.x:.2f},{initial_pos.y:.2f}), "
            f"final=({final_pos.x:.2f},{final_pos.y:.2f}))"
        )

    def test_odom_rate(self, spy: LcmSpy) -> None:
        """Verify odom publishes at a reasonable rate (>5 Hz)."""
        # Clear existing messages and collect fresh ones
        topic = "/odom#geometry_msgs.PoseStamped"
        with spy._messages_lock:
            spy.messages[topic] = []

        time.sleep(2.0)

        with spy._messages_lock:
            count = len(spy.messages.get(topic, []))

        rate = count / 2.0
        print(f"  odom rate: {rate:.1f} Hz ({count} messages in 2s)")
        assert rate > 5, f"Odom rate too low: {rate:.1f} Hz (expected >5 Hz)"
