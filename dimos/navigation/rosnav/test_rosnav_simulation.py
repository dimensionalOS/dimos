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

"""
Integration test for the ROSNav Docker module.

Starts the navigation stack in simulation mode with Unity and verifies that
the ROS→DimOS bridge produces data on expected streams.  Requires an X11
display (real or virtual) for Unity to render.

Requires:
    - Docker with BuildKit
    - SSH key in agent for private repo clone (first build only)
    - ~17 GB disk for the Docker image

Run:
    pytest dimos/navigation/rosnav/test_rosnav_simulation.py -m slow -s
"""

import threading
import time

from dimos_lcm.std_msgs import Bool
import pytest

from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.rosnav.rosnav_module import ROSNav

# Streams that should produce data in simulation mode without sending a goal.
# The nav stack publishes these as soon as the Unity sim is running.
EXPECTED_STREAMS = {
    "odom",
    "lidar",
    "color_image",
    "cmd_vel",
    "path",
}

# Streams that only produce data after a navigation goal is sent,
# or take a long time to appear. We report but don't assert.
OPTIONAL_STREAMS = {
    "global_pointcloud",
    "goal_active",
    "goal_reached",
}

# Total timeout for waiting for expected streams.
STREAM_TIMEOUT_SEC = 360  # 6 minutes


class StreamCollector(Module):
    """Test module that subscribes to all ROSNav output streams and records arrivals."""

    color_image: In[Image]
    lidar: In[PointCloud2]
    global_pointcloud: In[PointCloud2]
    odom: In[PoseStamped]
    goal_active: In[PoseStamped]
    goal_reached: In[Bool]
    path: In[NavPath]
    cmd_vel: In[Twist]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._received: dict[str, float] = {}
        self._lock = threading.Lock()
        self._unsub_fns: list = []

    @rpc
    def start(self) -> None:
        for stream_name in (
            "color_image",
            "lidar",
            "global_pointcloud",
            "odom",
            "goal_active",
            "goal_reached",
            "path",
            "cmd_vel",
        ):
            stream = getattr(self, stream_name)
            unsub = stream.subscribe(self._make_callback(stream_name))
            if unsub is not None:
                self._unsub_fns.append(unsub)

    def _make_callback(self, name: str):
        def _cb(_msg):
            with self._lock:
                if name not in self._received:
                    self._received[name] = time.time()

        return _cb

    @rpc
    def get_received(self) -> dict[str, float]:
        with self._lock:
            return dict(self._received)

    @rpc
    def stop(self) -> None:
        for unsub in self._unsub_fns:
            unsub()
        self._unsub_fns.clear()


@pytest.mark.slow
def test_rosnav_simulation_streams():
    """Start ROSNav in simulation mode and verify expected streams produce data."""

    coordinator = (
        autoconnect(
            ROSNav.blueprint(mode="simulation"),
            StreamCollector.blueprint(),
        )
        .global_config(viewer="none")
        .build()
    )

    try:
        collector = coordinator.get_instance(StreamCollector)
        start = time.time()
        missing = set(EXPECTED_STREAMS)

        while missing and (time.time() - start) < STREAM_TIMEOUT_SEC:
            received = collector.get_received()
            missing = EXPECTED_STREAMS - received.keys()
            if missing:
                time.sleep(2)

        received = collector.get_received()
        arrived = set(received.keys())

        for name in sorted(arrived):
            elapsed = received[name] - start
            print(f"  stream '{name}' first message after {elapsed:.1f}s")

        missing_expected = EXPECTED_STREAMS - arrived
        assert not missing_expected, (
            f"Timed out after {STREAM_TIMEOUT_SEC}s waiting for streams: {missing_expected}. "
            f"Received: {sorted(arrived)}"
        )

        for name in sorted(OPTIONAL_STREAMS & arrived):
            elapsed = received[name] - start
            print(f"  optional stream '{name}' arrived after {elapsed:.1f}s")
        for name in sorted(OPTIONAL_STREAMS - arrived):
            print(f"  optional stream '{name}' did not produce data (expected without hardware)")

    finally:
        coordinator.stop()
