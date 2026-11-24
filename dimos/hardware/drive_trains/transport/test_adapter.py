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

from dimos.core.transport_factory import make_transport
from dimos.hardware.drive_trains.transport.adapter import TransportTwistAdapter
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry


def test_odometry_on_the_odom_topic_becomes_the_base_odometry() -> None:
    adapter = TransportTwistAdapter(hardware_id="test_transport_twist")
    adapter.connect()
    publisher = make_transport("/test_transport_twist/odom", Odometry)
    odom = Odometry(
        frame_id="odom",
        child_frame_id="base_link",
        pose=Pose(Vector3(1.0, 2.0, 0.0), Quaternion.from_euler(Vector3(0.0, 0.0, 0.5))),
    )
    try:
        deadline = time.monotonic() + 5.0
        while adapter.read_odometry() is None and time.monotonic() < deadline:
            publisher.publish(odom)
            time.sleep(0.05)
        assert adapter.read_odometry() == pytest.approx([1.0, 2.0, 0.5])
    finally:
        adapter.disconnect()
        publisher.stop()
