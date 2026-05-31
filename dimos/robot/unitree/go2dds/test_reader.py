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
#
# Licensed under the Apache License, Version 2.0 (the "License").

"""Reader smoke tests against a real Go2 DDS mcap (LFS-backed)."""

from __future__ import annotations

from itertools import islice

import numpy as np
import pytest

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2dds import reader
from dimos.robot.unitree.go2dds.msgs.LowState import LowState
from dimos.utils.data import get_data

pytestmark = pytest.mark.self_hosted


@pytest.fixture(scope="module")
def mcap() -> str:
    return str(get_data("go2_china_office_indoor.mcap"))


def test_streams_lists_every_channel(mcap: str) -> None:
    """streams() reports all 8 DDS channels and which are decodable."""
    rows = reader.streams(mcap)

    for row in rows:
        print(row)

    by_topic = {ch["topic"]: ch for ch in rows}

    assert len(rows) == 8
    assert "rt/lowstate" in by_topic
    assert "telemetry" in by_topic  # json channel, listed but not decodable

    decodable = {ch["topic"] for ch in rows if ch["decodable"]}
    assert decodable == set(reader.REGISTRY)  # the 6 CDR channels
    assert by_topic["rt/lowstate"]["count"] > 0


def test_messages_iterates_decoded(mcap: str) -> None:
    """messages() yields (topic, ts, msg) in increasing ts for a topic."""
    rows = list(islice(reader.messages(mcap, "rt/lowstate"), 5))
    assert len(rows) == 5
    assert all(topic == "rt/lowstate" for topic, _ts, _msg in rows)
    assert all(isinstance(msg, LowState) for _topic, _ts, msg in rows)

    ts = [t for _topic, t, _msg in rows]
    assert ts == sorted(ts)
    assert len(rows[0][2].motor_state) == 20  # decoded nested array


def test_ros_channels_decode_into_dimos_types(mcap: str) -> None:
    """The 4 standard-ROS channels decode into dimos.msgs types with sane values."""

    def first(topic: str):  # type: ignore[no-untyped-def]
        return next(reader.messages(mcap, topic))[2]

    imu = first("rt/utlidar/imu")
    assert isinstance(imu, Imu)
    q = imu.orientation
    assert np.isclose(np.linalg.norm([q.x, q.y, q.z, q.w]), 1.0, atol=1e-2)
    assert abs(imu.linear_acceleration.z) == pytest.approx(9.8, abs=0.5)  # gravity

    pc = first("rt/utlidar/cloud")
    assert isinstance(pc, PointCloud2)
    assert pc.points_f32().shape[1] == 3 and len(pc.points_f32()) > 0

    odom = first("rt/utlidar/robot_odom")
    assert isinstance(odom, Odometry)
    assert odom.child_frame_id == "base_link"

    img = first("rt/frontvideo")
    assert isinstance(img, Image)
    assert img.as_numpy().ndim == 3
