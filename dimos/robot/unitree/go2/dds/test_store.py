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

"""Open a real Go2 DDS mcap as a memory2 store (LFS-backed).

One test per message type — each prints a sample and checks the decode.
"""

from __future__ import annotations

import importlib.util

import numpy as np
import pytest

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.dds.msgs.ControlEvent import ControlEvent
from dimos.robot.unitree.go2.dds.msgs.LowState import LowState
from dimos.robot.unitree.go2.dds.msgs.SportModeState import SportModeState
from dimos.robot.unitree.go2.dds.msgs.Telemetry import Telemetry
from dimos.robot.unitree.go2.dds.store import Go2McapStore

pytestmark = [
    pytest.mark.self_hosted,
    pytest.mark.skipif(importlib.util.find_spec("mcap") is None, reason="mcap not installed"),
]


@pytest.fixture(scope="module")
def store() -> Go2McapStore:
    return Go2McapStore(path="go2_china_office_indoor.mcap")


def test_lists_streams(store: Go2McapStore) -> None:
    """Every decodable channel present in the file is listed (CDR + JSON)."""
    print("\n" + store.summary())
    assert set(store.list_streams()) == {
        "lidar",
        "imu",
        "odom",
        "color_image",
        "lowstate",
        "sportmodestate",
        "control_log",
        "telemetry",
    }
    assert store.streams.lowstate.count() > 0


def test_lidar(store: Go2McapStore) -> None:
    pc = store.streams.lidar.first().data
    xyz = pc.points_f32()
    print(
        f"\nPointCloud2: {xyz.shape[0]} pts  frame={pc.frame_id!r}  "
        f"range=[{np.linalg.norm(xyz[:, :3], axis=1).min():.2f}, "
        f"{np.linalg.norm(xyz[:, :3], axis=1).max():.2f}] m"
    )
    assert isinstance(pc, PointCloud2)
    assert xyz.shape[1] == 3 and len(xyz) > 0


def test_imu(store: Go2McapStore) -> None:
    imu = store.streams.imu.first().data
    q = imu.orientation
    print(
        f"\nImu: |q|={np.linalg.norm([q.x, q.y, q.z, q.w]):.4f}  "
        f"acc=({imu.linear_acceleration.x:.2f}, {imu.linear_acceleration.y:.2f}, "
        f"{imu.linear_acceleration.z:.2f})  frame={imu.frame_id!r}"
    )
    assert isinstance(imu, Imu)
    assert abs(imu.linear_acceleration.z) == pytest.approx(9.8, abs=0.5)  # gravity


def test_odom(store: Go2McapStore) -> None:
    odom = store.streams.odom.first().data
    p = odom.pose.pose.position
    print(
        f"\nOdometry: pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})  "
        f"{odom.frame_id!r} -> {odom.child_frame_id!r}"
    )
    assert isinstance(odom, Odometry)
    assert odom.child_frame_id == "base_link"


def test_color_image(store: Go2McapStore) -> None:
    img = store.streams.color_image.first().data
    arr = img.as_numpy()
    print(f"\nImage: {arr.shape}  frame={img.frame_id!r}")
    assert isinstance(img, Image)
    assert arr.ndim == 3


def test_lowstate(store: Go2McapStore) -> None:
    ls = store.streams.lowstate.first().data
    print(f"\n{ls}")
    assert isinstance(ls, LowState)
    assert len(ls.motor_state) == 20
    assert np.isclose(np.linalg.norm(ls.imu_state.quaternion), 1.0, atol=1e-2)


def test_sportmodestate(store: Go2McapStore) -> None:
    sm = store.streams.sportmodestate.first().data
    print(f"\n{sm}")
    assert isinstance(sm, SportModeState)
    assert sm.body_height > 0


def test_control_log(store: Go2McapStore) -> None:
    ev = store.streams.control_log.first().data
    print(f"\n{ev}")
    assert isinstance(ev, ControlEvent)
    assert isinstance(ev.type, str)


def test_telemetry(store: Go2McapStore) -> None:
    t = store.streams.telemetry.first().data
    print(f"\n{t}")
    assert isinstance(t, Telemetry)
    assert 0.0 <= t.battery <= 1.0


def test_read_contract(store: Go2McapStore) -> None:
    """count / first / last / limit / offset / order_by / time filter."""
    s = store.streams.odom
    first, last = s.first(), s.last()
    assert first.ts < last.ts
    assert len(s.limit(3).to_list()) == 3
    assert [o.id for o in s.offset(2).limit(2).to_list()] == [2, 3]
    # mcap is ts-ascending, so desc just reverses — top item is the last obs
    assert s.order_by("ts", desc=True).first().ts == last.ts
    assert s.after(first.ts + 10).first().ts > first.ts + 10
