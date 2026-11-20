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

"""The masks the Rust body filter is tested against (robosense/rust/tests/body_filter.rs) are the Python filter's:
RoboPlan removes exactly the stored points from the real M20 scans, and every point of the synthetic body cloud."""

from pathlib import Path
import struct

import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.deeprobotics.m20 import constants
from dimos.robot.deeprobotics.m20.model import M20_MODEL
from dimos.robot.roboplan_body_filter import RoboPlanBodyFilter

_FIXTURES = Path(__file__).parents[3] / "hardware/sensors/lidar/robosense/rust/tests/fixtures"


def _records(path: Path) -> list[tuple[str, dict[str, float], np.ndarray, np.ndarray]]:
    data, at, out = path.read_bytes(), 0, []

    def string() -> str:
        nonlocal at
        (n,) = struct.unpack_from("<I", data, at)
        at += 4 + n
        return data[at - n : at].decode()

    while at < len(data):
        frame = string()
        _stamp, joint_count = struct.unpack_from("<dI", data, at)
        at += 12
        joints = {}
        for _ in range(joint_count):
            name = string()
            (joints[name],) = struct.unpack_from("<d", data, at)
            at += 8
        (n,) = struct.unpack_from("<I", data, at)
        at += 4
        points = np.frombuffer(data, dtype="<f4", count=3 * n, offset=at).reshape(-1, 3)
        at += 12 * n
        on_body = np.frombuffer(data, dtype=np.uint8, count=n, offset=at).astype(bool)
        at += n
        out.append((frame, joints, points, on_body))
    return out


def _roboplan_on_body(frame: str, joints: dict[str, float], points: np.ndarray) -> np.ndarray:
    front = frame == constants.FRONT_LIDAR_FRAME
    xyz = constants.FRONT_LIDAR_XYZ_M if front else constants.REAR_LIDAR_XYZ_M
    rpy = constants.FRONT_LIDAR_RPY if front else constants.REAR_LIDAR_RPY
    mount = Transform(
        translation=Vector3(*xyz),
        rotation=Quaternion.from_euler(Vector3(*rpy)),
        frame_id="base_link",
        child_frame_id=frame,
    ).to_matrix()
    base = np.asarray(points @ mount[:3, :3].T + mount[:3, 3], dtype=np.float64)
    keep = RoboPlanBodyFilter(M20_MODEL.load(), 0.01).keep_mask(base, joints, {})
    assert keep is not None
    return ~keep


def test_the_real_scan_masks_are_roboplans() -> None:
    records = _records(_FIXTURES / "m20_body_scans.bin")
    assert {frame for frame, *_ in records} == {"front_lidar", "rear_lidar"}
    for frame, joints, points, on_body in records:
        np.testing.assert_array_equal(_roboplan_on_body(frame, joints, points), on_body)
    assert sum(int(on_body.sum()) for *_, on_body in records) > 1_500


def test_roboplan_removes_the_whole_synthetic_body() -> None:
    for frame, joints, points, on_body in _records(_FIXTURES / "m20_synthetic_body.bin"):
        assert on_body.all()
        assert _roboplan_on_body(frame, joints, points).all()
