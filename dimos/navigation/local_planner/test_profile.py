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

import numpy as np

from dimos.message_utils.geometry import quaternion_from_euler
from dimos.message_utils.time import time_from_nanoseconds, time_from_seconds
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.std_msgs.msg import Header
from dimos.navigation.embodiment.go2 import GO2
from dimos.navigation.local_planner.profile import decode_ceilings

MIN_SPEED, MAX_SPEED = GO2.min_speed, GO2.max_speed


def _pose(x: float, y: float, yaw: float = 0.0, ts: float = 0.0) -> PoseStamped:
    return PoseStamped(
        header=Header(frame_id="odom", stamp=time_from_seconds(ts)),
        pose=Pose(position=Point(x=x, y=y, z=0.0), orientation=quaternion_from_euler(0, 0, yaw)),
    )


def _path(n: int = 21, step: float = 0.1) -> Path:
    return Path(
        header=Header(frame_id="odom", stamp=time_from_seconds(0.0)),
        poses=[_pose(i * step, 0.0, ts=1.0) for i in range(n)],
    )


def _stamp(path: Path, dts: list[float], t0: float = 0.0) -> Path:
    t = t0
    path.poses[0].header.stamp = time_from_seconds(t)
    for pose, dt in zip(path.poses[1:], dts, strict=True):
        t += dt
        pose.header.stamp = time_from_seconds(t)
    return path


def test_ceiling_is_segment_length_over_stamp_delta() -> None:
    speeds = np.linspace(MIN_SPEED, MAX_SPEED, 20)
    # Keep a varied speed sweep while using exactly representable ROS time deltas.
    # The old 0.1 / v fixture required sub-nanosecond timestamp precision.
    positions = np.concatenate([[0.0], np.cumsum(speeds * 0.5)])
    path = _stamp(
        Path(
            header=Header(frame_id="odom", stamp=time_from_seconds(0.0)),
            poses=[_pose(float(x), 0.0) for x in positions],
        ),
        [0.5] * 20,
    )
    ceilings = decode_ceilings(path, MIN_SPEED, MAX_SPEED)
    assert ceilings is not None
    # waypoint i carries segment i-1
    np.testing.assert_allclose(ceilings[1:], speeds, rtol=1e-9)
    assert ceilings[0] == ceilings[1]


def test_unstamped_path_decodes_none() -> None:
    assert decode_ceilings(_path(), MIN_SPEED, MAX_SPEED) is None  # flat ts everywhere


def test_non_monotone_decodes_none() -> None:
    path = _stamp(_path(), [0.1 / MAX_SPEED] * 20)
    path.poses[5].header.stamp = time_from_seconds(999.0)
    assert decode_ceilings(path, MIN_SPEED, MAX_SPEED) is None


def test_fan_inherits_ceiling() -> None:
    poses = [_pose(0.0, 0.0, 0.0), _pose(MAX_SPEED * 0.1, 0.0, 0.0)]
    poses += [_pose(MAX_SPEED * 0.1, 0.0, yaw) for yaw in (0.3, 0.6, 0.9)]  # rotation in place
    poses += [_pose(MAX_SPEED * 0.2, 0.0, 0.9)]
    fan_dt = 0.3 / GO2.max_yaw_rate
    path = _stamp(
        Path(header=Header(frame_id="odom", stamp=time_from_seconds(0.0)), poses=poses),
        [0.1] + [fan_dt] * 3 + [0.5],
    )
    ceilings = decode_ceilings(path, MIN_SPEED, MAX_SPEED)
    assert ceilings is not None
    assert ceilings[3] == MAX_SPEED  # fan inherits, does not read as slow motion


def test_decode_clips_to_safe_band() -> None:
    # a foreign producer stamping absurdly fast segments cannot raise the cap
    path = _stamp(_path(n=3), [1e-4, 1e-4])  # implies 1000 m/s
    ceilings = decode_ceilings(path, MIN_SPEED, MAX_SPEED)
    assert ceilings is not None
    assert float(np.max(ceilings)) <= MAX_SPEED


def test_empty_and_single_pose() -> None:
    assert (
        decode_ceilings(
            Path(header=Header(frame_id="odom", stamp=time_from_seconds(0.0)), poses=[]),
            MIN_SPEED,
            MAX_SPEED,
        )
        is None
    )
    assert (
        decode_ceilings(
            Path(header=Header(frame_id="odom", stamp=time_from_seconds(0.0)), poses=[_pose(0, 0)]),
            MIN_SPEED,
            MAX_SPEED,
        )
        is None
    )


def test_stall_never_reads_as_schedule() -> None:
    # decode depends only on deltas: shifting all stamps changes nothing
    dts = [0.1 / v for v in np.linspace(MIN_SPEED, MAX_SPEED, 20)]
    a, b = _stamp(_path(), dts, t0=0.0), _stamp(_path(), dts, t0=12345.6)
    ca, cb = decode_ceilings(a, MIN_SPEED, MAX_SPEED), decode_ceilings(b, MIN_SPEED, MAX_SPEED)
    assert ca is not None and cb is not None
    np.testing.assert_allclose(ca, cb)


def test_exact_epoch_nanosecond_deltas_drive_the_profile_without_float_loss():
    path = _path(n=3, step=0.1)
    base = 1700000000123456789
    for pose, delta in zip(path.poses, [0, 200000000, 700000000], strict=True):
        pose.header.stamp = time_from_nanoseconds(base + delta)
    ceilings = decode_ceilings(path, MIN_SPEED, MAX_SPEED)
    np.testing.assert_allclose(ceilings, [0.5, 0.5, 0.2], rtol=0, atol=1e-15)
