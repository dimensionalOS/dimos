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

import itertools
import math

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.control.profile import (
    decode_ceilings,
    encode_precision,
    governor_speed,
)
from dimos.navigation.motion.embodiment.go2 import GO2

FLOOR_CLEARANCE, SPEED_CLEARANCE = GO2.precision, GO2.speed_clearance
MIN_SPEED, MAX_SPEED = GO2.min_speed, GO2.max_speed


def _pose(x: float, y: float, yaw: float = 0.0, ts: float = 0.0) -> PoseStamped:
    return PoseStamped(
        ts=ts,
        frame_id="odom",
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0, 0, yaw)),
    )


def _path(n: int = 21, step: float = 0.1) -> Path:
    return Path(frame_id="odom", poses=[_pose(i * step, 0.0, ts=1.0) for i in range(n)])


def test_governor_curve_endpoints() -> None:
    v = governor_speed(np.array([FLOOR_CLEARANCE, SPEED_CLEARANCE, 99.0, 0.0]), GO2)
    assert v[0] == MIN_SPEED
    assert v[1] == v[2] == MAX_SPEED
    assert v[3] == MIN_SPEED  # below the floor never re-accelerates


def test_roundtrip_recovers_governor_speeds() -> None:
    path = _path()
    clearance = np.linspace(0.05, 0.6, len(path.poses))
    encode_precision(path, clearance, GO2, t0=5.0)
    ceilings = decode_ceilings(path, MIN_SPEED, MAX_SPEED)
    assert ceilings is not None
    v = governor_speed(clearance, GO2)
    # segment ceiling = min of endpoint speeds; waypoint i carries segment i-1
    expected = np.minimum(v[:-1], v[1:])
    np.testing.assert_allclose(ceilings[1:], expected, rtol=1e-9)
    assert ceilings[0] == ceilings[1]


def test_tight_zone_reads_slow() -> None:
    path = _path()
    clearance = np.full(len(path.poses), 1.0)
    clearance[8:12] = FLOOR_CLEARANCE
    encode_precision(path, clearance, GO2)
    ceilings = decode_ceilings(path, MIN_SPEED, MAX_SPEED)
    assert ceilings is not None
    assert ceilings[10] == np.clip(MIN_SPEED, MIN_SPEED, MAX_SPEED)
    assert ceilings[3] == MAX_SPEED


def test_unstamped_path_decodes_none() -> None:
    assert decode_ceilings(_path(), MIN_SPEED, MAX_SPEED) is None  # flat ts everywhere


def test_non_monotone_decodes_none() -> None:
    path = _path()
    encode_precision(path, np.full(len(path.poses), 1.0), GO2)
    path.poses[5].ts = 999.0
    assert decode_ceilings(path, MIN_SPEED, MAX_SPEED) is None


def test_fan_keeps_timeline_monotone_and_inherits_ceiling() -> None:
    poses = [_pose(0.0, 0.0, 0.0), _pose(0.1, 0.0, 0.0)]
    poses += [_pose(0.1, 0.0, yaw) for yaw in (0.3, 0.6, 0.9)]  # rotation in place
    poses += [_pose(0.2, 0.0, 0.9)]
    path = Path(frame_id="odom", poses=poses)
    encode_precision(path, np.full(len(poses), 1.0), GO2)
    ts = [p.ts for p in path.poses]
    assert all(b > a for a, b in itertools.pairwise(ts))
    ceilings = decode_ceilings(path, MIN_SPEED, MAX_SPEED)
    assert ceilings is not None
    assert ceilings[3] == MAX_SPEED  # fan inherits, does not read as slow motion


def test_decode_clips_to_safe_band() -> None:
    # a foreign producer stamping absurdly fast segments cannot raise the cap
    path = _path(n=3)
    for i, p in enumerate(path.poses):
        p.ts = i * 1e-4  # implies 1000 m/s
    ceilings = decode_ceilings(path, MIN_SPEED, MAX_SPEED)
    assert ceilings is not None
    assert float(np.max(ceilings)) <= MAX_SPEED


def test_empty_and_single_pose() -> None:
    empty = Path(frame_id="odom", poses=[])
    assert decode_ceilings(encode_precision(empty, np.zeros(0), GO2), MIN_SPEED, MAX_SPEED) is None
    single = Path(frame_id="odom", poses=[_pose(0, 0)])
    assert decode_ceilings(encode_precision(single, np.zeros(1), GO2), MIN_SPEED, MAX_SPEED) is None


def test_stall_never_reads_as_schedule() -> None:
    # decode depends only on deltas: shifting all stamps changes nothing
    a, b = _path(), _path()
    clearance = np.linspace(0.1, 0.5, len(a.poses))
    encode_precision(a, clearance, GO2, t0=0.0)
    encode_precision(b, clearance, GO2, t0=12345.6)
    ca, cb = decode_ceilings(a, MIN_SPEED, MAX_SPEED), decode_ceilings(b, MIN_SPEED, MAX_SPEED)
    assert ca is not None and cb is not None
    np.testing.assert_allclose(ca, cb)
    assert math.isclose(b.poses[0].ts, 12345.6)
