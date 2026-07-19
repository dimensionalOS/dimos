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

"""Geometry interpolators: math, frame guards, registry install, e2e over()."""

from __future__ import annotations

import math

import pytest

from dimos import pure as pm
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.pure import interpolators
from dimos.pure.align import interpolator_for


def _quat_close(a: Quaternion, b: Quaternion, tol: float = 1e-9) -> bool:
    # q and -q are the same rotation
    d = abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
    return abs(d - 1.0) < tol


def test_slerp_endpoints() -> None:
    qa = Quaternion(0.0, 0.0, 0.0, 1.0)
    qb = Quaternion(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))  # 90° about z
    assert _quat_close(interpolators.slerp(qa, qb, 0.0), qa)
    assert _quat_close(interpolators.slerp(qa, qb, 1.0), qb)


def test_slerp_midpoint_is_half_rotation() -> None:
    qa = Quaternion(0.0, 0.0, 0.0, 1.0)
    qb = Quaternion(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    mid = interpolators.slerp(qa, qb, 0.5)
    expected = Quaternion(0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))  # 45°
    assert _quat_close(mid, expected)


def test_slerp_takes_short_path() -> None:
    qa = Quaternion(0.0, 0.0, 0.0, 1.0)
    qb = Quaternion(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    neg = Quaternion(-qb.x, -qb.y, -qb.z, -qb.w)  # same rotation, antipodal encoding
    assert _quat_close(interpolators.slerp(qa, neg, 0.5), interpolators.slerp(qa, qb, 0.5))


def test_slerp_near_parallel_stays_unit() -> None:
    qa = Quaternion(0.0, 0.0, 0.0, 1.0)
    qb = Quaternion(0.0, 0.0, 1e-5, 1.0)  # unnormalized on purpose
    out = interpolators.slerp(qa, qb, 0.5)
    n = math.sqrt(out.x**2 + out.y**2 + out.z**2 + out.w**2)
    assert abs(n - 1.0) < 1e-9


def test_slerp_rejects_zero_norm() -> None:
    with pytest.raises(ValueError, match="zero-norm"):
        interpolators.slerp(Quaternion(0.0, 0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0), 0.5)


def test_lerp_vec_midpoint() -> None:
    v = interpolators.lerp_vec(Vector3(0.0, 2.0, -4.0), Vector3(1.0, 4.0, 4.0), 0.5)
    assert (v.x, v.y, v.z) == (0.5, 3.0, 0.0)


def test_interp_pose_combined() -> None:
    a = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    b = Pose(2.0, 0.0, 0.0, 0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    mid = interpolators.interp_pose(a, b, 0.5)
    assert (mid.position.x, mid.position.y, mid.position.z) == (1.0, 0.0, 0.0)
    assert _quat_close(
        mid.orientation, Quaternion(0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))
    )


def test_interp_pose_stamped_ts_and_frame() -> None:
    a = PoseStamped(ts=10.0, frame_id="odom", position=[0.0, 0.0, 0.0])
    b = PoseStamped(ts=20.0, frame_id="odom", position=[4.0, 0.0, 0.0])
    mid = interpolators.interp_pose_stamped(a, b, 0.25)
    assert mid.ts == 12.5
    assert mid.frame_id == "odom"
    assert mid.position.x == 1.0


def test_interp_pose_stamped_frame_mismatch_raises() -> None:
    a = PoseStamped(ts=1.0, frame_id="odom")
    b = PoseStamped(ts=2.0, frame_id="map")
    with pytest.raises(ValueError, match="different frames"):
        interpolators.interp_pose_stamped(a, b, 0.5)


def test_interp_transform_ts_and_frames() -> None:
    a = Transform(
        translation=Vector3(0.0, 0.0, 0.0), frame_id="odom", child_frame_id="base", ts=1.0
    )
    b = Transform(
        translation=Vector3(1.0, 1.0, 0.0), frame_id="odom", child_frame_id="base", ts=3.0
    )
    mid = interpolators.interp_transform(a, b, 0.5)
    assert mid.ts == 2.0
    assert (mid.frame_id, mid.child_frame_id) == ("odom", "base")
    assert (mid.translation.x, mid.translation.y) == (0.5, 0.5)
    c = Transform(frame_id="odom", child_frame_id="lidar", ts=3.0)
    with pytest.raises(ValueError, match="different frames"):
        interpolators.interp_transform(a, c, 0.5)


def test_install_registers_and_is_idempotent() -> None:
    interpolators.install()
    interpolators.install()
    for tp in (Vector3, Quaternion, Pose, PoseStamped, Transform):
        assert interpolator_for(tp) is not None


def test_pose_interpolates_through_over() -> None:
    interpolators.install()

    class Tracker(pm.PureModule):
        class In(pm.In):
            scan: PoseStamped = pm.tick()  # any stamped payload works as a trigger
            pose: PoseStamped = pm.interpolate()

        class Out(pm.Out):
            x: float = 0.0

        def step(self, i: In) -> Out:
            return Tracker.Out(x=i.pose.position.x)

    scans = [PoseStamped(ts=t, frame_id="s") for t in (1.0, 2.0)]
    poses = [
        PoseStamped(ts=0.5, frame_id="odom", position=[0.0, 0.0, 0.0]),
        PoseStamped(ts=2.5, frame_id="odom", position=[2.0, 0.0, 0.0]),
    ]
    rows = list(Tracker().over(scan=scans, pose=poses))
    # ticks at 1.0 and 2.0 → alpha 0.25 / 0.75 over [0.5, 2.5] → x = 0.5 / 1.5
    assert [r.x for r in rows] == [0.5, 1.5]
    assert [r.ts for r in rows] == [1.0, 2.0]
