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

"""Math behavior of the stock align interpolators (lerp_pose / interp_odom).

These guard against silent value drift: a sign error in the slerp shortcut, a
hard-coded midpoint instead of the true alpha fraction, or a division blowup
on parallel rotations would all still return plausible-looking poses.
"""

from __future__ import annotations

import math

import pytest

from dimos.memory2.interpolators import interp_odom, lerp_pose
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry


def _pose_obs(
    ts: float, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, frame_id: str = "odom"
) -> Observation[PoseStamped]:
    pose = PoseStamped(
        ts=ts,
        frame_id=frame_id,
        position=(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )
    return Observation(ts=ts, _data=pose)


def _odom_obs(
    ts: float, x: float = 0.0, vx: float = 0.0, wz: float = 0.0, yaw: float = 0.0
) -> Observation[Odometry]:
    odom = Odometry(
        ts=ts,
        frame_id="odom",
        child_frame_id="base_link",
        pose=Pose(position=(x, 0.0, 0.0), orientation=Quaternion.from_euler(Vector3(0, 0, yaw))),
        twist=Twist((vx, 0.0, 0.0), (0.0, 0.0, wz)),
    )
    return Observation(ts=ts, _data=odom)


class TestLerpPose:
    def test_midpoint_lerps_position_and_slerps_yaw_to_45_degrees(self) -> None:
        """Halfway between identity and a 90-degree yaw the pose is at the
        position midpoint with yaw 45 degrees - the canonical pose-at-scan-time
        case - stamped at the interpolated time in the prev frame."""
        prev = _pose_obs(1.0, x=0.0, y=0.0, yaw=0.0)
        nxt = _pose_obs(2.0, x=1.0, y=2.0, yaw=math.pi / 2)

        mid = lerp_pose(prev, nxt, 0.5)

        assert (mid.x, mid.y, mid.z) == pytest.approx((0.5, 1.0, 0.0))
        assert mid.yaw == pytest.approx(math.pi / 4)
        assert mid.ts == pytest.approx(1.5)
        assert mid.frame_id == "odom"

    def test_alpha_tracks_the_bracket_fraction_not_the_midpoint(self) -> None:
        """A quarter of the way in gives a quarter of the motion (yaw 22.5
        degrees, position quarter-point) - a hard-coded 0.5 would pass the
        midpoint test above but fail here."""
        prev = _pose_obs(1.0, x=0.0, yaw=0.0)
        nxt = _pose_obs(2.0, x=4.0, yaw=math.pi / 2)

        quarter = lerp_pose(prev, nxt, 0.25)

        assert quarter.x == pytest.approx(1.0)
        assert quarter.yaw == pytest.approx(math.pi / 8)
        assert quarter.ts == pytest.approx(1.25)

    def test_slerp_takes_the_short_arc_across_the_quaternion_sign_flip(self) -> None:
        """Between yaw +170 and -170 degrees the short arc passes through 180,
        so the midpoint sits 10 degrees from each endpoint. Skipping the
        double-cover sign flip would rotate the long way through 0, putting the
        midpoint 170 degrees from each."""
        prev = _pose_obs(1.0, yaw=math.radians(170.0))
        nxt = _pose_obs(2.0, yaw=math.radians(-170.0))

        mid = lerp_pose(prev, nxt, 0.5)

        assert mid.orientation.angle_to(prev.data.orientation) == pytest.approx(
            math.radians(10.0), abs=1e-9
        )
        assert mid.orientation.angle_to(nxt.data.orientation) == pytest.approx(
            math.radians(10.0), abs=1e-9
        )

    def test_parallel_orientations_interpolate_without_degeneracy(self) -> None:
        """Identical (and near-identical) orientations hit the sin(theta) -> 0
        regime; the nlerp fallback must return the same rotation as a unit
        quaternion rather than NaN or a zero division."""
        prev = _pose_obs(1.0, x=0.0, yaw=1.0)
        nxt = _pose_obs(2.0, x=2.0, yaw=1.0)

        mid = lerp_pose(prev, nxt, 0.5)

        assert mid.yaw == pytest.approx(1.0)
        q = mid.orientation
        assert math.isfinite(q.x) and math.isfinite(q.w)
        assert q.dot(q) == pytest.approx(1.0)
        assert mid.x == pytest.approx(1.0)


class TestInterpOdom:
    def test_pose_and_twist_interpolate_at_the_primary_ts(self) -> None:
        """Position, yaw, linear and angular velocity all move by the alpha
        fraction and the stamp is the interpolated time with both frames
        carried - a stale (snapped) twist would report the wrong velocity for
        the scan instant."""
        prev = _odom_obs(1.0, x=0.0, vx=0.0, wz=0.2, yaw=0.0)
        nxt = _odom_obs(2.0, x=2.0, vx=1.0, wz=0.6, yaw=math.pi / 2)

        out = interp_odom(prev, nxt, 0.5)

        assert out.x == pytest.approx(1.0)
        assert out.yaw == pytest.approx(math.pi / 4)
        assert out.vx == pytest.approx(0.5)
        assert out.wz == pytest.approx(0.4)
        assert out.ts == pytest.approx(1.5)
        assert out.frame_id == "odom"
        assert out.child_frame_id == "base_link"
