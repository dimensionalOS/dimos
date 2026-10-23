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


from copy import deepcopy
import math

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest

from dimos.mapping.occupancy.path_resampling import simple_resample_path, smooth_resample_path
from dimos.message_utils.time import time_from_nanoseconds
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.geometry_msgs.Pose import Pose as LegacyPose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped as LegacyStamped
from dimos.msgs.nav_msgs.Path import Path as LegacyPath


def make_path(points, stamp):
    header = Header(frame_id="map", stamp=time_from_nanoseconds(stamp))
    return Path(
        header=deepcopy(header),
        poses=[
            PoseStamped(
                header=deepcopy(header),
                pose=Pose(
                    position=Point(x=x, y=y, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)
                ),
            )
            for x, y in points
        ],
    )


def values(path):
    return np.array(
        [
            [
                p.pose.position.x,
                p.pose.position.y,
                p.pose.position.z,
                p.pose.orientation.x,
                p.pose.orientation.y,
                p.pose.orientation.z,
                p.pose.orientation.w,
            ]
            for p in path.poses
        ]
    )


@pytest.mark.parametrize("method", [simple_resample_path, smooth_resample_path])
@pytest.mark.parametrize("stamp", [0, -1, 1700000000123456789])
@pytest.mark.parametrize("goal_yaw", [0, math.pi / 2])
def test_generated_resampling_keeps_exact_metadata_and_matches_existing_numerics(
    method, stamp, goal_yaw
):
    points = [(0.0, 0.0), (0.4, 0.0), (0.4, 0.4), (0.8, 0.4)]
    source = decode(encode(make_path(points, stamp)), Path)
    before = deepcopy(source)
    quaternion = Quaternion(x=0, y=0, z=math.sin(goal_yaw / 2), w=math.cos(goal_yaw / 2))
    goal = Pose(position=Point(x=0.8, y=0.4, z=0), orientation=quaternion)
    legacy = LegacyPath(
        frame_id="map",
        poses=[
            LegacyStamped(frame_id="map", position=[x, y, 0], orientation=[0, 0, 0, 1])
            for x, y in points
        ],
    )
    legacy_goal = LegacyPose(0.8, 0.4, 0, 0, 0, quaternion.z, quaternion.w)

    result = method(source, goal, 0.1)
    expected = method(legacy, legacy_goal, 0.1)

    assert type(result) is Path
    assert result.header == source.header
    assert all(p.header == source.header for p in result.poses)
    np.testing.assert_allclose(
        values(result),
        [
            [
                p.position.x,
                p.position.y,
                p.position.z,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w,
            ]
            for p in expected.poses
        ],
        rtol=0,
        atol=1e-14,
    )
    np.testing.assert_allclose(values(result)[[0, -1], :3], [[0, 0, 0], [0.8, 0.4, 0]])
    assert decode(encode(result), Path) == result
    assert source == before
    result.poses[0].pose.position.x = 100
    result.poses[0].header.frame_id = "changed"
    result.header.frame_id = "changed"
    assert source == before
    assert result.poses[-1].header == source.header


@pytest.mark.parametrize("method", [simple_resample_path, smooth_resample_path])
@pytest.mark.parametrize(
    "points,spacing",
    [
        ([], 0.1),
        ([(1.0, 2.0)], 0.1),
        ([(0.0, 0.0), (1.0, 1.0)], 0),
        ([(0.0, 0.0), (1.0, 1.0)], -1),
    ],
)
def test_generated_resampling_handles_empty_single_repeated_and_disabled_paths(
    method, points, spacing
):
    path = make_path(points, 0)
    goal = Pose(position=Point(x=1, y=2, z=0), orientation=Quaternion(x=0, y=0, z=1, w=0))

    result = method(path, goal, spacing)

    assert result.header == path.header
    assert len(result.poses) == len(points)
    assert [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in result.poses] == [
        (x, y, 0) for x, y in points
    ]


@pytest.mark.parametrize("method,count", [(simple_resample_path, 1), (smooth_resample_path, 2)])
def test_generated_repeated_coordinates_retain_existing_method_behavior(method, count):
    path = make_path([(1.0, 2.0), (1.0, 2.0)], 0)
    goal = Pose(position=Point(x=1, y=2, z=0), orientation=Quaternion(x=0, y=0, z=1, w=0))
    result = method(path, goal, 0.1)
    assert len(result.poses) == count
    assert [(p.pose.position.x, p.pose.position.y) for p in result.poses] == [(1.0, 2.0)] * count


def test_generated_single_smooth_pose_uses_goal_orientation_and_copies_metadata():
    path = make_path([(1.0, 2.0)], -1)
    before = deepcopy(path)
    goal = Pose(position=Point(x=1, y=2, z=0), orientation=Quaternion(x=0, y=0, z=1, w=0))
    result = smooth_resample_path(path, goal, 0.1)
    assert result.poses[0].pose.orientation == goal.orientation
    assert result.header == path.header and result.poses[0].header == path.header
    assert path == before


def test_simple_resampling_preserves_existing_endpoint_source_headers_without_aliasing():
    path = make_path([(0.0, 0.0), (0.25, 0.0)], -1)
    path.poses[0].header.stamp = time_from_nanoseconds(0)
    path.poses[-1].header.stamp = time_from_nanoseconds(17)
    before = deepcopy(path)
    goal = Pose(position=Point(x=0.25, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1))

    result = simple_resample_path(path, goal, 0.1)

    assert [p.header.stamp for p in result.poses] == [
        path.poses[0].header.stamp,
        path.header.stamp,
        path.header.stamp,
        path.poses[-1].header.stamp,
    ]
    assert path == before
