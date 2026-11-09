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

from copy import deepcopy
from dataclasses import asdict

import numpy as np
import pytest

from dimos.memory.type.observation import Observation
from dimos.message_utils import time as message_time
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos.messages.nav_msgs.msg import Odometry
from dimos.messages.std_msgs.msg import Header


def canonical_pose_values():
    header = Header(frame_id="odom", stamp=message_time.time_from_nanoseconds(1700000000123456789))
    pose = Pose(
        position=Point(x=1.5, y=-2, z=0.7), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.927)
    )
    transform = Transform(
        translation=Vector3(x=1.5, y=-2, z=0.7), rotation=deepcopy(pose.orientation)
    )
    covariance = PoseWithCovariance(pose=deepcopy(pose), covariance=np.zeros(36, dtype=np.float64))
    return [
        pose,
        PoseStamped(header=deepcopy(header), pose=deepcopy(pose)),
        transform,
        TransformStamped(
            header=deepcopy(header), child_frame_id="base", transform=deepcopy(transform)
        ),
        covariance,
        Odometry(
            header=deepcopy(header),
            child_frame_id="base",
            pose=deepcopy(covariance),
            twist=TwistWithCovariance(
                twist=Twist(linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=0)),
                covariance=np.zeros(36, dtype=np.float64),
            ),
        ),
    ]


@pytest.mark.parametrize("index", range(6))
def test_canonical_pose_payloads_normalize_metadata_without_loading_or_mutation(index):
    value = canonical_pose_values()[index]
    before = asdict(value)
    loads = []

    def load():
        loads.append("loaded")
        return "payload"

    observation = Observation(id=7, ts=4.5, _loader=load)
    posed = observation.with_pose(value)
    assert posed.pose_tuple == (1.5, -2.0, 0.7, 0.1, 0.2, 0.3, 0.927)
    assert observation.pose_tuple is None and posed.id == 7 and posed.ts == 4.5
    assert loads == []
    assert posed.data == observation.data == "payload"
    assert loads == ["loaded"]
    np.testing.assert_equal(asdict(value), before)
    assert observation.with_pose(None).pose_tuple is None


def test_unsupported_message_is_not_silently_treated_as_a_pose():
    header = Header(frame_id="odom", stamp=message_time.time_from_nanoseconds(0))
    with pytest.raises(TypeError, match="Cannot coerce"):
        Observation(pose=header, _data="unchanged")


def test_pose_accessors_return_generated_values_with_exact_storage_coordinates():
    observation = Observation(ts=12.25, pose_tuple=(1.0, -2.0, 3.0, 0.0, 0.0, 0.0, 1.0))
    pose = observation.pose
    stamped = observation.pose_stamped
    assert isinstance(pose, Pose)
    assert isinstance(stamped, PoseStamped)
    assert (pose.position.x, pose.position.y, pose.position.z) == (1.0, -2.0, 3.0)
    assert stamped.pose == pose
    assert (stamped.header.stamp.sec, stamped.header.stamp.nanosec) == (12, 250000000)
    pose.position.x = 100.0
    assert observation.pose_tuple == (1.0, -2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
