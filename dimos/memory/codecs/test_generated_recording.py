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

from dimos_lcm.geometry_msgs.Vector3 import Vector3 as LCMVector3
import pytest

from dimos.memory.codecs.base import codec_for, codec_from_id, codec_id
from dimos.messages.geometry_msgs.msg import Vector3


def test_historical_envelope_returns_generated_value():
    old = LCMVector3(1.0, 2.0, 3.0)
    codec = codec_from_id("lcm", "dimos.msgs.geometry_msgs.Vector3.Vector3")
    result = codec.decode(old.lcm_encode())
    assert type(result) is Vector3
    assert (result.x, result.y, result.z) == (1.0, 2.0, 3.0)
    with pytest.raises(TypeError, match="read-only"):
        codec.encode(result)


def test_new_generated_recordings_use_cdr():
    codec = codec_for(Vector3)
    assert codec_id(codec) == "cdr"
    assert codec.decode(codec.encode(Vector3(1.0, 2.0, 3.0))) == Vector3(1.0, 2.0, 3.0)


def test_historical_weighted_segments_keep_generated_identity_and_weights():
    from dimos_lcm.geometry_msgs import PoseStamped
    from dimos_lcm.nav_msgs import Path

    from dimos.messages.dimos_msgs.msg import LineSegments3D

    old = Path()
    old.header.frame_id = "map"
    old.header.stamp.sec = 12
    a, b = PoseStamped(), PoseStamped()
    a.pose.position.x = 1.5
    b.pose.position.y = -2.5
    a.pose.orientation.w = 3.75
    old.poses = [a, b]
    old.poses_length = 2
    codec = codec_from_id("lcm", "dimos.msgs.nav_msgs.LineSegments3D.LineSegments3D")
    result = codec.decode(old.lcm_encode())
    assert type(result) is LineSegments3D
    assert result.header.frame_id == "map"
    assert result.header.stamp.sec == 12
    assert result.segments[0].start.x == 1.5
    assert result.segments[0].end.y == -2.5
    assert result.segments[0].weight == 3.75


def test_historical_transform_wrapper_reads_first_tf_edge_as_generated_stamped():
    from dimos_lcm.geometry_msgs import TransformStamped
    from dimos_lcm.tf2_msgs import TFMessage

    from dimos.messages.geometry_msgs.msg import TransformStamped as GeneratedTransform

    old = TFMessage()
    edge = TransformStamped()
    edge.header.frame_id = "map"
    edge.header.stamp.sec = 7
    edge.child_frame_id = "sensor"
    edge.transform.translation.z = 4.25
    old.transforms = [edge]
    old.transforms_length = 1
    result = codec_from_id("lcm", "dimos.msgs.geometry_msgs.Transform.Transform").decode(
        old.lcm_encode()
    )
    assert type(result) is GeneratedTransform
    assert result.header.stamp.sec == 7
    assert result.header.frame_id == "map"
    assert result.child_frame_id == "sensor"
    assert result.transform.translation.z == 4.25
