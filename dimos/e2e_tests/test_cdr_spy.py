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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode

from dimos.core.global_config import global_config
from dimos.e2e_tests.lcm_spy import LcmSpy


def test_spy_publishes_generated_cdr_on_qualified_channel(mocker, monkeypatch):
    monkeypatch.setattr(global_config, "transport", "lcm")
    bus = mocker.patch("dimos.e2e_tests.lcm_spy.LCMPubSubBase").return_value
    spy = LcmSpy()
    message = PoseStamped(
        header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)),
        pose=Pose(
            position=Point(x=2, y=3, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        ),
    )
    spy.publish("/goal_request#geometry_msgs/msg/PoseStamped", message)
    topic, payload = bus.publish.call_args.args
    assert str(topic) == "/goal_request#geometry_msgs/msg/PoseStamped"
    decoded = cdr_decode(payload, PoseStamped)
    assert (decoded.pose.position.x, decoded.pose.position.y, decoded.header.frame_id) == (
        2,
        3,
        "map",
    )


def test_spy_odometry_predicate_uses_nested_pose(mocker, monkeypatch):
    monkeypatch.setattr(global_config, "transport", "lcm")
    mocker.patch("dimos.e2e_tests.lcm_spy.LCMPubSubBase")
    spy = LcmSpy()
    wait = mocker.patch.object(spy, "wait_for_message_result")
    spy.wait_until_odom_position(2, 3, threshold=0.5)
    topic, message_type, predicate, _, timeout = wait.call_args.args
    assert (topic, message_type, timeout) == (
        "/odom#geometry_msgs/msg/PoseStamped",
        PoseStamped,
        60,
    )
    assert predicate(
        PoseStamped(
            pose=Pose(
                position=Point(x=2, y=3, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
    )
    assert not predicate(
        PoseStamped(
            pose=Pose(
                position=Point(x=3, y=3, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
    )
