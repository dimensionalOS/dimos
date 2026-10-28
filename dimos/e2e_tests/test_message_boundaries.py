# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Exercise generated messages at the simulator goal and raw bus boundaries."""

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from unittest.mock import Mock

from dimos_cdrgen.registry import encode

from dimos.e2e_tests.dim_sim_client import DimSimClient
from dimos.e2e_tests.lcm_spy import LcmSpy
from dimos.message_utils.time import time_from_seconds
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.std_msgs.msg import Header


def test_spy_decodes_cdr_pose_before_testing_position(monkeypatch) -> None:
    message = PoseStamped(
        header=Header(stamp=time_from_seconds(12.25), frame_id="map"),
        pose=Pose(position=Point(3.5, -2.25, 0.75), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
    )
    spy = object.__new__(LcmSpy)

    @contextmanager
    def listener(topic: str, callback: Callable[[bytes], None]) -> Iterator[None]:
        assert topic == "/odom#geometry_msgs.PoseStamped"
        callback(encode(message))
        yield

    monkeypatch.setattr(spy, "topic_listener", listener)
    spy.wait_until_odom_position(3.5, -2.25, threshold=0.001, timeout=0.1)


def test_simulator_goal_contains_requested_coordinates() -> None:
    client = object.__new__(DimSimClient)
    publisher = Mock()
    client._goal_request = publisher
    client.publish_goal(3.5, -2.25)
    (message,) = publisher.publish.call_args.args
    assert isinstance(message, PoseStamped)
    assert message.header.frame_id == "world"
    assert message.pose.position == Point(3.5, -2.25, 0.0)
    assert message.pose.orientation == Quaternion(0.0, 0.0, 0.0, 1.0)
