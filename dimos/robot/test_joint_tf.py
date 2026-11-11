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

"""A description, a joint stream and a tf stream are enough to get a robot onto tf."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.joint_tf import JointTf

_URDF = """<?xml version="1.0"?>
<robot name="box_robot">
  <link name="base"/>
  <link name="arm"/>
  <link name="hand"/>
  <link name="finger"/>
  <joint name="shoulder" type="{joint_type}">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
  </joint>
  <joint name="wrist" type="fixed">
    <parent link="arm"/>
    <child link="hand"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>
</robot>
"""


class _Sink:
    """A tf stream that only remembers what it was handed."""

    def __init__(self) -> None:
        self.sent: list[Transform] = []

    def publish(self, *transforms: Transform) -> None:
        self.sent.extend(transforms)

    @property
    def edges(self) -> list[tuple[str, str]]:
        return [(one.frame_id, one.child_frame_id) for one in self.sent]


class _Stream:
    """A joint-state stream a test can push onto."""

    def __init__(self) -> None:
        self.callbacks: list[Callable[[JointState], Any]] = []
        self.unsubscribed = 0

    def subscribe(self, cb: Callable[[JointState], Any]) -> Callable[[], None]:
        self.callbacks.append(cb)

        def drop() -> None:
            self.unsubscribed += 1
            self.callbacks.remove(cb)

        return drop

    def publish(self, state: JointState) -> None:
        for cb in list(self.callbacks):
            cb(state)


@pytest.fixture
def make_model(tmp_path: Path) -> Iterator[Callable[..., RobotModel]]:
    urdf = tmp_path / "robot.urdf"

    def make(joint_type: str = "revolute") -> RobotModel:
        urdf.write_text(_URDF.format(joint_type=joint_type))
        return RobotModel.from_file(urdf)

    yield make


def _wait_for(predicate: Callable[[], bool], what: str, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError(f"timed out waiting for {what}")


def test_a_joint_state_is_one_tf_edge_per_joint(make_model) -> None:
    joint_tf = JointTf(make_model("prismatic"))
    sink = _Sink()

    joint_tf.add_joint_state(JointState(ts=1.0, name=["shoulder"], position=[0.3]), sink)

    assert sink.edges == [("base", "arm"), ("arm", "hand")]
    moved = next(one for one in sink.sent if one.child_frame_id == "arm")
    np.testing.assert_allclose(moved.translation.to_tuple(), (0.3, 0.0, 0.5), atol=1e-6)
    assert moved.ts == 1.0, "the edge carries the state's own stamp, not the wall clock"


def test_a_joint_the_state_leaves_out_stays_at_zero(make_model) -> None:
    joint_tf = JointTf(make_model("prismatic"))
    sink = _Sink()

    joint_tf.add_joint_state(JointState(ts=1.0, name=["elbow"], position=[0.3]), sink)

    moved = next(one for one in sink.sent if one.child_frame_id == "arm")
    np.testing.assert_allclose(moved.translation.to_tuple(), (0.0, 0.0, 0.5), atol=1e-6)


def test_the_fixed_edges_need_no_joint_state(make_model) -> None:
    joint_tf = JointTf(make_model("prismatic"))

    edges = joint_tf.static_transforms()

    assert [(one.frame_id, one.child_frame_id) for one in edges] == [("arm", "hand")], (
        "a prismatic joint needs a state; a weld does not"
    )


def test_frame_names_are_the_caller_s_to_choose(make_model) -> None:
    joint_tf = JointTf(make_model(), frame=lambda name: f"left/{name}")
    sink = _Sink()

    joint_tf.add_joint_state(JointState(ts=1.0, name=["shoulder"], position=[0.1]), sink)

    assert sink.edges == [("left/base", "left/arm"), ("left/arm", "left/hand")], (
        "three robots in one world need three sets of frame names"
    )


def test_an_edge_that_has_not_moved_is_not_sent_again(make_model) -> None:
    joint_tf = JointTf(make_model(), unchanged_tf_hz=0.001, joint_tf_hz=1000.0)
    sink = _Sink()
    state = JointState(ts=1.0, name=["shoulder"], position=[0.1])

    joint_tf.add_joint_state(state, sink)
    joint_tf.add_joint_state(state, sink)

    assert len(sink.sent) == 2, (
        "both edges once; neither moved the second time and neither is stale"
    )


def test_a_stale_edge_is_sent_again_even_though_it_never_moved(make_model) -> None:
    joint_tf = JointTf(make_model(), unchanged_tf_hz=1000.0, joint_tf_hz=1000.0)
    sink = _Sink()
    state = JointState(ts=1.0, name=["shoulder"], position=[0.1])

    joint_tf.add_joint_state(state, sink)
    time.sleep(0.01)
    joint_tf.add_joint_state(state, sink)

    assert len(sink.sent) == 4, "a tf lookup needs a sample inside its tolerance, moved or not"


def test_states_faster_than_the_rate_are_dropped(make_model) -> None:
    joint_tf = JointTf(make_model(), joint_tf_hz=0.001)
    sink = _Sink()

    joint_tf.add_joint_state(JointState(ts=1.0, name=["shoulder"], position=[0.1]), sink)
    joint_tf.add_joint_state(JointState(ts=1.1, name=["shoulder"], position=[0.2]), sink)

    assert [one.ts for one in sink.sent] == [1.0, 1.0], "the second came in under the period"


def test_a_started_joint_tf_publishes_what_arrives_on_the_stream(make_model) -> None:
    """The whole point: a module hands over its usd, a joint stream and a tf stream, and stops."""
    stream, sink = _Stream(), _Sink()
    joint_tf = JointTf(make_model("prismatic"), stream, sink)
    joint_tf.start()
    try:
        stream.publish(JointState(ts=2.0, name=["shoulder"], position=[0.4]))
        _wait_for(lambda: ("base", "arm") in sink.edges, "the moving edge to go out")
    finally:
        joint_tf.stop()

    moved = next(one for one in sink.sent if one.child_frame_id == "arm")
    np.testing.assert_allclose(moved.translation.to_tuple(), (0.4, 0.0, 0.5), atol=1e-6)
    assert stream.unsubscribed == 1, "stop() lets go of the joint stream"


def test_the_fixed_edges_go_out_with_no_joint_state_at_all(make_model) -> None:
    sink = _Sink()
    joint_tf = JointTf(make_model("prismatic"), None, sink, unchanged_tf_hz=100.0)
    joint_tf.start()
    try:
        _wait_for(lambda: ("arm", "hand") in sink.edges, "the weld to go out on its own")
    finally:
        joint_tf.stop()

    assert ("base", "arm") not in sink.edges, "posing a prismatic joint at zero would be a lie"


def test_starting_with_nowhere_to_publish_says_so(make_model) -> None:
    with pytest.raises(ValueError, match="nowhere to publish"):
        JointTf(make_model()).start()


def test_a_rate_of_zero_is_refused(make_model) -> None:
    with pytest.raises(ValueError, match="above zero"):
        JointTf(make_model(), joint_tf_hz=0.0)


def test_a_mimic_joint_follows_its_leader(tmp_path: Path) -> None:
    urdf = tmp_path / "hand.urdf"
    urdf.write_text(
        """<?xml version="1.0"?>
<robot name="gripper">
  <link name="base"/>
  <link name="left"/>
  <link name="right"/>
  <joint name="left_finger" type="prismatic">
    <parent link="base"/><child link="left"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
  </joint>
  <joint name="right_finger" type="prismatic">
    <parent link="base"/><child link="right"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <mimic joint="left_finger" multiplier="-1.0" offset="0.0"/>
  </joint>
</robot>
"""
    )
    joint_tf = JointTf(RobotModel.from_file(urdf))
    sink = _Sink()

    joint_tf.add_joint_state(JointState(ts=1.0, name=["left_finger"], position=[0.02]), sink)

    right = next(one for one in sink.sent if one.child_frame_id == "right")
    np.testing.assert_allclose(right.translation.to_tuple(), (-0.02, 0.0, 0.0), atol=1e-6)


def test_nothing_to_say_and_nobody_to_hear_gets_no_thread(tmp_path: Path) -> None:
    """A manager standing on a bare world link has no joints and no controller."""
    urdf = tmp_path / "bare.urdf"
    urdf.write_text('<?xml version="1.0"?><robot name="world"><link name="world"/></robot>')
    joint_tf = JointTf(RobotModel.from_file(urdf), None, _Sink())

    joint_tf.start()
    try:
        assert joint_tf.static_transforms() == []
        assert not any(one.name == "JointTf" for one in threading.enumerate())
    finally:
        joint_tf.stop()
