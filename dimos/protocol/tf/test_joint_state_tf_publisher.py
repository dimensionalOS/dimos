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

import math
from pathlib import Path
import time

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.tf.joint_state_tf_publisher import JointStateTfPublisher
from dimos.robot.assets.model import RobotModel

# Camera 0.2 m ahead of a torso pitch joint 1.0 m above the base, so a quarter
# turn swings it from straight ahead to straight down by exactly the offset.
URDF = """
<robot name="pitch_and_camera">
  <link name="base_link"/>
  <link name="torso_link"/>
  <link name="camera_link"/>
  <joint name="torso_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="camera_mount" type="fixed">
    <parent link="torso_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


class Bus:
    """In-process stand-in for a Transport, so `In` ports can be fed directly."""

    def __init__(self) -> None:
        self.subscribers: list = []

    def subscribe(self, callback, stream=None):
        self.subscribers.append(callback)
        return lambda: self.subscribers.remove(callback)

    def publish(self, msg) -> None:
        for callback in list(self.subscribers):
            callback(msg)

    def stop(self) -> None:
        pass


@pytest.fixture
def model(tmp_path: Path) -> RobotModel:
    urdf = tmp_path / "pitch_and_camera.urdf"
    urdf.write_text(URDF)
    return RobotModel.from_file(urdf)


@pytest.fixture
def module():
    built = []

    def build(**config):
        instance = JointStateTfPublisher(**config)
        instance.joint_states.transport = Bus()
        built.append(instance)
        return instance

    yield build
    for instance in built:
        instance.dispose()


def joint_state(pitch: float, ts: float = 1.0, name: str = "torso_pitch") -> JointState:
    return JointState(name=[name], position=[pitch], ts=ts)


def settle(messages: list, expected: int) -> list:
    """`In.observable()` is backpressured onto a worker thread, so delivery is async."""
    deadline = time.monotonic() + 5.0
    while len(messages) < expected and time.monotonic() < deadline:
        time.sleep(0.01)
    time.sleep(0.05)
    return messages


def run(instance: JointStateTfPublisher, *states: JointState) -> list:
    messages: list = []
    instance.tf.subscribe(messages.append)
    instance.start()
    for state in states:
        instance.joint_states.transport.publish(state)
        # Backpressure is latest-wins, so a burst would coalesce and look like
        # throttling; space them out so every message really reaches the module.
        time.sleep(0.05)
    return settle(messages, 1)


def test_runs_forward_kinematics_on_the_live_joint_angles(model, module):
    instance = module(model=model, links=("camera_link",))
    messages = run(instance, joint_state(math.pi / 2))

    transform = messages[0].transforms[0]
    assert transform.frame_id == "base_link"
    assert transform.child_frame_id == "camera_link"
    np.testing.assert_allclose(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        [0.0, 0.0, 0.8],
        atol=1e-6,
    )
    half = math.sqrt(0.5)
    np.testing.assert_allclose(
        [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w],
        [0.0, half, 0.0, half],
        atol=1e-6,
    )


def test_a_static_mount_would_be_wrong(model, module):
    """The whole point of FK here: the camera moves when the torso moves."""
    instance = module(model=model, links=("camera_link",))
    messages = run(instance, joint_state(0.0), joint_state(math.pi / 2, ts=10.0))
    settle(messages, 2)

    first, second = messages[0].transforms[0], messages[-1].transforms[0]
    assert not np.isclose(first.translation.x, second.translation.x, atol=1e-3)


def test_joints_absent_from_the_message_stay_at_the_model_neutral(model, module):
    """Feedback naming a joint the URDF does not have must not shift the others."""
    instance = module(model=model, links=("camera_link",))
    messages = run(instance, JointState(name=["not_a_joint"], position=[1.0], ts=1.0))

    transform = messages[0].transforms[0]
    np.testing.assert_allclose(
        [transform.translation.x, transform.translation.z], [0.2, 1.0], atol=1e-6
    )


def test_root_link_override_reparents_the_edge(model, module):
    instance = module(model=model, links=("camera_link",), root_link="odom")
    messages = run(instance, joint_state(0.0))

    assert messages[0].transforms[0].frame_id == "odom"


def test_an_unmodelled_link_is_rejected_at_start(model, module):
    instance = module(model=model, links=("no_such_link",))

    with pytest.raises(ValueError, match="no_such_link"):
        instance.start()


def test_throttles_to_max_publish_hz(model, module):
    instance = module(model=model, links=("camera_link",), max_publish_hz=10.0)
    messages = run(
        instance,
        joint_state(0.0, ts=1.0),
        joint_state(0.1, ts=1.01),
        joint_state(0.2, ts=1.02),
        joint_state(0.3, ts=1.5),
    )
    time.sleep(0.3)

    assert [message.transforms[0].ts for message in messages] == [1.0, 1.5]
