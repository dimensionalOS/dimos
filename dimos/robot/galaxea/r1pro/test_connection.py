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

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_UPPER_BODY_JOINTS,
    ArticulatedTf,
    R1ProConnectionConfig,
)

HEAD = "camera_head_left_link"


@pytest.fixture(scope="module")
def config() -> R1ProConnectionConfig:
    return R1ProConnectionConfig()


@pytest.fixture(scope="module")
def fk(config: R1ProConnectionConfig) -> ArticulatedTf:
    return ArticulatedTf(R1PRO_MODEL, config.articulated_frame_ids, root_link=config.frame_id)


def joint_state(**positions: float) -> JointState:
    """motor_states as the connection publishes it, with named joints bent."""
    values = dict.fromkeys(R1PRO_UPPER_BODY_JOINTS, 0.0)
    for joint, angle in positions.items():
        values[f"r1pro/{joint}"] = angle
    return JointState(
        ts=1.0,
        frame_id="base_link",
        name=R1PRO_UPPER_BODY_JOINTS,
        position=list(values.values()),
    )


def test_motor_states_joint_names_all_exist_in_the_model(fk: ArticulatedTf) -> None:
    """The URDF renames every joint to `r1pro/<joint>`; motor_states must match.

    A mismatch is silent: unknown joints are skipped, FK falls back to the
    neutral pose, and the camera transform looks plausible but never moves.
    """
    fk.transforms(joint_state())

    assert fk._unknown_joints == set()


def test_head_camera_moves_when_the_torso_bends(fk: ArticulatedTf) -> None:
    """The head sits past four revolute torso joints, so it cannot be static."""
    upright = {t.child_frame_id: t.translation for t in fk.transforms(joint_state())}
    bent = {t.child_frame_id: t.translation for t in fk.transforms(joint_state(torso_joint2=0.5))}

    assert bent[HEAD].x - upright[HEAD].x == pytest.approx(0.436, abs=0.01)
    assert bent[HEAD].z - upright[HEAD].z == pytest.approx(-0.144, abs=0.01)


def test_transforms_hang_off_the_configured_root(
    fk: ArticulatedTf, config: R1ProConnectionConfig
) -> None:
    transforms = fk.transforms(joint_state())

    assert {t.frame_id for t in transforms} == {config.frame_id}
    assert [t.child_frame_id for t in transforms] == list(config.articulated_frame_ids)
    assert all(t.ts == 1.0 for t in transforms)


def test_unknown_links_are_rejected_at_construction() -> None:
    with pytest.raises(ValueError, match="no_such_link"):
        ArticulatedTf(R1PRO_MODEL, ("no_such_link",))


def test_the_lidar_stream_is_restamped_with_the_configured_frame():
    """The vendor driver stamps clouds `livox_frame`; nothing publishes that edge.

    Before this, the raytracing map looked up `odom -> livox_frame`, found
    nothing, and dropped every cloud — as a warning, so the run looked healthy
    while the voxel map stayed empty. The connection owns both the cloud and the
    `base_link -> lidar_frame_id` edge, so it is what has to make them agree.
    """
    import queue
    import threading
    from unittest.mock import patch

    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.robot.galaxea.r1pro.connection import convert_loop

    stop = threading.Event()
    published: list[PointCloud2] = []

    class _Out:
        def publish(self, msg):
            published.append(msg)
            stop.set()

    incoming: queue.Queue = queue.Queue()
    incoming.put(object())

    cloud = PointCloud2(frame_id="livox_frame")
    with patch(
        "dimos.protocol.pubsub.impl.rospubsub_conversion.ros_to_dimos",
        return_value=cloud,
    ):
        convert_loop(
            stream="lidar",
            queue_in=incoming,
            dimos_type=PointCloud2,
            out=_Out(),
            stop=stop,
            record_decode=lambda *a, **k: None,
            frame_id="lidar_chassis_left_link",
        )

    assert len(published) == 1
    assert published[0].frame_id == "lidar_chassis_left_link"


def test_a_stream_without_a_frame_override_is_left_alone():
    import queue
    import threading
    from unittest.mock import patch

    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.robot.galaxea.r1pro.connection import convert_loop

    stop = threading.Event()
    published: list[Image] = []

    class _Out:
        def publish(self, msg):
            published.append(msg)
            stop.set()

    incoming: queue.Queue = queue.Queue()
    incoming.put(object())

    image = Image(frame_id="vendor_optical")
    with patch(
        "dimos.protocol.pubsub.impl.rospubsub_conversion.ros_to_dimos",
        return_value=image,
    ):
        convert_loop(
            stream="head_depth",
            queue_in=incoming,
            dimos_type=Image,
            out=_Out(),
            stop=stop,
            record_decode=lambda *a, **k: None,
        )

    assert published[0].frame_id == "vendor_optical"
