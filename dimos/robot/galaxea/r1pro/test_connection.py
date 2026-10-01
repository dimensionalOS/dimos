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

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

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
    return ArticulatedTf(R1PRO_MODEL, root_link=config.frame_id)


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


def pose_in_root(transforms: list, link: str) -> tuple[np.ndarray, Rotation]:
    """Compose the published edges from the root down to `link`."""
    edges = {t.child_frame_id: t for t in transforms}
    position, rotation = np.zeros(3), Rotation.identity()
    while link in edges:
        t = edges[link]
        q = t.rotation
        local = Rotation.from_quat([q.x, q.y, q.z, q.w])
        position = local.apply(position) + np.array(
            [t.translation.x, t.translation.y, t.translation.z]
        )
        rotation = local * rotation
        link = t.frame_id
    return position, rotation


def test_motor_states_joint_names_all_exist_in_the_model(fk: ArticulatedTf) -> None:
    """A mismatch is silent: unknown joints are skipped and the camera never moves."""
    fk.transforms(joint_state())

    assert fk._unknown_joints == set()


def test_head_camera_moves_when_the_torso_bends(fk: ArticulatedTf) -> None:
    """The head sits past four revolute torso joints, so it cannot be static."""
    upright, _ = pose_in_root(fk.transforms(joint_state()), HEAD)
    bent, _ = pose_in_root(fk.transforms(joint_state(torso_joint2=0.5)), HEAD)

    assert bent[0] - upright[0] == pytest.approx(0.436, abs=0.01)
    assert bent[2] - upright[2] == pytest.approx(-0.144, abs=0.01)


def test_the_head_moves_through_torso_edges_not_its_camera_edges(fk: ArticulatedTf) -> None:
    """Only joint edges change with the torso; head_link -> camera stays a fixed mount."""

    def edges(state: JointState) -> dict[str, tuple[float, ...]]:
        return {
            t.child_frame_id: (
                *t.translation,
                t.rotation.x,
                t.rotation.y,
                t.rotation.z,
                t.rotation.w,
            )
            for t in fk.transforms(state)
        }

    upright, bent = edges(joint_state()), edges(joint_state(torso_joint2=0.5))

    assert bent[HEAD] == pytest.approx(upright[HEAD])
    assert bent["torso_link2"] != pytest.approx(upright["torso_link2"])


def test_transforms_hang_off_the_configured_root(
    fk: ArticulatedTf, config: R1ProConnectionConfig
) -> None:
    transforms = fk.transforms(joint_state())
    children = [t.child_frame_id for t in transforms]
    parents = {t.frame_id for t in transforms}

    assert len(children) == len(set(children)), "a link with two parents breaks the tree"
    assert parents - set(children) == {config.frame_id}
    assert {HEAD, "head_link", "camera_head_right_link"} <= set(children)
    assert all(t.ts == 1.0 for t in transforms)


def test_the_lidar_stream_is_restamped_with_the_configured_frame():
    """The vendor stamps clouds `livox_frame`, which no tf edge reaches."""
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
