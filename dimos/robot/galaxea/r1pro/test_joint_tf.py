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

from dimos.message_utils import time as message_time
from dimos_messages.sensor_msgs.msg import JointState
from dimos_messages.std_msgs.msg import Header
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
from dimos.robot.galaxea.r1pro.connection import R1PRO_UPPER_BODY_JOINTS
from dimos.robot.urdf_manager import UrdfManager

HEAD = "camera_head_left_link"


@pytest.fixture(scope="module")
def fk() -> UrdfManager:
    return UrdfManager(model=R1PRO_MODEL)


def joint_state(**positions: float) -> JointState:
    """motor_states as the connection publishes it, with named joints bent."""
    values = dict.fromkeys(R1PRO_UPPER_BODY_JOINTS, 0.0)
    for joint, angle in positions.items():
        values[f"r1pro/{joint}"] = angle
    positions_array = np.asarray(list(values.values()), dtype=np.float64)
    return JointState(
        header=Header(stamp=message_time.time_from_seconds(1.0), frame_id="base_link"),
        name=R1PRO_UPPER_BODY_JOINTS,
        position=positions_array,
        velocity=np.zeros(len(positions_array), dtype=np.float64),
        effort=np.zeros(0, dtype=np.float64),
    )


def pose_in_root(transforms: list, link: str) -> tuple[np.ndarray, Rotation]:
    """Compose the published edges from the root down to `link`."""
    edges = {transform.child_frame_id: transform for transform in transforms}
    position, rotation = np.zeros(3), Rotation.identity()
    while link in edges:
        edge = edges[link]
        quaternion = edge.transform.rotation
        local = Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        position = local.apply(position) + np.array(
            [
                edge.transform.translation.x,
                edge.transform.translation.y,
                edge.transform.translation.z,
            ]
        )
        rotation = local * rotation
        link = edge.header.frame_id
    return position, rotation


def test_motor_states_joint_names_all_exist_in_the_model(fk: UrdfManager) -> None:
    """A mismatch is silent: unknown joints are skipped and the camera never moves."""
    assert set(R1PRO_UPPER_BODY_JOINTS) <= set(fk._joints)


def test_head_camera_moves_when_the_torso_bends(fk: UrdfManager) -> None:
    """The head sits past four revolute torso joints, so it cannot be static."""
    upright, _ = pose_in_root(fk.joint_transforms(joint_state()), HEAD)
    bent, _ = pose_in_root(fk.joint_transforms(joint_state(torso_joint2=0.5)), HEAD)

    assert bent[0] - upright[0] == pytest.approx(0.436, abs=0.01)
    assert bent[2] - upright[2] == pytest.approx(-0.144, abs=0.01)


def test_the_head_moves_through_torso_edges_not_its_camera_edges(fk: UrdfManager) -> None:
    """Only joint edges change with the torso; head_link -> camera stays a fixed mount."""

    def edges(state: JointState) -> dict[str, tuple[float, ...]]:
        return {
            transform.child_frame_id: (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            for transform in fk.joint_transforms(state)
        }

    upright, bent = edges(joint_state()), edges(joint_state(torso_joint2=0.5))

    assert bent[HEAD] == pytest.approx(upright[HEAD])
    assert bent["torso_link2"] != pytest.approx(upright["torso_link2"])


def test_transforms_hang_off_base_link(fk: UrdfManager) -> None:
    transforms = fk.joint_transforms(joint_state())
    children = [transform.child_frame_id for transform in transforms]
    parents = {transform.header.frame_id for transform in transforms}

    assert len(children) == len(set(children)), "a link with two parents breaks the tree"
    assert parents - set(children) == {"base_link"}
    assert {HEAD, "head_link", "camera_head_right_link"} <= set(children)
    assert all(message_time.to_seconds(transform.header.stamp) == 1.0 for transform in transforms)
