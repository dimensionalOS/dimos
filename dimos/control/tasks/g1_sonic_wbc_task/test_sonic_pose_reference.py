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
import rerun as rr

from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import WRIST_ONNX_INDICES
from dimos.msgs.helpers import resolve_msg_type
from dimos.msgs.visualization_msgs.SonicPoseReference import (
    CURRENT_BONES_PATH,
    CURRENT_JOINTS_PATH,
    LEFT_WRIST_AXES_PATH,
    PREVIOUS_BONES_PATH,
    PREVIOUS_JOINTS_PATH,
    RIGHT_WRIST_AXES_PATH,
    ROOT_AXES_PATH,
    SMPL_PARENTS,
    SonicPoseReference,
    _bone_segments,
    _orientation_axes,
)


def _fields() -> dict[str, np.ndarray]:
    joints = np.arange(2 * 24 * 3, dtype=np.float32).reshape(2, 24, 3)
    joint_pos = np.zeros((2, 29), dtype=np.float32)
    joint_pos[1, WRIST_ONNX_INDICES] = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    return {
        "frame_index": np.array([7, 8], dtype=np.int64),
        "smpl_joints": joints,
        "body_quat_w": np.array([[1.0, 0.0, 0.0, 0.0]] * 2, dtype=np.float32),
        "joint_pos": joint_pos,
    }


def _reference(fields: dict[str, np.ndarray]) -> SonicPoseReference:
    return SonicPoseReference.from_arrays(
        frame_indices=fields["frame_index"],
        smpl_joints=fields["smpl_joints"],
        body_quat_w=fields["body_quat_w"],
        wrist_joint_pos=fields["joint_pos"][:, WRIST_ONNX_INDICES],
    )


def test_reference_copies_exact_sonic_fields() -> None:
    fields = _fields()

    reference = _reference(fields)
    fields["smpl_joints"][:] = -1.0

    assert reference.active is True
    assert reference.frame_indices.tolist() == [7, 8]
    assert reference.smpl_joints[1, 23].tolist() == [141.0, 142.0, 143.0]
    np.testing.assert_allclose(reference.wrist_joint_pos[1], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])


def test_reference_round_trips_through_typed_lcm_payload() -> None:
    reference = _reference(_fields())

    decoded = SonicPoseReference.lcm_decode(reference.lcm_encode())

    assert resolve_msg_type(SonicPoseReference.msg_name) is SonicPoseReference
    assert decoded.active is True
    np.testing.assert_array_equal(decoded.frame_indices, reference.frame_indices)
    np.testing.assert_array_equal(decoded.smpl_joints, reference.smpl_joints)
    np.testing.assert_array_equal(decoded.body_quat_w, reference.body_quat_w)
    np.testing.assert_array_equal(decoded.wrist_joint_pos, reference.wrist_joint_pos)


def test_bone_segments_follow_smpl_parent_graph() -> None:
    joints = _fields()["smpl_joints"][1]

    segments = _bone_segments(joints)

    assert segments.shape == (23, 2, 3)
    np.testing.assert_array_equal(segments[0], [joints[0], joints[1]])
    np.testing.assert_array_equal(segments[-1], [joints[SMPL_PARENTS[23]], joints[23]])


def test_orientation_axes_use_wxyz_quaternion() -> None:
    origin = np.array([1.0, 2.0, 3.0], dtype=np.float32)

    origins, vectors = _orientation_axes(
        origin,
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
    )

    np.testing.assert_array_equal(origins, np.repeat(origin[None, :], 3, axis=0))
    np.testing.assert_allclose(vectors, np.eye(3) * 0.18, atol=1e-7)


def test_to_rerun_creates_independently_toggleable_reference_subtree() -> None:
    entities = dict(_reference(_fields()).to_rerun())

    assert set(entities) == {
        CURRENT_BONES_PATH,
        CURRENT_JOINTS_PATH,
        PREVIOUS_BONES_PATH,
        PREVIOUS_JOINTS_PATH,
        ROOT_AXES_PATH,
        LEFT_WRIST_AXES_PATH,
        RIGHT_WRIST_AXES_PATH,
    }
    assert isinstance(entities[CURRENT_BONES_PATH], rr.LineStrips3D)
    assert isinstance(entities[CURRENT_JOINTS_PATH], rr.Points3D)
    assert isinstance(entities[ROOT_AXES_PATH], rr.Arrows3D)


def test_inactive_reference_clears_every_entity() -> None:
    active_paths = [path for path, _ in _reference(_fields()).to_rerun()]
    cleared_paths = [path for path, _ in SonicPoseReference.clear().to_rerun()]

    assert cleared_paths == active_paths
