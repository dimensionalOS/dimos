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

"""NVIDIA GEAR-SONIC canonical SMPL retargeting."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.msgs.visualization_msgs.SonicPoseReference import SMPL_PARENTS

# SONIC's PICO streamer reconstructs the human with fixed SMPL rest joints
# rather than the operator's tracked limb lengths. These are the main-body and
# thumb chains from gear_sonic/data/human/human_joints_info.pkl in NVIDIA's
# GR00T-WholeBodyControl repository.
_CANONICAL_REST_JOINTS: Final[NDArray[np.float64]] = np.array(
    [
        [0.0031232606, -0.3514074683, 0.0120365508],
        [0.0613126531, -0.4441709518, -0.0139646353],
        [-0.0601442158, -0.4553154707, -0.0092138201],
        [0.0003605621, -0.2415168583, -0.0155810807],
        [0.1160081103, -0.8229243755, -0.0233606994],
        [-0.1043541729, -0.8176955581, -0.0260377023],
        [0.0098082609, -0.1096636057, -0.0215210654],
        [0.0725546628, -1.2259838581, -0.0552366450],
        [-0.0889373645, -1.2284233570, -0.0462299734],
        [-0.0015221529, -0.0574284494, 0.0069258320],
        [0.1198119670, -1.2839812040, 0.0629796833],
        [-0.1277497709, -1.2867517471, 0.0728190243],
        [-0.0136866113, 0.1077386066, -0.0246895105],
        [0.0448420048, 0.0275152735, -0.0002946509],
        [-0.0492170788, 0.0269102231, -0.0064740698],
        [0.0110968733, 0.2681904137, -0.0039522452],
        [0.1640810370, 0.0852432996, -0.0157555901],
        [-0.1517948210, 0.0804346725, -0.0191425979],
        [0.4182038903, 0.0130927814, -0.0582144447],
        [-0.4229443669, 0.0439421907, -0.0456096828],
        [0.6701906323, 0.0363140106, -0.0606865250],
        [-0.6722118258, 0.0394096449, -0.0609348677],
        [0.7108263969, 0.0183372851, -0.0350756459],
        [0.7278420925, 0.0193130989, -0.0100975055],
        [0.7483652234, 0.0141535439, 0.0054255710],
        [-0.7108249664, 0.0183352213, -0.0350735225],
        [-0.7278403044, 0.0193113182, -0.0100959428],
        [-0.7483659387, 0.0141541166, 0.0054256050],
    ],
    dtype=np.float64,
)

# Compact hierarchy for original SMPL-X joints 0:22, 37:40, and 52:55.
_CANONICAL_PARENTS: Final[tuple[int, ...]] = (
    *SMPL_PARENTS[:22],
    20,
    22,
    23,
    21,
    25,
    26,
)
_OUTPUT_JOINTS: Final[NDArray[np.intp]] = np.array(
    [*range(22), 24, 27], dtype=np.intp
)
_PICO_GLOBAL_OFFSET: Final[Rotation] = Rotation.from_euler("y", 180.0, degrees=True)
_SMPL_Y_UP_TO_Z_UP: Final[Rotation] = Rotation.from_euler("x", 90.0, degrees=True)
_SMPL_BASE_ROTATION_INVERSE: Final[Rotation] = Rotation.from_quat(
    [-0.5, -0.5, -0.5, 0.5]
)
_ELBOW_AXIS: Final[NDArray[np.float64]] = np.array([0.0, 1.0, 0.0], dtype=np.float64)


@dataclass(frozen=True)
class NvidiaSonicPose:
    """Canonical fields consumed by SONIC's SMPL encoder."""

    smpl_pose: NDArray[np.float32]
    smpl_joints: NDArray[np.float32]
    body_quat_w: NDArray[np.float32]
    wrist_joint_pos: NDArray[np.float32]


def _canonical_joints(
    root_rotation: Rotation,
    body_pose: NDArray[np.float64],
) -> NDArray[np.float64]:
    local_matrices = np.repeat(np.eye(3, dtype=np.float64)[None, :, :], 28, axis=0)
    local_matrices[0] = root_rotation.as_matrix()
    local_matrices[1:22] = Rotation.from_rotvec(body_pose).as_matrix()

    world_matrices = np.empty_like(local_matrices)
    world_positions = np.empty_like(_CANONICAL_REST_JOINTS)
    world_matrices[0] = local_matrices[0]
    world_positions[0] = _CANONICAL_REST_JOINTS[0]
    for index in range(1, len(_CANONICAL_PARENTS)):
        parent = _CANONICAL_PARENTS[index]
        world_matrices[index] = world_matrices[parent] @ local_matrices[index]
        rest_offset = _CANONICAL_REST_JOINTS[index] - _CANONICAL_REST_JOINTS[parent]
        world_positions[index] = world_positions[parent] + world_matrices[parent] @ rest_offset
    return world_positions[_OUTPUT_JOINTS]


def _elbow_swing_euler(elbow_rotvec: NDArray[np.float64]) -> NDArray[np.float64]:
    elbow = Rotation.from_rotvec(elbow_rotvec)
    quaternion = elbow.as_quat()
    twist_vector = float(np.dot(quaternion[:3], _ELBOW_AXIS)) * _ELBOW_AXIS
    twist_quaternion = np.array([*twist_vector, quaternion[3]], dtype=np.float64)
    norm = float(np.linalg.norm(twist_quaternion))
    if norm < 1e-8:
        twist = Rotation.identity()
    else:
        twist = Rotation.from_quat(twist_quaternion / norm)
    return np.asarray((twist.inv() * elbow).as_euler("XYZ"), dtype=np.float64)


def retarget_nvidia_smpl_wrist_targets(
    body_pose: NDArray[np.float64],
) -> NDArray[np.float32]:
    """Map SMPL elbow and wrist rotations to NVIDIA's six G1 wrist targets."""
    left_elbow_swing = _elbow_swing_euler(body_pose[17])
    right_elbow_swing = _elbow_swing_euler(body_pose[18])
    left_wrist = Rotation.from_rotvec(body_pose[19]).as_euler("XYZ")
    right_wrist = Rotation.from_rotvec(body_pose[20]).as_euler("XYZ")
    return np.asarray(
        [
            left_elbow_swing[0] + left_wrist[0],
            -(right_elbow_swing[0] + right_wrist[0]),
            left_wrist[1],
            -right_wrist[1],
            left_elbow_swing[2] + left_wrist[2],
            right_elbow_swing[2] + right_wrist[2],
        ],
        dtype=np.float32,
    )


def retarget_nvidia_smpl(
    global_quaternions_xyzw: NDArray[np.float64],
) -> NvidiaSonicPose:
    """Match NVIDIA's PICO-to-canonical-SMPL conversion."""
    if global_quaternions_xyzw.shape != (24, 4):
        raise ValueError(
            f"expected 24 global XYZW quaternions, got {global_quaternions_xyzw.shape}"
        )

    global_rotations = Rotation.from_quat(global_quaternions_xyzw) * _PICO_GLOBAL_OFFSET
    local_rotations: list[Rotation] = [global_rotations[0]]
    for index in range(1, 22):
        parent = SMPL_PARENTS[index]
        local_rotations.append(global_rotations[parent].inv() * global_rotations[index])

    root_y_up = local_rotations[0]
    root_z_up = _SMPL_Y_UP_TO_Z_UP * root_y_up
    body_pose = np.asarray(
        [rotation.as_rotvec() for rotation in local_rotations[1:]], dtype=np.float64
    )
    world_joints = _canonical_joints(root_z_up, body_pose)

    reference_root = root_z_up * _SMPL_BASE_ROTATION_INVERSE
    local_joints = reference_root.inv().apply(world_joints)
    root_xyzw = reference_root.as_quat()
    return NvidiaSonicPose(
        smpl_pose=body_pose.astype(np.float32),
        smpl_joints=local_joints.astype(np.float32),
        body_quat_w=np.asarray([root_xyzw[3], *root_xyzw[:3]], dtype=np.float32),
        wrist_joint_pos=retarget_nvidia_smpl_wrist_targets(body_pose),
    )
