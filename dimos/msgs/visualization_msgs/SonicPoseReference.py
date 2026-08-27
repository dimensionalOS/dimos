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

"""Rerun diagnostic payload for pose chunks accepted by SONIC."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype

SMPL_PARENTS = (
    -1,
    0,
    0,
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    9,
    9,
    12,
    13,
    14,
    16,
    17,
    18,
    19,
    20,
    21,
)

REFERENCE_ROOT = "world/sonic_reference"
CURRENT_BONES_PATH = f"{REFERENCE_ROOT}/current/bones"
CURRENT_JOINTS_PATH = f"{REFERENCE_ROOT}/current/joints"
PREVIOUS_BONES_PATH = f"{REFERENCE_ROOT}/previous/bones"
PREVIOUS_JOINTS_PATH = f"{REFERENCE_ROOT}/previous/joints"
ROOT_AXES_PATH = f"{REFERENCE_ROOT}/current/root_axes"
LEFT_WRIST_AXES_PATH = f"{REFERENCE_ROOT}/current/left_wrist_axes"
RIGHT_WRIST_AXES_PATH = f"{REFERENCE_ROOT}/current/right_wrist_axes"

_CURRENT_COLOR = (0, 235, 255, 235)
_PREVIOUS_COLOR = (80, 130, 170, 70)
_AXIS_COLORS = [(255, 65, 65, 255), (65, 255, 65, 255), (65, 125, 255, 255)]
_AXIS_LENGTH = 0.18
_BONE_RADIUS = 0.018
_JOINT_RADIUS = 0.028
_WIRE_HEADER = struct.Struct(">4s?I")
_WIRE_MAGIC = b"SPR1"


def _bone_segments(joints: NDArray[np.float32]) -> NDArray[np.float32]:
    return np.asarray(
        [[joints[SMPL_PARENTS[index]], joints[index]] for index in range(1, 24)],
        dtype=np.float32,
    )


def _orientation_axes(
    origin: NDArray[np.float32], quaternion_wxyz: NDArray[np.float32]
) -> tuple[NDArray[np.float32], NDArray[np.float64]]:
    quaternion_xyzw = np.asarray([*quaternion_wxyz[1:], quaternion_wxyz[0]], dtype=np.float64)
    vectors = Rotation.from_quat(quaternion_xyzw).as_matrix() * _AXIS_LENGTH
    return np.repeat(origin[None, :], 3, axis=0), vectors.T


def _euler_axes(
    origin: NDArray[np.float32], euler_xyz: NDArray[np.float32]
) -> tuple[NDArray[np.float32], NDArray[np.float64]]:
    vectors = Rotation.from_euler("XYZ", euler_xyz).as_matrix() * _AXIS_LENGTH
    return np.repeat(origin[None, :], 3, axis=0), vectors.T


@dataclass(frozen=True)
class SonicPoseReference:
    """Exact reference chunk accepted by the SONIC pose pipeline."""

    msg_name = "visualization_msgs.SonicPoseReference"

    active: bool
    frame_indices: NDArray[np.int64]
    smpl_joints: NDArray[np.float32]
    body_quat_w: NDArray[np.float32]
    wrist_joint_pos: NDArray[np.float32]

    @classmethod
    def from_arrays(
        cls,
        *,
        frame_indices: NDArray[np.int64],
        smpl_joints: NDArray[np.float32],
        body_quat_w: NDArray[np.float32],
        wrist_joint_pos: NDArray[np.float32],
    ) -> SonicPoseReference:
        indices = np.asarray(frame_indices, dtype=np.int64).reshape(-1).copy()
        frame_count = len(indices)
        return cls(
            active=True,
            frame_indices=indices,
            smpl_joints=np.asarray(smpl_joints, dtype=np.float32)
            .reshape(frame_count, 24, 3)
            .copy(),
            body_quat_w=np.asarray(body_quat_w, dtype=np.float32).reshape(frame_count, 4).copy(),
            wrist_joint_pos=np.asarray(wrist_joint_pos, dtype=np.float32)
            .reshape(frame_count, 6)
            .copy(),
        )

    @classmethod
    def clear(cls) -> SonicPoseReference:
        return cls(
            active=False,
            frame_indices=np.empty(0, dtype=np.int64),
            smpl_joints=np.empty((0, 24, 3), dtype=np.float32),
            body_quat_w=np.empty((0, 4), dtype=np.float32),
            wrist_joint_pos=np.empty((0, 6), dtype=np.float32),
        )

    def lcm_encode(self) -> bytes:
        count = len(self.frame_indices)
        return b"".join(
            (
                _WIRE_HEADER.pack(_WIRE_MAGIC, self.active, count),
                np.asarray(self.frame_indices, dtype=">i8").tobytes(),
                np.asarray(self.smpl_joints, dtype=">f4").tobytes(),
                np.asarray(self.body_quat_w, dtype=">f4").tobytes(),
                np.asarray(self.wrist_joint_pos, dtype=">f4").tobytes(),
            )
        )

    @classmethod
    def lcm_decode(cls, data: bytes, **_: object) -> SonicPoseReference:
        if len(data) < _WIRE_HEADER.size:
            raise ValueError("SONIC pose reference payload is truncated")
        magic, active, count = _WIRE_HEADER.unpack_from(data)
        if magic != _WIRE_MAGIC:
            raise ValueError("SONIC pose reference payload has invalid magic")
        expected = _WIRE_HEADER.size + count * (8 + 24 * 3 * 4 + 4 * 4 + 6 * 4)
        if len(data) != expected:
            raise ValueError(
                f"SONIC pose reference payload has {len(data)} bytes, expected {expected}"
            )

        offset = _WIRE_HEADER.size
        indices = np.frombuffer(data, dtype=">i8", count=count, offset=offset).astype(np.int64)
        offset += count * 8
        joints = np.frombuffer(data, dtype=">f4", count=count * 24 * 3, offset=offset)
        offset += count * 24 * 3 * 4
        quaternions = np.frombuffer(data, dtype=">f4", count=count * 4, offset=offset)
        offset += count * 4 * 4
        wrists = np.frombuffer(data, dtype=">f4", count=count * 6, offset=offset)
        return cls(
            active=active,
            frame_indices=indices,
            smpl_joints=joints.astype(np.float32).reshape(count, 24, 3),
            body_quat_w=quaternions.astype(np.float32).reshape(count, 4),
            wrist_joint_pos=wrists.astype(np.float32).reshape(count, 6),
        )

    def to_rerun(self) -> list[tuple[str, Archetype]]:
        import rerun as rr

        if not self.active or len(self.frame_indices) == 0:
            return [
                (CURRENT_BONES_PATH, rr.LineStrips3D([])),
                (CURRENT_JOINTS_PATH, rr.Points3D([])),
                (PREVIOUS_BONES_PATH, rr.LineStrips3D([])),
                (PREVIOUS_JOINTS_PATH, rr.Points3D([])),
                (ROOT_AXES_PATH, rr.Arrows3D(origins=[], vectors=[])),
                (LEFT_WRIST_AXES_PATH, rr.Arrows3D(origins=[], vectors=[])),
                (RIGHT_WRIST_AXES_PATH, rr.Arrows3D(origins=[], vectors=[])),
            ]

        current_joints = self.smpl_joints[-1]
        current_bones = _bone_segments(current_joints)
        entities: list[tuple[str, Archetype]] = [
            (
                CURRENT_BONES_PATH,
                rr.LineStrips3D(
                    strips=current_bones,
                    colors=[_CURRENT_COLOR] * len(current_bones),
                    radii=[_BONE_RADIUS] * len(current_bones),
                ),
            ),
            (
                CURRENT_JOINTS_PATH,
                rr.Points3D(
                    positions=current_joints,
                    colors=[_CURRENT_COLOR],
                    radii=[_JOINT_RADIUS],
                ),
            ),
        ]

        if len(self.frame_indices) > 1:
            previous_joints = self.smpl_joints[-2]
            previous_bones = _bone_segments(previous_joints)
            entities.extend(
                [
                    (
                        PREVIOUS_BONES_PATH,
                        rr.LineStrips3D(
                            strips=previous_bones,
                            colors=[_PREVIOUS_COLOR] * len(previous_bones),
                            radii=[_BONE_RADIUS] * len(previous_bones),
                        ),
                    ),
                    (
                        PREVIOUS_JOINTS_PATH,
                        rr.Points3D(
                            positions=previous_joints,
                            colors=[_PREVIOUS_COLOR],
                            radii=[_JOINT_RADIUS],
                        ),
                    ),
                ]
            )
        else:
            entities.extend(
                [
                    (PREVIOUS_BONES_PATH, rr.LineStrips3D([])),
                    (PREVIOUS_JOINTS_PATH, rr.Points3D([])),
                ]
            )

        root_origins, root_vectors = _orientation_axes(current_joints[0], self.body_quat_w[-1])
        wrist_targets = self.wrist_joint_pos[-1]
        left_origins, left_vectors = _euler_axes(current_joints[20], wrist_targets[[0, 2, 4]])
        right_origins, right_vectors = _euler_axes(current_joints[21], wrist_targets[[1, 3, 5]])
        for path, origins, vectors in (
            (ROOT_AXES_PATH, root_origins, root_vectors),
            (LEFT_WRIST_AXES_PATH, left_origins, left_vectors),
            (RIGHT_WRIST_AXES_PATH, right_origins, right_vectors),
        ):
            entities.append(
                (
                    path,
                    rr.Arrows3D(
                        origins=origins,
                        vectors=vectors,
                        colors=_AXIS_COLORS,
                        radii=[_BONE_RADIUS] * 3,
                    ),
                )
            )
        return entities
