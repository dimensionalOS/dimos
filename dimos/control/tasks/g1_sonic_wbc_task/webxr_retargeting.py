# Copyright 2025-2026 Dimensional Inc.
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

"""Tracked WebXR body-skeleton conversion for SONIC SMPL mode."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Final, cast

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.control.tasks.g1_sonic_wbc_task.nvidia_smpl import (
    retarget_nvidia_smpl,
    retarget_nvidia_smpl_wrist_targets,
)
from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import (
    NUM_JOINTS,
    SONIC_V1_1_PIPELINE,
    WRIST_ONNX_INDICES,
    SonicTeleopPipeline,
    sonic_model_profile,
)
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot

# Standard SMPL body order plus the terminal hand points used by SONIC.
SMPL_WEBXR_JOINTS: Final[tuple[str, ...]] = (
    "hips",
    "left-upper-leg",
    "right-upper-leg",
    "spine-lower",
    "left-lower-leg",
    "right-lower-leg",
    "spine-middle",
    "left-foot-ankle",
    "right-foot-ankle",
    "spine-upper",
    "left-foot-ball",
    "right-foot-ball",
    "neck",
    "left-shoulder",
    "right-shoulder",
    "head",
    "left-arm-upper",
    "right-arm-upper",
    "left-arm-lower",
    "right-arm-lower",
    "left-hand-wrist",
    "right-hand-wrist",
    "left-hand-palm",
    "right-hand-palm",
)

POSE_TARGET_FPS: Final[float] = 50.0
POSE_MAX_GAP_SECONDS: Final[float] = 0.15


class IncompleteBodyPoseError(ValueError):
    """A WebXR snapshot cannot produce a complete SONIC reference."""


class PoseStreamError(ValueError):
    """Tracked pose timing cannot extend the current SONIC stream."""


class PoseStreamGapError(PoseStreamError):
    """A capture gap started a fresh pose window."""


@dataclass(frozen=True)
class RetargetedSonicFrame:
    """One or more packed-message-equivalent SONIC protocol-v3 frames."""

    fields: dict[str, NDArray[Any]]


class WebXRSonicRetargeter:
    """Convert complete WebXR snapshots into NVIDIA's canonical SMPL frames."""

    @staticmethod
    def missing_joints(snapshot: BodyTrackingSnapshot) -> tuple[str, ...]:
        joints = snapshot.joints
        if joints is None:
            return SMPL_WEBXR_JOINTS
        return tuple(name for name in SMPL_WEBXR_JOINTS if name not in joints)

    @classmethod
    def is_complete(cls, snapshot: BodyTrackingSnapshot) -> bool:
        return not cls.missing_joints(snapshot)

    def retarget(
        self,
        snapshot: BodyTrackingSnapshot,
        *,
        frame_index: int,
    ) -> RetargetedSonicFrame:
        missing = self.missing_joints(snapshot)
        if missing:
            raise IncompleteBodyPoseError(f"missing WebXR body joints: {', '.join(missing)}")
        assert snapshot.joints is not None

        global_quaternions = np.empty((24, 4), dtype=np.float64)
        for index, name in enumerate(SMPL_WEBXR_JOINTS):
            pose = snapshot.joints[name]
            position = np.asarray(pose.position, dtype=np.float64)
            quaternion = np.asarray(pose.orientation, dtype=np.float64)
            norm = float(np.linalg.norm(quaternion))
            if not np.isfinite(position).all() or not np.isfinite(quaternion).all() or norm < 1e-8:
                raise IncompleteBodyPoseError(f"invalid WebXR body pose for {name!r}")
            global_quaternions[index] = quaternion / norm

        nvidia_pose = retarget_nvidia_smpl(global_quaternions)
        joint_pos = np.zeros(NUM_JOINTS, dtype=np.float32)
        joint_pos[WRIST_ONNX_INDICES] = nvidia_pose.wrist_joint_pos
        joint_vel = np.zeros(NUM_JOINTS, dtype=np.float32)
        fields: dict[str, NDArray[Any]] = {
            "frame_index": np.array([frame_index], dtype=np.int64),
            "joint_pos": joint_pos.reshape(1, NUM_JOINTS),
            "joint_vel": joint_vel.reshape(1, NUM_JOINTS),
            "body_quat_w": nvidia_pose.body_quat_w.reshape(1, 4),
            "smpl_joints": nvidia_pose.smpl_joints.reshape(1, 24, 3),
            "smpl_pose": nvidia_pose.smpl_pose.reshape(1, 21, 3),
        }
        return RetargetedSonicFrame(fields=fields)


def _interpolate_rotvecs(
    left: NDArray[np.float32], right: NDArray[np.float32], alpha: float
) -> NDArray[np.float32]:
    left_rotation = Rotation.from_rotvec(left.reshape(-1, 3))
    right_rotation = Rotation.from_rotvec(right.reshape(-1, 3))
    delta = left_rotation.inv() * right_rotation
    result = left_rotation * Rotation.from_rotvec(delta.as_rotvec() * alpha)
    return cast("NDArray[np.float32]", result.as_rotvec().astype(np.float32).reshape(left.shape))


def _interpolate_quaternion_wxyz(
    left: NDArray[np.float32], right: NDArray[np.float32], alpha: float
) -> NDArray[np.float32]:
    right_value = right.copy()
    if float(np.dot(left, right_value)) < 0.0:
        right_value *= -1.0
    result = (1.0 - alpha) * left + alpha * right_value
    norm = float(np.linalg.norm(result))
    if norm < 1e-8:
        raise PoseStreamError("interpolated root quaternion is invalid")
    return (result / norm).astype(np.float32)


class WebXRSonicPoseStream:
    """Resample WebXR poses to a selected rolling 50 Hz SONIC stream."""

    def __init__(
        self,
        sonic_pipeline: SonicTeleopPipeline = SONIC_V1_1_PIPELINE,
    ) -> None:
        self._sonic_pipeline = sonic_pipeline
        self._window_frames = sonic_model_profile(sonic_pipeline).smpl_frames
        self._retargeter = WebXRSonicRetargeter()
        self._frames: deque[dict[str, NDArray[Any]]] = deque(maxlen=self._window_frames)
        self._previous_time: float | None = None
        self._previous_fields: dict[str, NDArray[Any]] | None = None
        self._next_target_time: float | None = None
        self._next_frame_index = 0
        self._generation = 0

    @property
    def buffered_frames(self) -> int:
        return len(self._frames)

    @property
    def sonic_pipeline(self) -> SonicTeleopPipeline:
        return self._sonic_pipeline

    @property
    def window_frames(self) -> int:
        return self._window_frames

    @property
    def ready(self) -> bool:
        return len(self._frames) == self._window_frames

    @property
    def generation(self) -> int:
        return self._generation

    def reset(self) -> None:
        self._frames.clear()
        self._previous_time = None
        self._previous_fields = None
        self._next_target_time = None
        self._next_frame_index = 0
        self._generation = 0

    def push(self, snapshot: BodyTrackingSnapshot) -> int:
        current = self._retargeter.retarget(snapshot, frame_index=0).fields
        capture_time = float(snapshot.capture_time_s)
        if not np.isfinite(capture_time):
            raise PoseStreamError("body capture time is invalid")

        if self._previous_time is None or self._previous_fields is None:
            self._prime(capture_time, current)
            return 0

        delta = capture_time - self._previous_time
        if delta <= 0.0:
            self.reset()
            self._prime(capture_time, current)
            raise PoseStreamError("body capture time did not increase")
        if delta > POSE_MAX_GAP_SECONDS:
            self.reset()
            self._prime(capture_time, current)
            raise PoseStreamGapError("body capture time gap exceeded 150 ms")

        assert self._next_target_time is not None
        emitted = 0
        step = 1.0 / POSE_TARGET_FPS
        while self._next_target_time <= capture_time + 1e-9:
            alpha = (self._next_target_time - self._previous_time) / delta
            alpha = min(1.0, max(0.0, alpha))
            self._frames.append(self._interpolate(self._previous_fields, current, alpha))
            self._next_frame_index += 1
            self._next_target_time += step
            self._generation += 1
            emitted += 1

        self._previous_time = capture_time
        self._previous_fields = current
        return emitted

    def fields(self) -> dict[str, NDArray[Any]]:
        if not self.ready:
            raise PoseStreamError(
                f"pose stream needs {self._window_frames} frames, has {len(self._frames)}"
            )
        keys = self._frames[0].keys()
        return {key: np.concatenate([frame[key] for frame in self._frames], axis=0) for key in keys}

    def _prime(self, capture_time: float, fields: dict[str, NDArray[Any]]) -> None:
        self._previous_time = capture_time
        self._previous_fields = fields
        self._next_target_time = capture_time

    def _interpolate(
        self,
        left: dict[str, NDArray[Any]],
        right: dict[str, NDArray[Any]],
        alpha: float,
    ) -> dict[str, NDArray[Any]]:
        pose = _interpolate_rotvecs(left["smpl_pose"], right["smpl_pose"], alpha)
        joint_pos = np.zeros((1, NUM_JOINTS), dtype=np.float32)
        joint_pos[0, WRIST_ONNX_INDICES] = retarget_nvidia_smpl_wrist_targets(pose[0])
        return {
            "frame_index": np.array([self._next_frame_index], dtype=np.int64),
            "joint_pos": joint_pos,
            "joint_vel": np.zeros((1, NUM_JOINTS), dtype=np.float32),
            "body_quat_w": _interpolate_quaternion_wxyz(
                left["body_quat_w"][0], right["body_quat_w"][0], alpha
            ).reshape(1, 4),
            "smpl_joints": (
                (1.0 - alpha) * left["smpl_joints"] + alpha * right["smpl_joints"]
            ).astype(np.float32),
            "smpl_pose": pose,
        }
