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


"""Optional SDK adapter. Shared policy runtime owns rollout and execution."""

from __future__ import annotations

from copy import deepcopy
from importlib import import_module
from threading import RLock
import time
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.control.tasks.eef_trajectory_task.trajectory import EefTrajectory, EefTrajectoryPoint
from dimos.imitation.policy.dreamlabs.config import DreamlabsBackendConfig, yam_arms
from dimos.imitation.policy.dreamlabs.constants import (
    ACTION_HZ,
    HORIZON,
    OBSERVATION_PERIOD,
    OBSERVATION_WINDOW,
)
from dimos.imitation.policy.dreamlabs.conversion import encode_image, joint_actions, policy_config
from dimos.imitation.policy.dreamlabs.kinematics import DreamlabsKinematics
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS
from dimos.robot.manipulators.dual_openyam.model import DUAL_OPENYAM_MODEL
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DreamlabsPolicyConfig(Protocol):
    dreamlabs: DreamlabsBackendConfig
    joint_names: list[str]
    policy_joint_names: list[str] | None
    policy_path: str
    robot_type: str
    fps: float | None
    execution_steps: int | None


def create_client() -> Any:
    # The isolated runtime is the only place that imports the optional SDK.
    return import_module("dream").DreamClient()


class DreamlabsBackend:
    n_action_steps = HORIZON
    chunk_size: int | None = HORIZON
    fps = ACTION_HZ
    action_lower: NDArray[np.float32] | None = None
    action_upper: NDArray[np.float32] | None = None
    observation_period: float | None = OBSERVATION_PERIOD
    observations_required = OBSERVATION_WINDOW

    def __init__(self, config: DreamlabsPolicyConfig) -> None:
        if config.fps is not None and config.fps != ACTION_HZ:
            raise ValueError("Dreamlabs requires fps=20")
        if config.execution_steps is not None and config.execution_steps != HORIZON:
            raise ValueError("Dreamlabs requires execution_steps=48")
        self.config = config.dreamlabs
        self._checkpoint = config.policy_path
        self._joint_names = config.joint_names
        required = {
            name
            for arm in self.config.arms.values()
            for name in (*arm.joint_names, arm.gripper_joint)
        }
        if len(set(self._joint_names)) != len(self._joint_names) or not required <= set(
            self._joint_names
        ):
            raise ValueError(
                "Policy state must contain all uniquely named configured arm and gripper joints"
            )
        if self.config.mode == "joint" and (
            config.robot_type not in {"yam", "dual_openyam"}
            or self.config.model != DUAL_OPENYAM_MODEL
            or self.config.arms != yam_arms()
            or set(self._joint_names) != set(DUAL_OPENYAM_JOINTS)
        ):
            raise ValueError(
                "Dreamlabs joint mode requires the canonical dual YAM model and named joints"
            )
        self._fk = DreamlabsKinematics(self.config)
        self._lock = RLock()
        self._request_lock = RLock()
        self._client: Any = None
        self._latest: dict[str, float] | None = None
        self._task: str | None = None
        self._last_timestamp: float | None = None
        self._clock_offset = time.monotonic() - time.time()
        self._last_action_response: dict[str, Any] | None = None
        self._closed = False

    def _positions(self, state: NDArray[np.float32]) -> dict[str, float]:
        if state.shape != (len(self._joint_names),) or not np.isfinite(state).all():
            raise ValueError("Dreamlabs requires finite state matching configured joint names")
        positions = dict(zip(self._joint_names, map(float, state), strict=True))
        for arm in self.config.arms.values():
            if not 0.0 <= positions[arm.gripper_joint] <= 1.0:
                raise ValueError("Dreamlabs state grippers must be normalized to [0, 1]")
        return positions

    def observe(
        self,
        images: dict[str, NDArray[np.uint8]],
        state: NDArray[np.float32],
        *,
        task: str,
        timestamp: float,
    ) -> None:
        if not task.strip() or not np.isfinite(timestamp):
            raise ValueError("Dreamlabs requires a task and finite capture timestamp")
        positions = self._positions(state)
        if set(images) != set(self.config.camera_roles):
            raise ValueError(f"Dreamlabs requires camera roles {self.config.camera_roles}")
        frames = {
            f"video.{role}": encode_image(Image(data=images[role], format=ImageFormat.RGB))
            for role in self.config.camera_roles
        }
        with self._lock:
            if self._closed:
                raise RuntimeError("Dreamlabs backend is closed")
            if self._last_timestamp is not None and timestamp <= self._last_timestamp:
                raise ValueError("Dreamlabs requires a newer capture timestamp")
            if self._task is not None and task != self._task:
                raise RuntimeError("Reset the Dreamlabs episode before changing its task")
            if self._client is None:
                client = create_client()
                try:
                    client.policy(
                        policy_config(
                            self._checkpoint,
                            self.config.timeout_ms,
                            self.config.mode,
                            self.config.camera_roles,
                        )
                    )
                except Exception:
                    client.close()
                    raise
                self._client = client
            self._client.set_observation(
                frames,
                self._fk.observation(positions, task),
                timestamp=timestamp + self._clock_offset,
            )
            self._latest = positions
            self._last_timestamp = timestamp
            self._task = task

    def predict(
        self, images: dict[str, NDArray[np.uint8]], state: NDArray[np.float32], *, task: str
    ) -> NDArray[np.float32] | EefTrajectory:
        del images
        anchor = self._positions(state)
        with self._request_lock:
            with self._lock:
                if self._closed or self._client is None or self._latest is None:
                    raise RuntimeError("Fresh observations must precede Dreamlabs inference")
                if task != self._task:
                    raise RuntimeError("Reset the Dreamlabs episode before changing its task")
                client = self._client
            response = client.get_action()
            with self._lock:
                self._last_action_response = deepcopy(response)
                # Capture the freshest feedback available after the request, while
                # the SDK's absolute EEF answers retain their original frame.
                anchor = dict(self._latest) if self._latest is not None else anchor
            if self.config.mode == "joint":
                rows = joint_actions(response, self._fk.joint_bounds)
                return rows[:, [DUAL_OPENYAM_JOINTS.index(name) for name in self._joint_names]]
            return self._eef_trajectory(response, anchor)

    def _eef_trajectory(self, response: dict[str, Any], anchor: dict[str, float]) -> EefTrajectory:
        parts = response.get("per_key")
        if not isinstance(parts, dict):
            raise ValueError("Dreamlabs EEF reply requires per_key")
        poses: dict[str, NDArray[np.float64]] = {}
        grips: dict[str, NDArray[np.float64]] = {}
        for side in self.config.arms:
            xyz = np.asarray(parts.get(f"action.{side}_eef_xyz"), dtype=np.float64)
            rot6 = np.asarray(parts.get(f"action.{side}_eef_rot6"), dtype=np.float64)
            grip = np.asarray(parts.get(f"action.{side}_grip"), dtype=np.float64)
            if grip.shape == (HORIZON,):
                grip = grip[:, None]
            if (
                xyz.shape != (HORIZON, 3)
                or rot6.shape != (HORIZON, 6)
                or grip.shape != (HORIZON, 1)
            ):
                raise ValueError(
                    "Dreamlabs EEF reply requires 48 xyz/rot6/grip rows of widths 3/6/1"
                )
            if not all(np.isfinite(values).all() for values in (xyz, rot6, grip)):
                raise ValueError("Dreamlabs EEF reply contains non-finite values")
            matrices = np.repeat(np.eye(4)[None, :, :], HORIZON, axis=0)
            first = rot6[:, :3]
            norm = np.linalg.norm(first, axis=1, keepdims=True)
            if np.any(norm < 1e-8):
                raise ValueError("Dreamlabs EEF reply has degenerate rotation columns")
            first = first / norm
            second = rot6[:, 3:] - np.sum(first * rot6[:, 3:], axis=1, keepdims=True) * first
            norm = np.linalg.norm(second, axis=1, keepdims=True)
            if np.any(norm < 1e-8):
                raise ValueError("Dreamlabs EEF reply has degenerate rotation columns")
            second = second / norm
            matrices[:, :3, :3] = np.stack((first, second, np.cross(first, second)), axis=2)
            matrices[:, :3, 3] = xyz
            poses[side] = matrices
            invalid = np.flatnonzero((grip[:, 0] < 0) | (grip[:, 0] > 1))
            if invalid.size:
                logger.warning(
                    "Clipping finite Dreamlabs EEF gripper output to [0, 1]",
                    arm=side,
                    indices=invalid.tolist(),
                    raw_values=grip[invalid, 0].tolist(),
                )
            grips[side] = np.clip(grip[:, 0], 0.0, 1.0)
        points = [
            EefTrajectoryPoint(
                time_from_start=0.0,
                poses={
                    arm.target_frame: self._fk.to_pose(self._fk.root_pose(arm.target_frame, anchor))
                    for arm in self.config.arms.values()
                },
                grippers={
                    arm.gripper_joint: anchor[arm.gripper_joint]
                    for arm in self.config.arms.values()
                },
            )
        ]
        points.extend(
            EefTrajectoryPoint(
                time_from_start=(index + 1) / self.fps,
                poses={
                    arm.target_frame: self._fk.canonical_pose(side, poses[side][index])
                    for side, arm in self.config.arms.items()
                },
                grippers={
                    arm.gripper_joint: float(grips[side][index])
                    for side, arm in self.config.arms.items()
                },
            )
            for index in range(HORIZON)
        )
        return EefTrajectory(points=points)

    def get_last_action_response(self) -> dict[str, Any] | None:
        with self._lock:
            return deepcopy(self._last_action_response)

    def reset(self) -> None:
        with self._request_lock, self._lock:
            if self._client is not None:
                self._client.reset_episode()
            self._latest = None
            self._task = None
            self._last_timestamp = None

    def close(self) -> None:
        with self._request_lock, self._lock:
            if self._client is not None:
                self._client.close()
                self._client = None
            self._latest = None
            self._closed = True
