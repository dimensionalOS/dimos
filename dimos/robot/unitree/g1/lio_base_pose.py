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

"""Convert the G1's Mid360 LIO pose into a live pelvis pose.

Point-LIO estimates the moving lidar body. The physical Mid360 is mounted
upside down, while the G1 URDF describes an upright ``mid360_link`` attached
through the moving waist. This module makes both transforms explicit and
publishes the canonical ``world -> pelvis`` pose used by manipulation.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import threading
from typing import Any

import numpy as np
from reactivex.disposable import Disposable
import yourdfpy  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.g1.config import G1
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

WAIST_JOINTS = ("waist_yaw", "waist_roll", "waist_pitch")
# Raw Point-LIO uses the physical IMU axes. The URDF link is upright, so the
# fixed physical-sensor -> modeled-link correction is a 180 degree roll.
MID360_MOUNT_CORRECTION = np.diag([1.0, -1.0, -1.0])


@dataclass(frozen=True)
class G1LioPoseStatus:
    waist_ready: bool
    published_samples: int
    rejected_samples: int
    last_lio_ts: float | None
    last_pose: PoseStamped | None
    message: str


class G1LioBasePoseConfig(ModuleConfig):
    world_frame: str = "world"
    sensor_frame: str = "mid360_link"
    base_frame: str = "pelvis"


def pelvis_to_sensor_from_urdf(
    urdf: Any,
    joint_positions: Mapping[str, float],
    *,
    sensor_frame: str = "mid360_link",
    base_frame: str = "pelvis",
) -> np.ndarray:
    """Return ``base_frame -> sensor_frame`` at the supplied waist state."""
    cfg = np.zeros(len(urdf.actuated_joint_names))
    for index, urdf_name in enumerate(urdf.actuated_joint_names):
        short_name = urdf_name.removesuffix("_joint")
        cfg[index] = joint_positions.get(short_name, 0.0)
    urdf.update_cfg(cfg)
    return np.asarray(urdf.get_transform(sensor_frame, base_frame), dtype=np.float64)


def lio_odometry_to_base_pose(
    odometry: Odometry,
    pelvis_to_sensor: np.ndarray,
    *,
    world_frame: str = "world",
    mount_correction: np.ndarray = MID360_MOUNT_CORRECTION,
) -> PoseStamped:
    """Resolve a raw physical-sensor LIO pose into the modeled pelvis pose."""
    if pelvis_to_sensor.shape != (4, 4):
        raise ValueError(f"pelvis_to_sensor must be 4x4, got {pelvis_to_sensor.shape}")
    if mount_correction.shape != (3, 3):
        raise ValueError(f"mount_correction must be 3x3, got {mount_correction.shape}")

    world_to_sensor = np.eye(4)
    world_to_sensor[:3, :3] = odometry.orientation.to_rotation_matrix() @ mount_correction
    world_to_sensor[:3, 3] = np.asarray(odometry.position.as_tuple, dtype=np.float64)
    world_to_pelvis = world_to_sensor @ np.linalg.inv(pelvis_to_sensor)
    orientation = Quaternion.from_rotation_matrix(world_to_pelvis[:3, :3])
    return PoseStamped(
        ts=odometry.ts,
        frame_id=world_frame,
        position=world_to_pelvis[:3, 3],
        orientation=orientation,
    )


class G1LioBasePose(Module):
    """Publish ``world -> pelvis`` only after live waist state is available."""

    config: G1LioBasePoseConfig

    odometry: In[Odometry]
    coordinator_joint_state: In[JointState]
    base_pose: Out[PoseStamped]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._urdf: Any = None
        self._waist: dict[str, float] = {}
        self._pelvis_to_sensor: np.ndarray | None = None
        self._published_samples = 0
        self._rejected_samples = 0
        self._last_lio_ts: float | None = None
        self._last_pose: PoseStamped | None = None
        self._message = "Waiting for all live waist joints"

    @rpc
    def start(self) -> None:
        super().start()
        self._urdf = yourdfpy.URDF.load(str(G1.model_path), load_meshes=False)
        subscriptions = (
            self.coordinator_joint_state.subscribe(self._on_joint_state),
            self.odometry.subscribe(self._on_odometry),
        )
        for subscription in subscriptions:
            self.register_disposable(
                Disposable(subscription) if callable(subscription) else subscription
            )

    @rpc
    def get_status(self) -> G1LioPoseStatus:
        """Return frame readiness and the latest observer-only pelvis pose."""
        with self._lock:
            return G1LioPoseStatus(
                waist_ready=self._pelvis_to_sensor is not None,
                published_samples=self._published_samples,
                rejected_samples=self._rejected_samples,
                last_lio_ts=self._last_lio_ts,
                last_pose=self._last_pose,
                message=self._message,
            )

    def _on_joint_state(self, joint_state: JointState) -> None:
        updates: dict[str, float] = {}
        for name, position in zip(joint_state.name, joint_state.position, strict=False):
            short_name = name.removeprefix("g1/").removesuffix("_joint")
            if short_name in WAIST_JOINTS:
                updates[short_name] = float(position)
        if not updates:
            return

        with self._lock:
            self._waist.update(updates)
            if not all(name in self._waist for name in WAIST_JOINTS):
                missing = sorted(set(WAIST_JOINTS) - self._waist.keys())
                self._message = f"Waiting for waist joints: {', '.join(missing)}"
                return
            assert self._urdf is not None
            self._pelvis_to_sensor = pelvis_to_sensor_from_urdf(
                self._urdf,
                self._waist,
                sensor_frame=self.config.sensor_frame,
                base_frame=self.config.base_frame,
            )
            self._message = "Publishing live world -> pelvis pose"

    def _on_odometry(self, odometry: Odometry) -> None:
        with self._lock:
            self._last_lio_ts = float(odometry.ts)
            if odometry.frame_id != self.config.world_frame:
                self._rejected_samples += 1
                self._message = (
                    f"Rejected LIO frame '{odometry.frame_id}', expected "
                    f"'{self.config.world_frame}'"
                )
                return
            if odometry.child_frame_id != self.config.sensor_frame:
                self._rejected_samples += 1
                self._message = (
                    f"Rejected LIO child '{odometry.child_frame_id}', expected "
                    f"'{self.config.sensor_frame}'"
                )
                return
            if self._pelvis_to_sensor is None:
                self._rejected_samples += 1
                pelvis_to_sensor = None
            else:
                pelvis_to_sensor = self._pelvis_to_sensor.copy()

        if pelvis_to_sensor is None:
            return

        pose = lio_odometry_to_base_pose(
            odometry,
            pelvis_to_sensor,
            world_frame=self.config.world_frame,
        )
        transform = Transform(
            translation=Vector3(pose.position),
            rotation=Quaternion(pose.orientation),
            frame_id=self.config.world_frame,
            child_frame_id=self.config.base_frame,
            ts=pose.ts,
        )
        self.base_pose.publish(pose)
        self.tf.publish(TFMessage(transform))
        with self._lock:
            self._last_pose = pose
            self._published_samples += 1
            self._message = "Publishing live world -> pelvis pose"
