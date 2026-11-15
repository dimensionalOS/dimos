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

"""A UrdfManager that also cuts the robot out of a point cloud; kept apart so a plain UrdfManager never needs RoboPlan."""

from __future__ import annotations

import asyncio
from collections.abc import Mapping
import time
from typing import Protocol

import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.roboplan_body_filter import RoboPlanBodyFilter
from dimos.robot.urdf_manager import MULTI_DOF_JOINT_TYPES, UrdfManager, UrdfManagerConfig
from dimos.types.timestamped import TimestampedBufferCollection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class UrdfManagerPointClearingConfig(UrdfManagerConfig):
    padding_m: float = Field(default=0.01, ge=0.0)
    # How far a cloud's capture time may be from the tf and joint state it is matched with.
    tf_tolerance_s: float = Field(default=0.02, ge=0.0)
    tf_forward_tolerance_s: float = Field(default=0.05, ge=0.0)
    state_tolerance_s: float = Field(default=0.02, ge=0.0)
    state_history_s: float = Field(default=5.0, gt=0.0)
    # Seconds between self-filter summary log lines (0 disables).
    filter_stats_interval_s: float = Field(default=10.0, ge=0.0)


class UrdfManagerPointClearing(UrdfManager):
    """A UrdfManager that also drops the robot's own returns from `pointcloud` at their capture time."""

    config: UrdfManagerPointClearingConfig

    pointcloud: In[PointCloud2]
    filtered_pointcloud: Out[PointCloud2]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._body_filter: RobotBodyFilter = RoboPlanBodyFilter(
            self._description, self.config.padding_m
        )
        self._base_link = self._description.root_link
        self._states = TimestampedBufferCollection[JointState](self.config.state_history_s)
        self._clouds_in = self._clouds_dropped = self._points_in = self._points_removed = 0
        self._next_stats_t = time.monotonic() + self.config.filter_stats_interval_s

    async def handle_pointcloud(self, cloud: PointCloud2) -> None:
        """Filter the latest capture without starving TF transport callbacks."""
        await asyncio.to_thread(self._on_pointcloud, cloud)

    def add_joint_state(self, state: JointState) -> None:
        """Buffer the full model state (out-of-order arrivals stay timestamped) and publish its joints on tf."""
        with self._lock:
            if np.isfinite(state.ts):
                self._states.remove_by_timestamp(state.ts)
                self._states.add(JointState(state))
                latest = self._states.last()
                if latest is not None:
                    self._states.prune_old(latest.ts - self.config.state_history_s)
        super().add_joint_state(state)

    def filter_cloud(self, cloud: PointCloud2) -> PointCloud2 | None:
        """The capture with the robot's points removed, frame and point fields unchanged; None when it can't be."""
        filtered = self._filter_capture(cloud)
        self._clouds_in += 1
        if filtered is None:
            self._clouds_dropped += 1
        else:
            self._points_in += len(cloud)
            self._points_removed += len(cloud) - len(filtered)
        if self.config.filter_stats_interval_s > 0 and time.monotonic() >= self._next_stats_t:
            self._next_stats_t = time.monotonic() + self.config.filter_stats_interval_s
            logger.info(
                f"self filter: {self._clouds_in} clouds, {self._clouds_dropped} dropped for missing tf/joint state, "
                f"{self._points_removed} of {self._points_in} points were the robot"
            )
            self._clouds_in = self._clouds_dropped = self._points_in = self._points_removed = 0
        return filtered

    def _filter_capture(self, cloud: PointCloud2) -> PointCloud2 | None:
        with self._lock:
            # Sensors sharing this input interleave, so an older capture is filtered at its own time rather than dropped.
            if not np.isfinite(cloud.ts):
                logger.warning("Dropping cloud: invalid capture timestamp")
                return None
            points = cloud.points_f32()
            if not np.isfinite(points).all():
                return None
            keep = self._compute_keep_mask(cloud, points)
            if keep is None:
                return None
            filtered = PointCloud2(frame_id=cloud.frame_id, ts=cloud.ts, seq=cloud.seq)
            for name, values in cloud.pointcloud_tensor.point.items():
                filtered.pointcloud_tensor.point[name] = values[keep]
            return filtered

    def _on_pointcloud(self, cloud: PointCloud2) -> None:
        with self._lock:
            filtered = self.filter_cloud(cloud)
            if filtered is not None:
                self.filtered_pointcloud.publish(filtered)

    def _compute_keep_mask(
        self, cloud: PointCloud2, points: NDArray[np.float32]
    ) -> NDArray[np.bool_] | None:
        """One keep flag per point (input order), or None when the capture-time state or tf is missing."""
        base_from_sensor = self._tf_at(self._base_link, cloud.frame_id, cloud.ts)
        state = self._capture_state(cloud.ts)
        motions = self._multi_dof_motions(cloud.ts)
        keep = None
        if base_from_sensor is not None and state is not None and motions is not None:
            transform = base_from_sensor.to_matrix()
            base_points = np.asarray(
                points @ transform[:3, :3].T + transform[:3, 3], dtype=np.float64
            )
            keep = self._body_filter.keep_mask(base_points, state, motions)
        if keep is None:
            logger.warning("Dropping cloud: capture-time robot state or TF unavailable")
        return keep

    def _capture_state(self, stamp: float) -> dict[str, float] | None:
        """Joint positions at capture time (empty when none is close enough), or None when that state is malformed."""
        state = self._states.find_closest(stamp, self.config.state_tolerance_s)
        if state is None:
            return {}
        if len(state.name) != len(state.position) or len(set(state.name)) != len(state.name):
            return None
        positions = dict(zip(state.name, state.position, strict=True))
        if not np.isfinite(list(positions.values())).all():
            return None
        return positions

    def _multi_dof_motions(self, stamp: float) -> dict[str, NDArray[np.float64]] | None:
        """Each floating/planar joint's motion past its URDF origin, from capture-time tf (they have no scalar
        JointState); None when one is missing."""
        motions = {}
        for joint in self._joints.values():
            if joint.type in MULTI_DOF_JOINT_TYPES:
                parent_from_child = self._tf_at(joint.parent_link, joint.child_link, stamp)
                if parent_from_child is None:
                    return None
                motions[joint.name] = np.linalg.inv(joint.origin()) @ parent_from_child.to_matrix()
        return motions

    def _tf_at(self, parent: str, child: str, stamp: float) -> Transform | None:
        return self.tfbuffer.get(
            parent,
            child,
            time_point=stamp,
            time_tolerance=self.config.tf_tolerance_s,
            forward_tolerance=self.config.tf_forward_tolerance_s,
        )


class RobotBodyFilter(Protocol):
    """A self-filter backend: which points lie on the robot at a given joint state."""

    def keep_mask(
        self,
        root_points: NDArray[np.float64],
        joint_positions: Mapping[str, float],
        joint_motions: Mapping[str, NDArray[np.float64]],
    ) -> NDArray[np.bool_] | None:
        """One keep flag per point (points in the URDF root frame, input order), or None when the state lacks a
        joint the backend needs. joint_motions holds each floating/planar joint's 4x4 motion past its URDF origin."""
        ...
