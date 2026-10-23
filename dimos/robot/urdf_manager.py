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

"""One URDF and the live joint state: every joint on tf, the URDF and its files for renderers, and (UrdfManagerPointClearing)
the robot cut out of any point cloud wired in."""

from __future__ import annotations

import asyncio
from collections.abc import Mapping
from pathlib import Path
from threading import RLock
import time
from typing import Protocol
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.files.MeshFile import MeshFile
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import RobotModel
from dimos.types.timestamped import TimestampedBufferCollection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Their pose comes from tf (odometry, a planar base), so they are read from tf, never published to it.
_MULTI_DOF_JOINT_TYPES = frozenset({"floating", "planar"})


class UrdfManagerConfig(ModuleConfig):
    model: RobotModel
    # Every single-DOF and fixed URDF joint as a parent -> child tf edge; off where another module owns those frames.
    publish_joint_tf: bool = True
    # An edge goes out when it moves, at most this often...
    joint_tf_hz: float = Field(default=30.0, gt=0.0)
    # ...and otherwise at this keepalive rate: a tf lookup needs a sample within its time tolerance, so keep
    # 1 / unchanged_tf_hz under twice the tightest tolerance of the tf readers (0.25 s in r1pro-nav: >= 2 Hz).
    unchanged_tf_hz: float = Field(default=4.0, gt=0.0)


class UrdfManager(Module):
    """Poses the URDF from the joint state onto tf, and serves the URDF and the files it names to renderers."""

    config: UrdfManagerConfig

    # IO: the joints it poses are published on it (and UrdfManagerPointClearing reads a cloud's mount from it)
    tf: IO[TFMessage]
    coordinator_joint_state: In[JointState]
    # A request for a *.urdf gets the URDF; one for a file the URDF names (mesh, texture, splat, ...) by its full path
    # gets that file. Either goes out on file_info named as requested.
    render_file_request: In[String]
    file_info: Out[MeshFile]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._description = self.config.model.load()
        self._file_paths = _files_named_by(self._description.xml)
        self._joints = {joint.name: joint for joint in self._description.joints}
        self._next_joint_tf_t = 0.0
        # Per edge: the last transform sent (translation + rotation) and when.
        self._sent_edges: dict[tuple[str, str], tuple[tuple[float, ...], float]] = {}

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - Initialize TF before binding input handlers.
        super().start()

    async def handle_render_file_request(self, request: String) -> None:
        name = request.data
        if name.endswith(".urdf"):
            self.file_info.publish(MeshFile(name, self._description.xml.encode()))
            return
        # Only files the URDF names: a request is never an arbitrary path into the filesystem.
        if name not in self._file_paths:
            logger.warning(f"render_file_request for {name!r}, which is not a file the URDF names")
            return
        data = await asyncio.to_thread(Path(name).read_bytes)
        self.file_info.publish(MeshFile(name, data))

    async def handle_coordinator_joint_state(self, state: JointState) -> None:
        await asyncio.to_thread(self.add_joint_state, state)

    def add_joint_state(self, state: JointState) -> None:
        """Publish the state's joints on tf."""
        if not self.config.publish_joint_tf or time.monotonic() < self._next_joint_tf_t:
            return
        self._next_joint_tf_t = time.monotonic() + 1.0 / self.config.joint_tf_hz
        with self._lock:
            transforms = self._edges_to_send(self.joint_transforms(state))
        if transforms:
            self.tfbuffer.publish(*transforms)

    def _edges_to_send(self, transforms: list[Transform]) -> list[Transform]:
        """The edges that moved since they were last sent, and those not sent for one keepalive period."""
        now = time.monotonic()
        keepalive_s = 1.0 / self.config.unchanged_tf_hz
        due = []
        for transform in transforms:
            key = (transform.frame_id, transform.child_frame_id)
            value = (*transform.translation.to_tuple(), *transform.rotation.to_tuple())
            sent = self._sent_edges.get(key)
            if (
                sent is None
                or now - sent[1] >= keepalive_s
                or not np.allclose(value, sent[0], atol=1e-6)
            ):
                due.append(transform)
                self._sent_edges[key] = (value, now)
        return due

    def joint_transforms(self, state: JointState) -> list[Transform]:
        """One parent -> child transform per URDF joint; joints the state leaves out stay at zero."""
        positions = dict(zip(state.name, state.position, strict=False))
        for joint in self._joints.values():
            if joint.mimic is not None and joint.mimic[0] in positions:
                leader, multiplier, offset = joint.mimic
                positions[joint.name] = multiplier * positions[leader] + offset
        # Fixed joints are republished with the moving ones: a chained lookup is only as fresh as its stalest edge.
        return [
            Transform.from_matrix(
                joint.parent_from_child(positions.get(joint.name, 0.0)),
                frame_id=joint.parent_link,
                child_frame_id=joint.child_link,
                ts=state.ts,
            )
            for joint in self._joints.values()
            if joint.type not in _MULTI_DOF_JOINT_TYPES
        ]


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
        # RoboPlan is the one backend today, imported only here so a plain UrdfManager runs without it.
        from dimos.robot.roboplan_body_filter import RoboPlanBodyFilter

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
            if joint.type in _MULTI_DOF_JOINT_TYPES:
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


def _files_named_by(xml: str) -> set[str]:
    """The full path of every file the URDF names (mesh, texture, splat, ...): any filename attribute."""
    return {
        element.attrib["filename"]
        for element in ET.fromstring(xml).iter()
        if "filename" in element.attrib
    }
