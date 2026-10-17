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

"""One URDF and the live joint state: every joint on tf, and the robot cut out of any point cloud wired in."""

from __future__ import annotations

import asyncio
import importlib
from pathlib import Path
from threading import RLock
import time
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.shape_msgs.MeshFile import MeshFile
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import LoadedRobotModel, RobotModel
from dimos.types.timestamped import TimestampedBufferCollection
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose

logger = setup_logger()


# Their pose comes from tf (odometry, a planar base), so they are read from tf, never published to it.
_MULTI_DOF_JOINT_TYPES = frozenset({"floating", "planar"})


class UrdfManagerConfig(ModuleConfig):
    model: RobotModel
    # Cut the robot out of `pointcloud` onto `filtered_pointcloud`; off where nothing needs a robot-free cloud.
    self_filter: bool = True
    padding_m: float = Field(default=0.01, ge=0.0)
    # How far a cloud's capture time may be from the tf and joint state it is matched with.
    tf_tolerance_s: float = Field(default=0.02, ge=0.0)
    tf_forward_tolerance_s: float = Field(default=0.05, ge=0.0)
    state_tolerance_s: float = Field(default=0.02, ge=0.0)
    state_history_s: float = Field(default=5.0, gt=0.0)
    # Every single-DOF and fixed URDF joint as a parent -> child tf edge; off where another module owns those frames.
    publish_joint_tf: bool = True
    # An edge goes out when it moves, at most this often...
    joint_tf_hz: float = Field(default=30.0, gt=0.0)
    # ...and otherwise at this keepalive rate: a tf lookup needs a sample within its time tolerance, so keep
    # 1 / unchanged_tf_hz under twice the tightest tolerance of the tf readers (0.25 s in r1pro-nav: >= 2 Hz).
    unchanged_tf_hz: float = Field(default=4.0, gt=0.0)
    # urdf_info is re-sent so a late subscriber gets it; nothing on these streams is latched.
    urdf_info_period_s: float = Field(default=5.0, gt=0.0)
    # Seconds between self-filter summary log lines (0 disables).
    filter_stats_interval_s: float = Field(default=10.0, ge=0.0)


class UrdfManager(Module):
    """Poses the URDF from the joint state: publishes it on tf, and (self_filter) drops robot returns from clouds at
    their capture time."""

    config: UrdfManagerConfig

    pointcloud: In[PointCloud2]
    # IO: the joints it poses are published on it, and a cloud's mount is read from it
    tf: IO[TFMessage]
    coordinator_joint_state: In[JointState]
    filtered_pointcloud: Out[PointCloud2]
    # The URDF XML, its meshes named package://<package>/<path>; any of those names on mesh_request gets the file on mesh_info.
    urdf_info: Out[String]
    mesh_request: In[String]
    mesh_info: Out[MeshFile]

    _running: bool = False

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._filter_lock = RLock()
        self._states = TimestampedBufferCollection[JointState](self.config.state_history_s)
        self._last_capture: float | None = None
        self._load_scene()
        self._next_joint_tf_t = 0.0
        # Per edge: the last transform sent (translation + rotation) and when.
        self._sent_edges: dict[tuple[str, str], tuple[tuple[float, ...], float]] = {}
        self._clouds_in = self._clouds_dropped = self._points_in = self._points_removed = 0
        self._next_stats_t = time.monotonic() + self.config.filter_stats_interval_s

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - Initialize TF before binding input handlers.
        super().start()
        self._running = True
        self.spawn(self._publish_urdf_loop())

    @rpc
    def stop(self) -> None:
        self._running = False
        super().stop()

    async def _publish_urdf_loop(self) -> None:
        while self._running:
            self.urdf_info.publish(String(self._urdf_xml))
            await asyncio.sleep(self.config.urdf_info_period_s)

    async def handle_mesh_request(self, request: String) -> None:
        # Only meshes the URDF names: a request is never a path into the filesystem.
        path = self._mesh_paths.get(request.data)
        if path is None:
            logger.warning(f"mesh_request for {request.data!r}, which is not a mesh in urdf_info")
            return
        self.mesh_info.publish(MeshFile(request.data, await asyncio.to_thread(path.read_bytes)))

    async def handle_pointcloud(self, cloud: PointCloud2) -> None:
        """Filter the latest capture without starving TF transport callbacks."""
        if self.config.self_filter:
            await asyncio.to_thread(self._on_pointcloud, cloud)

    async def handle_coordinator_joint_state(self, state: JointState) -> None:
        await asyncio.to_thread(self.add_joint_state, state)

    def add_joint_state(self, state: JointState) -> None:
        """Buffer the full model state (out-of-order arrivals stay timestamped) and publish its joints on tf."""
        with self._filter_lock:
            if np.isfinite(state.ts):
                self._states.remove_by_timestamp(state.ts)
                self._states.add(JointState(state))
                latest = self._states.last()
                if latest is not None:
                    self._states.prune_old(latest.ts - self.config.state_history_s)
        if not self.config.publish_joint_tf or time.monotonic() < self._next_joint_tf_t:
            return
        self._next_joint_tf_t = time.monotonic() + 1.0 / self.config.joint_tf_hz
        # The scene's FK state is shared with the cloud filter's thread.
        with self._filter_lock:
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
        with self._filter_lock:
            if not np.isfinite(cloud.ts) or (
                self._last_capture is not None and cloud.ts < self._last_capture
            ):
                logger.warning("Dropping cloud: invalid or out-of-order capture timestamp")
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
            self._last_capture = cloud.ts
            return filtered

    def _on_pointcloud(self, cloud: PointCloud2) -> None:
        with self._filter_lock:
            filtered = self.filter_cloud(cloud)
            if filtered is not None:
                self.filtered_pointcloud.publish(filtered)

    def joint_transforms(self, state: JointState) -> list[Transform]:
        """One parent -> child transform per URDF joint; joints the state leaves out stay at zero."""
        positions = dict(zip(state.name, state.position, strict=False))
        q = self._neutral_q.copy()
        for name in self._scene.getJointNames():
            info = self._scene.getJointInfo(name)
            if info.num_velocity_dofs == 1 and name in positions:
                value = positions[name]
                values = [np.cos(value), np.sin(value)] if info.num_position_dofs == 2 else [value]
                q[self._scene.getJointPositionIndices([name])] = values
        # Fixed joints are republished with the moving ones: a chained lookup is only as fresh as its stalest edge.
        return [
            Transform.from_matrix(
                np.asarray(self._context.forwardKinematics(q, joint.child_link, joint.parent_link)),
                frame_id=joint.parent_link,
                child_frame_id=joint.child_link,
                ts=state.ts,
            )
            for joint in self._joints.values()
            if joint.type not in _MULTI_DOF_JOINT_TYPES
        ]

    def _compute_keep_mask(
        self, cloud: PointCloud2, points: NDArray[np.float32]
    ) -> NDArray[np.bool_] | None:
        """One keep flag per point (input order), or None when the capture-time state or tf is missing."""
        base_from_sensor = self.tfbuffer.get(
            self._base_link,
            cloud.frame_id,
            time_point=cloud.ts,
            time_tolerance=self.config.tf_tolerance_s,
            forward_tolerance=self.config.tf_forward_tolerance_s,
        )
        q = self._capture_configuration(cloud.ts)
        if base_from_sensor is None or q is None:
            logger.warning("Dropping cloud: capture-time robot state or TF unavailable")
            return None
        # This Scene is rooted at the URDF base, so its world is the base frame.
        transform = base_from_sensor.to_matrix()
        base_points = np.asarray(points @ transform[:3, :3].T + transform[:3, 3], dtype=np.float64)
        return ~np.asarray(self._body_filter.computeMask(q, base_points), dtype=bool)

    def _capture_configuration(self, stamp: float) -> NDArray[np.float64] | None:
        state = self._states.find_closest(stamp, self.config.state_tolerance_s)
        positions: dict[str, float] = {}
        if state is not None:
            if len(state.name) != len(state.position) or len(set(state.name)) != len(state.name):
                return None
            positions = dict(zip(state.name, state.position, strict=True))
            if not np.isfinite(list(positions.values())).all():
                return None
        # Start from the model's neutral configuration, never its latest state.
        q = self._neutral_q.copy()
        for name in self._scene.getJointNames():
            info = self._scene.getJointInfo(name)
            if info.num_velocity_dofs == 1:
                if name not in positions:
                    return None
                value = positions[name]
                values = [np.cos(value), np.sin(value)] if info.num_position_dofs == 2 else [value]
            else:
                # Multi-DOF joints have no scalar JointState representation.
                # Recover their configuration from capture-time relative TF.
                joint = self._joints[name]
                parent_from_child = self.tfbuffer.get(
                    joint.parent_link,
                    joint.child_link,
                    time_point=stamp,
                    time_tolerance=self.config.tf_tolerance_s,
                    forward_tolerance=self.config.tf_forward_tolerance_s,
                )
                if parent_from_child is None:
                    return None
                origin = np.asarray(
                    self._context.forwardKinematics(
                        self._neutral_q, joint.child_link, joint.parent_link
                    )
                )
                motion = np.linalg.inv(origin) @ parent_from_child.to_matrix()
                pose = matrix_to_pose(motion)
                if info.num_position_dofs == 4:
                    if not np.allclose(motion[2], [0, 0, 1, 0], atol=1e-6):
                        return None
                    angle = np.arctan2(motion[1, 0], motion[0, 0])
                    values = [motion[0, 3], motion[1, 3], np.cos(angle), np.sin(angle)]
                elif info.num_position_dofs == 7:
                    values = [
                        *motion[:3, 3],
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ]
                else:
                    raise ValueError(f"Unsupported robot joint layout: {name}")
            q[self._scene.getJointPositionIndices([name])] = values
        return q

    def _load_scene(self) -> None:
        # RoboPlan is optional for stacks that do not use this module.
        native = importlib.import_module("roboplan.core")

        description = self.config.model.load()
        root = ET.fromstring(description.xml)
        root.set("version", "1.0")
        self._scene = native.Scene(
            "dimos_self_filter",
            native.loadUrdfSceneDescriptionFromXml(
                ET.tostring(root, encoding="unicode"),
                [str(path) for path in description.package_paths.values()],
            ),
        )
        self._neutral_q = np.asarray(self._scene.getCurrentJointPositions()).copy()
        self._context = native.SceneContext(self._scene)
        self._body_filter = native.RobotBodyFilter(
            self._scene,
            native.RobotBodyFilterOptions(
                padding=self.config.padding_m,
                method=native.RobotBodyFilterMethod.Narrowphase,
                num_threads=1,
            ),
        )
        self._urdf_xml, self._mesh_paths = _shareable_urdf(description)
        # Reuse the asset loader's topology for multi-DOF TF lookup, not geometry parsing.
        self._base_link = description.root_link
        self._joints = {joint.name: joint for joint in description.joints}


def _shareable_urdf(description: LoadedRobotModel) -> tuple[str, dict[str, Path]]:
    """The URDF with each mesh's local path swapped for package://<package>/<path>, and those names' files."""
    root = ET.fromstring(description.xml)
    mesh_paths: dict[str, Path] = {}
    for mesh in root.iter("mesh"):
        path = Path(mesh.attrib["filename"])
        uri = str(path)
        for package, package_root in description.package_paths.items():
            if path.is_relative_to(package_root):
                uri = f"package://{package}/{path.relative_to(package_root).as_posix()}"
                break
        mesh.set("filename", uri)
        mesh_paths[uri] = path
    return ET.tostring(root, encoding="unicode"), mesh_paths
