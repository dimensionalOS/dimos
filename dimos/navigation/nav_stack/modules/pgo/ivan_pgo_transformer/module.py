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

"""Ivan's CURRENT PGO (dimos/mapping/loop_closure/pgo.py, main, June 2026)
forced into an online LoopClosure module.

That code ships as an offline memory2 Transformer, but its `_PGOState` core is
already incremental — one `process(pose, ts, world_cloud)` call per frame. This
wrapper brute-forces the online shape: every arriving scan is paired with the
latest odometry pose and pushed straight into `_PGOState`, then the corrected
odometry / pose graph / loop events are read back out of its (private) state.
No changes to the mapping code itself; this module owns the coupling.

`_PGOState.process` expects world-frame registered scans (it unregisters them
internally via the odom pose) — set `input_is_registered: false` for raw
body-frame scans and they'll be pre-registered here first."""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.loop_closure.pgo import (
    PGOConfig as MappingPGOConfig,
    _PGOState,
    _pose3_to_transform,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Graph3D import Graph3D
from dimos.msgs.nav_msgs.GraphDelta3D import GraphDelta3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.specs import LoopClosure
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PGOConfig(ModuleConfig):
    world_frame: str = "map"
    odom_frame: str = "odom"
    body_frame: str = "base_link"
    # World-frame registered input (fastlio); false = raw body-frame scans.
    input_is_registered: bool = True

    # Mirrors mapping/loop_closure PGOConfig — forwarded verbatim.
    key_pose_delta_trans: float = 0.5
    key_pose_delta_deg: float = 10.0
    loop_search_radius: float = 2.0
    loop_time_thresh: float = 20.0
    loop_score_thresh: float = 0.3
    loop_submap_half_range: int = 10
    min_icp_inliers: int = 10
    min_keyframes_for_loop_search: int = 10
    loop_closure_extra_iterations: int = 4
    submap_resolution: float = 0.2
    min_loop_detect_duration: float = 5.0
    max_icp_iterations: int = 50
    max_icp_correspondence_dist: float = 1.0
    odom_rot_var: float = 1e-6
    odom_trans_var_xy: float = 1e-4
    odom_trans_var_z: float = 1e-6
    loop_rot_var: float = 0.05


def _mapping_config(config: PGOConfig) -> MappingPGOConfig:
    fields = {name: getattr(config, name) for name in MappingPGOConfig.model_fields}
    return MappingPGOConfig(**fields)


def _pose3_node(index: int, pose: Any, ts: float, world_frame: str) -> Graph3D.Node3D:
    translation = np.asarray(pose.translation())
    quaternion = Rotation.from_matrix(pose.rotation().matrix()).as_quat()  # [x,y,z,w]
    return Graph3D.Node3D(
        pose=PoseStamped(
            ts=ts,
            frame_id=world_frame,
            position=[float(v) for v in translation],
            orientation=[float(v) for v in quaternion],
        ),
        id=index,
    )


class PGO(Module, LoopClosure):
    """Online wrapper over Ivan's current incremental PGO core (`_PGOState`)."""

    config: PGOConfig

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    corrected_odometry: Out[Odometry]
    pose_graph: Out[Graph3D]
    loop_closure_event: Out[GraphDelta3D]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._state: _PGOState | None = None
        self._latest_odom: Odometry | None = None
        self._published_loops = 0
        self._odom_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._unsub_odom: Any = None
        self._unsub_lidar: Any = None

    @rpc
    def start(self) -> None:
        super().start()
        self._state = _PGOState(_mapping_config(self.config))
        # Identity map -> odom so consumers querying map -> body get a result
        # before any correction exists.
        self.tf.publish(self._correction_tf(np.eye(4), time.time()))
        self._unsub_odom = self.odometry.subscribe(self._on_odom)
        self._unsub_lidar = self.registered_scan.subscribe(self._on_scan)
        logger.info("PGO (ivan transformer core) started")

    @rpc
    def stop(self) -> None:
        if self._unsub_odom is not None:
            self._unsub_odom.dispose()
        if self._unsub_lidar is not None:
            self._unsub_lidar.dispose()
        super().stop()

    def _on_odom(self, msg: Odometry) -> None:
        with self._odom_lock:
            self._latest_odom = msg

    def _on_scan(self, cloud: PointCloud2) -> None:
        import gtsam  # type: ignore[import-not-found,import-untyped]

        with self._odom_lock:
            odom = self._latest_odom
        if odom is None or len(cloud) == 0:
            return
        state = self._state
        assert state is not None

        position = odom.pose.position
        orientation = odom.pose.orientation
        local_pose = gtsam.Pose3(
            gtsam.Rot3.Quaternion(orientation.w, orientation.x, orientation.y, orientation.z),
            gtsam.Point3(position.x, position.y, position.z),
        )
        ts = odom.ts if odom.ts else time.time()

        if not self.config.input_is_registered:
            cloud = cloud.transform(
                _pose3_to_transform(
                    local_pose,
                    ts=ts,
                    frame_id=self.config.world_frame,
                    child_frame_id=self.config.body_frame,
                )
            )

        with self._state_lock:
            state.process(local_pose, ts, cloud)
            keyframe_count = len(state._key_poses)
            correction = state._world_correction
            corrected = correction.compose(local_pose)
            graph_msg = self._snapshot_graph(state, ts) if keyframe_count else None
            loop_events = self._new_loop_events(state, ts)

        self._publish_corrected_odometry(corrected, ts)
        self.tf.publish(self._correction_tf(correction.matrix(), ts))
        if graph_msg is not None:
            self.pose_graph.publish(graph_msg)
        for event in loop_events:
            self.loop_closure_event.publish(event)

    def _publish_corrected_odometry(self, pose: Any, ts: float) -> None:
        translation = np.asarray(pose.translation())
        quaternion = Rotation.from_matrix(pose.rotation().matrix()).as_quat()
        self.corrected_odometry.publish(
            Odometry(
                ts=ts,
                frame_id=self.config.world_frame,
                child_frame_id=self.config.body_frame,
                pose=Pose(
                    position=[float(v) for v in translation],
                    orientation=[float(v) for v in quaternion],
                ),
            )
        )

    def _correction_tf(self, matrix: np.ndarray, ts: float) -> Transform:
        quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        return Transform(
            frame_id=self.config.world_frame,
            child_frame_id=self.config.odom_frame,
            translation=Vector3(float(matrix[0, 3]), float(matrix[1, 3]), float(matrix[2, 3])),
            rotation=Quaternion(*[float(v) for v in quaternion]),
            ts=ts,
        )

    def _snapshot_graph(self, state: _PGOState, ts: float) -> Graph3D:
        """Optimized keyframes + odometry-chain and loop edges.

        Caller must hold ``_state_lock``."""
        world_frame = self.config.world_frame
        nodes = [
            _pose3_node(index, key_pose.optimized, key_pose.timestamp, world_frame)
            for index, key_pose in enumerate(state._key_poses)
        ]
        edges = [
            Graph3D.Edge(
                start_id=index - 1, end_id=index, timestamp=state._key_poses[index].timestamp
            )
            for index in range(1, len(state._key_poses))
        ]
        edges += [
            Graph3D.Edge(start_id=pair.target, end_id=pair.source, metadata_id=1)
            for pair in state._accepted_loops
        ]
        return Graph3D(ts=ts, nodes=nodes, edges=edges)

    def _new_loop_events(self, state: _PGOState, ts: float) -> list[GraphDelta3D]:
        """One GraphDelta3D per accepted loop not yet published.

        Caller must hold ``_state_lock``."""
        world_frame = self.config.world_frame
        identity = GraphDelta3D.Transform(
            translation=Vector3(0.0, 0.0, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
        )
        events: list[GraphDelta3D] = []
        for pair in state._accepted_loops[self._published_loops :]:
            source = state._key_poses[pair.source]
            target = state._key_poses[pair.target]
            events.append(
                GraphDelta3D(
                    ts=ts,
                    nodes=[
                        _pose3_node(pair.target, target.optimized, target.timestamp, world_frame),
                        _pose3_node(pair.source, source.optimized, source.timestamp, world_frame),
                    ],
                    transforms=[identity, identity],
                )
            )
        self._published_loops = len(state._accepted_loops)
        return events
