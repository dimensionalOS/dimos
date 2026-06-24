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

"""The unrefined PGO — a frozen standalone snapshot of main's cmu_nav PGO
(GTSAM iSAM2 + PCL ICP C++ binary, the original gsc_pgo was refined from),
adapted to the LoopClosure spec without touching the binary.

The cpp/ directory here is a copy of `cmu_nav/modules/pgo/cpp` at the time
of the snapshot, so later cmu_nav changes don't silently move this baseline.

Spec adaptations, all Python-side:
  * `lidar` — the binary expects a `registered_scan` topic; `_collect_topics`
    aliases the lidar port's topic onto that arg name.
  * `pose_graph` — the binary doesn't expose its internal graph, only the
    current map->odom offset (`pgo_tf`) and corrected odometry. The wrapper
    keyframes the RAW odometry stream (same delta gates as the binary) and
    re-applies the LATEST offset to every keyframe on each correction update.
    A single global offset can't reproduce iSAM2's per-keyframe smoothing —
    but that offset is exactly what this PGO exposes to consumers, so the
    synthesized graph is an honest picture of its output.
  * `loop_closure_event` — emitted when the offset jumps by more than the
    `_LOOP_EVENT_*` thresholds (the offset only moves materially when a loop
    closure lands)."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time

import numpy as np
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.components.loop_closure.spec import LoopClosure
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
from dimos.navigation.jnav.msgs.GraphDelta3D import GraphDelta3D
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Offset jumps below these are smoothing noise, not loop closures.
_LOOP_EVENT_MIN_TRANS_M = 0.05
_LOOP_EVENT_MIN_ROT_DEG = 1.0


class PGOConfig(NativeModuleConfig):
    cwd: str | None = str(Path(__file__).resolve().parent / "cpp")
    # Absolute so the exists() check works from any worker cwd (skips rebuild).
    executable: str = str(Path(__file__).resolve().parent / "cpp/result/bin/pgo")
    # path:$PWD makes nix see this (git-untracked) copied directory.
    build_command: str | None = 'nix build "path:$PWD#default" --no-write-lock-file'

    # Frame names
    world_frame: str = "map"
    local_frame: str = "odom"

    # Keyframe detection
    key_pose_delta_deg: float = 10.0
    key_pose_delta_trans: float = 0.5

    # Loop closure
    loop_search_radius: float = 1.0
    loop_time_thresh: float = 60.0
    loop_score_thresh: float = 0.15
    loop_submap_half_range: int = 5
    submap_resolution: float = 0.1
    min_loop_detect_duration: float = 5.0

    # Input mode: transform world-frame scans to body-frame using odom
    unregister_input: bool = True

    # Global map publishing
    global_map_voxel_size: float = 0.1
    global_map_publish_rate: float = 1.0

    debug: bool = False


@dataclass
class _RawKeyframe:
    ts: float
    translation: np.ndarray  # (3,)
    rotation: np.ndarray  # 3x3


class PGO(NativeModule, LoopClosure):
    """Pose graph optimization with loop closure using GTSAM iSAM2 + PCL ICP."""

    config: PGOConfig

    lidar: In[PointCloud2]
    odometry: In[Odometry]
    corrected_odometry: Out[Odometry]
    pose_graph: Out[Graph3D]
    loop_closure_event: Out[GraphDelta3D]
    global_map: Out[PointCloud2]
    pgo_tf: Out[Odometry]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._keyframes: list[_RawKeyframe] = []
        self._offset_rotation = np.eye(3)
        self._offset_translation = np.zeros(3)
        self._graph_lock = threading.Lock()

    def _collect_topics(self) -> dict[str, str]:
        topics = super()._collect_topics()
        # The binary asks for --registered_scan; feed it the lidar topic.
        if "lidar" in topics:
            topics["registered_scan"] = topics["lidar"]
        return topics

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.pgo_tf.transport.subscribe(self._on_tf_correction, self.pgo_tf))
        )
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._on_raw_odometry, self.odometry))
        )
        # Seed identity TF so consumers can query map->body immediately.
        self._publish_tf(
            translation=(0.0, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
            ts=time.time(),
        )
        if self.config.debug:
            logger.info("unrefined PGO native module started (C++ iSAM2 + PCL ICP)")

    @rpc
    def stop(self) -> None:
        super().stop()

    # --- TF passthrough (same as the cmu_nav wrapper) -----------------------

    def _publish_tf(
        self,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
        ts: float,
    ) -> None:
        self.tf.publish(
            Transform(
                frame_id=self.config.world_frame,
                child_frame_id=self.config.local_frame,
                translation=Vector3(*translation),
                rotation=Quaternion(*rotation),
                ts=ts,
            )
        )

    def _on_tf_correction(self, msg: Odometry) -> None:
        self._publish_tf(
            translation=(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
            rotation=(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
            ts=msg.ts or time.time(),
        )

        new_rotation = Rotation.from_quat(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        ).as_matrix()
        new_translation = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        with self._graph_lock:
            delta_trans = float(np.linalg.norm(new_translation - self._offset_translation))
            cos_theta = float(
                np.clip((np.trace(self._offset_rotation.T @ new_rotation) - 1.0) / 2.0, -1.0, 1.0)
            )
            delta_deg = math.degrees(math.acos(cos_theta))
            self._offset_rotation = new_rotation
            self._offset_translation = new_translation
            is_loop = (
                delta_trans > _LOOP_EVENT_MIN_TRANS_M or delta_deg > _LOOP_EVENT_MIN_ROT_DEG
            ) and bool(self._keyframes)
            graph_msg = self._build_graph(msg.ts) if self._keyframes else None
            event = self._build_loop_event(msg.ts) if is_loop else None

        if graph_msg is not None:
            self.pose_graph.publish(graph_msg)
        if event is not None:
            self.loop_closure_event.publish(event)

    # --- synthesized pose graph (the binary doesn't expose its own) ----------

    def _on_raw_odometry(self, msg: Odometry) -> None:
        rotation = Rotation.from_quat(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        ).as_matrix()
        translation = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        with self._graph_lock:
            if not self._is_keyframe(rotation, translation):
                return
            self._keyframes.append(
                _RawKeyframe(ts=msg.ts, translation=translation, rotation=rotation)
            )
            graph_msg = self._build_graph(msg.ts)
        self.pose_graph.publish(graph_msg)

    def _is_keyframe(self, rotation: np.ndarray, translation: np.ndarray) -> bool:
        if not self._keyframes:
            return True
        last = self._keyframes[-1]
        delta_trans = float(np.linalg.norm(translation - last.translation))
        cos_theta = float(np.clip((np.trace(last.rotation.T @ rotation) - 1.0) / 2.0, -1.0, 1.0))
        delta_deg = math.degrees(math.acos(cos_theta))
        return (
            delta_trans > self.config.key_pose_delta_trans
            or delta_deg > self.config.key_pose_delta_deg
        )

    def _corrected_node(self, index: int, keyframe: _RawKeyframe) -> Graph3D.Node3D:
        rotation = self._offset_rotation @ keyframe.rotation
        translation = self._offset_rotation @ keyframe.translation + self._offset_translation
        quaternion = Rotation.from_matrix(rotation).as_quat()
        return Graph3D.Node3D(
            pose=PoseStamped(
                ts=keyframe.ts,
                frame_id=self.config.world_frame,
                position=[float(v) for v in translation],
                orientation=[float(v) for v in quaternion],
            ),
            id=index,
        )

    def _build_graph(self, ts: float) -> Graph3D:
        """Caller must hold ``_graph_lock``."""
        nodes = [
            self._corrected_node(index, keyframe) for index, keyframe in enumerate(self._keyframes)
        ]
        edges = [
            Graph3D.Edge(start_id=index - 1, end_id=index, timestamp=self._keyframes[index].ts)
            for index in range(1, len(self._keyframes))
        ]
        return Graph3D(ts=ts, nodes=nodes, edges=edges)

    def _build_loop_event(self, ts: float) -> GraphDelta3D:
        """Caller must hold ``_graph_lock``."""
        latest_index = len(self._keyframes) - 1
        identity = GraphDelta3D.Transform(
            translation=Vector3(0.0, 0.0, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
        )
        return GraphDelta3D(
            ts=ts,
            nodes=[self._corrected_node(latest_index, self._keyframes[latest_index])],
            transforms=[identity],
        )
