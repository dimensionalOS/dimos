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

"""Vibe-adapted from the pure-modules branch to conform the spec.py"""

from __future__ import annotations

from collections.abc import AsyncGenerator
from typing import Any

import numpy as np

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.loop_closure.pgo import (
    PGOConfig,
    _PGOState,
    _transform_to_pose3,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
from dimos.navigation.jnav.msgs.GraphDelta3D import GraphDelta3D


class BaselinePGOConfig(ModuleConfig):
    world_frame: str = "map"
    body_frame: str = "base_link"

    # PGOConfig knobs (mirrors dimos/mapping/loop_closure/pgo.py::PGOConfig).
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


class BaselinePGO(Module):
    """Online ISAM2+ICP PGO (`_PGOState`) wired to the eval harness streams."""

    config: BaselinePGOConfig

    cloud: In[PointCloud2]
    odometry: In[Odometry]
    corrected_odometry: Out[Odometry]
    pose_graph: Out[Graph3D]
    loop_closure_event: Out[GraphDelta3D]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._pgo: _PGOState | None = None
        self._latest_odom: Transform | None = None
        self._loops_emitted = 0

    @property
    def pgo(self) -> _PGOState:
        if self._pgo is None:
            fields = PGOConfig.model_fields if hasattr(PGOConfig, "model_fields") else {}
            cfg = PGOConfig(
                **{
                    name: getattr(self.config, name)
                    for name in fields
                    if hasattr(self.config, name)
                }
            )
            self._pgo = _PGOState(cfg)
        return self._pgo

    async def main(self) -> AsyncGenerator[None, None]:
        yield

    async def handle_odometry(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._latest_odom = Transform(
            translation=Vector3(pose.position.x, pose.position.y, pose.position.z),
            rotation=Quaternion(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ),
            frame_id=self.config.world_frame,
            child_frame_id=self.config.body_frame,
            ts=msg.ts,
        )

    async def handle_cloud(self, msg: PointCloud2) -> None:
        odom = self._latest_odom
        # No odom yet, or a placeholder pose (zero translation / uninitialized
        # quaternion) — ack with a pass-through so the lockstep replay advances.
        if odom is None or odom.translation.is_zero() or odom.rotation.is_zero():
            self._publish_corrected(odom, msg.ts)
            return

        kfs_before = len(self.pgo._key_poses)
        loops_before = len(self.pgo._accepted_loops)
        world_cloud = msg.transform(odom)  # sensor-frame scan -> world (process unregisters it)
        self.pgo.process(_transform_to_pose3(odom), msg.ts, world_cloud)

        self._publish_corrected(odom, msg.ts)
        if len(self.pgo._key_poses) > kfs_before or len(self.pgo._accepted_loops) > loops_before:
            self._publish_pose_graph(msg.ts)
        self._emit_new_loops(msg.ts)

    def _publish_corrected(self, odom: Transform | None, ts: float) -> None:
        """Ack each scan with the drift-corrected current pose (world_correction ∘ odom)."""
        if odom is None:
            corrected = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        else:
            optimized = self.pgo._world_correction.compose(_transform_to_pose3(odom))
            t = np.asarray(optimized.translation())
            q = Quaternion.from_rotation_matrix(optimized.rotation().matrix())
            corrected = Pose(float(t[0]), float(t[1]), float(t[2]), q.x, q.y, q.z, q.w)
        self.corrected_odometry.publish(
            Odometry(
                ts=ts,
                frame_id=self.config.world_frame,
                child_frame_id=self.config.body_frame,
                pose=corrected,
            )
        )

    def _publish_pose_graph(self, ts: float) -> None:
        nodes: list[Graph3D.Node3D] = []
        for idx, key_pose in enumerate(self.pgo._key_poses):
            t = np.asarray(key_pose.optimized.translation())
            q = Quaternion.from_rotation_matrix(key_pose.optimized.rotation().matrix())
            nodes.append(
                Graph3D.Node3D(
                    pose=PoseStamped(
                        ts=key_pose.timestamp,
                        frame_id=self.config.world_frame,
                        position=Vector3(float(t[0]), float(t[1]), float(t[2])),
                        orientation=q,
                    ),
                    id=idx,
                )
            )
        self.pose_graph.publish(Graph3D(ts=ts, nodes=nodes))

    def _emit_new_loops(self, ts: float) -> None:
        """One GraphDelta3D per newly accepted loop (GraphCapture counts these)."""
        accepted = self.pgo._accepted_loops
        while self._loops_emitted < len(accepted):
            pair = accepted[self._loops_emitted]
            endpoints: list[Graph3D.Node3D] = []
            deltas: list[GraphDelta3D.Transform] = []
            for kf_idx in (pair.source, pair.target):
                key_pose = self.pgo._key_poses[kf_idx]
                t = np.asarray(key_pose.optimized.translation())
                q = Quaternion.from_rotation_matrix(key_pose.optimized.rotation().matrix())
                endpoints.append(
                    Graph3D.Node3D(
                        pose=PoseStamped(
                            ts=key_pose.timestamp,
                            frame_id=self.config.world_frame,
                            position=Vector3(float(t[0]), float(t[1]), float(t[2])),
                            orientation=q,
                        ),
                        id=kf_idx,
                    )
                )
                deltas.append(
                    GraphDelta3D.Transform(
                        translation=Vector3(0.0, 0.0, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
                    )
                )
            self.loop_closure_event.publish(GraphDelta3D(ts=ts, nodes=endpoints, transforms=deltas))
            self._loops_emitted += 1
