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

"""Publish TF from simulation poses, independently of robot model serving."""

from typing import Any

from dimos_lcm.sim_msgs import WorldState
import numpy as np
from pydantic import Field
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import UsdModel
from dimos.sim2.sensors.poses import PoseSensorConfig, _PoseSensorModule


class SimTfConfig(PoseSensorConfig):
    hz: float = Field(default=30.0, gt=0, allow_inf_nan=False)


class SimTfModule(_PoseSensorModule):
    config: SimTfConfig
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._scene: UsdModel | None = None
        self._sim_clock: str | None = None
        self._sim_epoch = -1
        self._next_sim_tf_ns = 0

    @rpc
    def start(self) -> None:
        self.tfbuffer  # noqa: B018 - Bind TF before starting the pose worker.
        super().start()

    def open_sensor(self, scene: UsdModel) -> None:
        self._scene = scene
        self._sim_clock = None
        self._sim_epoch = -1
        self._next_sim_tf_ns = 0

    def close_sensor(self) -> None:
        self._scene = None

    def acquire(self, sample: WorldState, *, latest: bool) -> None:
        """Publish the same model tree from a complete physical pose sample."""
        h = sample.header
        if self._sim_clock is not None and self._sim_clock != h.clock_id:
            raise ValueError("robot model TF clock changed; restart its binding")
        self._sim_clock = h.clock_id
        if h.epoch < self._sim_epoch:
            return
        if h.epoch > self._sim_epoch:
            self._sim_epoch, self._next_sim_tf_ns = h.epoch, 0
        if h.time_ns < self._next_sim_tf_ns:
            return
        period = round(1e9 / self.config.hz)
        self._next_sim_tf_ns = (h.time_ns // period + 1) * period
        poses = {}
        for body in sample.bodies:
            p, q = body.pose.position, body.pose.orientation
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            matrix[:3, 3] = [p.x, p.y, p.z]
            poses[body.body_id] = matrix
        edges = []
        model = self._scene
        assert model is not None
        poses["world"] = np.eye(4)
        if model.root_link not in poses:
            raise ValueError(f"missing model root in body poses: {model.root_link}")
        if model.root_link != "world":
            edges.append(
                Transform.from_matrix(
                    poses[model.root_link],
                    frame_id="world",
                    child_frame_id=model.root_link,
                    ts=sample.header.time_ns / 1e9,
                )
            )
        for joint in model.joints:
            parent_id, child_id = joint.parent_link, joint.child_link
            parent, child = poses[parent_id], poses[child_id]
            relative = np.eye(4)
            relative[:3, :3] = parent[:3, :3].T @ child[:3, :3]
            relative[:3, 3] = parent[:3, :3].T @ (child[:3, 3] - parent[:3, 3])
            edges.append(
                Transform.from_matrix(
                    relative,
                    frame_id=parent_id,
                    child_frame_id=child_id,
                    ts=sample.header.time_ns / 1e9,
                )
            )
        self.tfbuffer.publish(*edges)
        self.count("acquisitions")
