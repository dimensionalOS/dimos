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

"""Simulated hardware owns its USD/files/mounts and publishes its own TF edges."""

from functools import cached_property
from typing import TYPE_CHECKING, Any

from dimos_lcm.sim_msgs import WorldState
import numpy as np
from pydantic import Field
from scipy.spatial.transform import Rotation

from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig, Mount
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.sim2.description import joint_paths
from dimos.sim2.sensors.sampling import Acquisition

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint


class SimHardwareConfig(HardwareModuleConfig):
    tf_hz: float = Field(default=50, gt=0)


class SimHardwareModule(HardwareModule):
    config: SimHardwareConfig
    observations: In[WorldState]

    @classmethod
    def attach(cls, mount: Mount, **kwargs: Any) -> "Blueprint":
        return super().attach(mount, **kwargs).remappings([(cls, "observations", "observations")])

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._tf_acquisition = Acquisition(self.config.tf_hz)

    def publish_static_tf(self) -> None:
        """All simulation transforms are stamped by observations, after physics starts."""

    @cached_property
    def _joint_paths(self) -> dict[str, str]:
        return {
            name: f"{self.namespace}/{path}".lstrip("/")
            for name, path in joint_paths(self.config.usd).items()
        }

    async def handle_observations(self, sample: WorldState) -> None:
        if not self._tf_acquisition.due(sample.header):
            return
        stamp = sample.header.time_ns / 1e9
        q = dict(zip(sample.joint_truth.name, sample.joint_truth.position, strict=True))
        positions = {name: q[path] for name, path in self._joint_paths.items() if path in q}
        edges = self.static_transforms(stamp) + self.moving_transforms(positions, stamp)
        poses = {"world": np.eye(4)}
        needed = {frame for edge in edges for frame in (edge.frame_id, edge.child_frame_id)}
        for body in sample.bodies:
            if body.body_id not in needed:
                continue
            p, r = body.pose.position, body.pose.orientation
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
            matrix[:3, 3] = p.x, p.y, p.z
            poses[body.body_id] = matrix
        # USD scopes and joint anchor frames need not be native physics bodies.
        # Fill those frames by FK, retaining physics poses for floating bodies.
        pending = list(edges)
        while pending:
            rest = []
            for edge in pending:
                if edge.child_frame_id in poses:
                    continue
                if edge.frame_id in poses:
                    poses[edge.child_frame_id] = poses[edge.frame_id] @ edge.to_matrix()
                else:
                    rest.append(edge)
            if len(rest) == len(pending):
                break
            pending = rest
        # Publish only the edges owned by this hardware, using physics truth.
        for i, edge in enumerate(edges):
            parent, child = poses.get(edge.frame_id), poses.get(edge.child_frame_id)
            if parent is not None and child is not None:
                edges[i] = Transform.from_matrix(
                    np.linalg.inv(parent) @ child,
                    frame_id=edge.frame_id,
                    child_frame_id=edge.child_frame_id,
                    ts=stamp,
                )
        if edges:
            self.tfbuffer.publish(*edges)
