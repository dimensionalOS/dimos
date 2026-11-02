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

"""One URDF and the live joint state: every joint on tf, and the URDF and its files for renderers."""

from __future__ import annotations

import asyncio
from pathlib import Path
from threading import RLock
import time
import xml.etree.ElementTree as ET

import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.files.File import File
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import RobotModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Their pose comes from tf (odometry, a planar base), so they are read from tf, never published to it.
MULTI_DOF_JOINT_TYPES = frozenset({"floating", "planar"})


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

    # IO: the joints it poses are published on it (and UrdfManagerPointClearing reads a cloud's mount from it).
    tf: IO[TFMessage]
    coordinator_joint_state: In[JointState]
    # A request for a *.urdf gets the URDF; one for a file the URDF names (mesh, texture, splat, ...) by its full path
    # gets that file. Either goes out on file_info named as requested.
    render_file_request: In[String]
    file_info: Out[File]

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
            self.file_info.publish(File(name, self._description.xml.encode()))
            return
        # Only files the URDF names: a request is never an arbitrary path into the filesystem.
        if name not in self._file_paths:
            logger.warning(f"render_file_request for {name!r}, which is not a file the URDF names")
            return
        data = await asyncio.to_thread(Path(name).read_bytes)
        self.file_info.publish(File(name, data))

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
            if joint.type not in MULTI_DOF_JOINT_TYPES
        ]


def _files_named_by(xml: str) -> set[str]:
    """The full path of every file the URDF names (mesh, texture, splat, ...): any filename attribute."""
    return {
        element.attrib["filename"]
        for element in ET.fromstring(xml).iter()
        if "filename" in element.attrib
    }
