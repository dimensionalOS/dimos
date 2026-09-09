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

"""Publish moving link transforms onto tf by running FK on a JointState stream.

:class:`~dimos.protocol.tf.static_tf_publisher.StaticTfPublisher` covers rigid
mounts. This covers the other case: a sensor mounted beyond one or more moving
joints, where the transform is only correct if it is recomputed from the live
joint angles. Sensor frames published here resolve like any other tf edge, so a
consumer that looks up a cloud's ``frame_id`` gets the pose the joints were
actually at.

FK runs through pinocchio rather than yourdfpy: pinocchio is a core dependency,
whereas yourdfpy is visualization-only and is excluded on linux/aarch64 — which
is exactly the kind of board these modules run on.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import RobotModel
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


class JointStateTfPublisherConfig(ModuleConfig):
    model: RobotModel
    # Links whose transforms get published. Naming them explicitly keeps tf to
    # the frames something actually looks up, instead of every link in the URDF.
    links: tuple[str, ...] = ()
    # Parent for every published edge. Empty uses the model's own root link.
    root_link: str = ""
    # Joint feedback is typically 100 Hz, which is far more tf than any consumer
    # needs; the voxel map matches on stamp within a tolerance anyway.
    max_publish_hz: float = Field(default=30.0, gt=0.0)


class JointStateTfPublisher(Module):
    """Forward-kinematics tf source for sensors mounted past a moving joint."""

    config: JointStateTfPublisherConfig
    joint_states: In[JointState]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._pin_model: Any = None
        self._pin_data: Any = None
        self._root_link = ""
        self._frame_ids: dict[str, int] = {}
        self._joint_q_index: dict[str, int] = {}
        self._neutral_q: NDArray[np.float64] | None = None
        self._unknown_joints: set[str] = set()
        self._last_publish_ts = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        import pinocchio

        loaded = self.config.model.load()
        self._pin_model = pinocchio.buildModelFromXML(loaded.xml)
        self._pin_data = self._pin_model.createData()
        self._neutral_q = pinocchio.neutral(self._pin_model)
        self._root_link = self.config.root_link or loaded.root_link

        for joint_id in range(1, self._pin_model.njoints):
            joint = self._pin_model.joints[joint_id]
            if joint.nq == 1:
                self._joint_q_index[self._pin_model.names[joint_id]] = joint.idx_q

        for link in self.config.links:
            if not self._pin_model.existFrame(link):
                raise ValueError(
                    f"{type(self).__name__}: link {link!r} is not in "
                    f"{self.config.model.source_path}"
                )
            self._frame_ids[link] = self._pin_model.getFrameId(link)

        logger.info(
            "%s: publishing %s relative to %r",
            type(self).__name__,
            ", ".join(self.config.links),
            self._root_link,
        )
        self.register_disposable(self.joint_states.observable().subscribe(self._on_joint_state))  # type: ignore[no-untyped-call]

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_joint_state(self, msg: JointState) -> None:
        min_period = 1.0 / self.config.max_publish_hz
        if msg.ts - self._last_publish_ts < min_period:
            return
        self._last_publish_ts = msg.ts
        self.tf.publish(TFMessage(*self._link_transforms(msg)))

    def _link_transforms(self, msg: JointState) -> list[Transform]:
        import pinocchio
        from scipy.spatial.transform import Rotation

        assert self._neutral_q is not None
        q = self._neutral_q.copy()
        for name, position in zip(msg.name, msg.position, strict=False):
            index = self._joint_q_index.get(name)
            if index is None:
                if name not in self._unknown_joints:
                    self._unknown_joints.add(name)
                    logger.warning("%s: no model joint named %r", type(self).__name__, name)
                continue
            q[index] = position

        pinocchio.framesForwardKinematics(self._pin_model, self._pin_data, q)
        transforms = []
        for link, frame_id in self._frame_ids.items():
            placement = self._pin_data.oMf[frame_id]
            x, y, z, w = Rotation.from_matrix(placement.rotation).as_quat()
            transforms.append(
                Transform(
                    translation=Vector3(*placement.translation),
                    rotation=Quaternion(x, y, z, w),
                    frame_id=self._root_link,
                    child_frame_id=link,
                    ts=msg.ts,
                )
            )
        return transforms
