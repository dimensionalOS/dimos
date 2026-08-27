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

"""Publish a robot model's complete TF tree from measured joint state."""

from __future__ import annotations

from io import BytesIO

import numpy as np
from reactivex.disposable import Disposable
import yourdfpy  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RobotTfPublisherConfig(ModuleConfig):
    robot_model: RobotModelConfig
    fixed_frame: str = "world"


class RobotTfPublisher(Module):
    """Resolve one complete parent-child TF sample per measured joint state.

    Model-driven, so it works for any URDF rather than one robot. Manipulation
    needs this because its sensor pose comes from forward kinematics, not SLAM.
    """

    config: RobotTfPublisherConfig  # type: ignore[assignment]
    coordinator_joint_state: In[JointState]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        model = self.publisher_config.robot_model
        description = prepare_urdf_for_drake(
            model.model.load(),
            convert_meshes=bool(model.auto_convert_meshes),
        )
        self._robot = yourdfpy.URDF.load(
            BytesIO(description.xml.encode()),
            load_meshes=False,
            load_collision_meshes=False,
            build_collision_scene_graph=False,
        )
        self._actuated_joint_names = frozenset(self._robot.actuated_joint_names)
        # A model-owned "world" parent would fight the base_pose edge below.
        self._edges = tuple(
            (joint.parent, joint.child)
            for joint in self._robot.robot.joints
            if joint.parent != "world"
        )

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.coordinator_joint_state.subscribe(self.on_joint_state))
        )

    def on_joint_state(self, state: JointState) -> bool:
        """Publish all model edges for one complete measured state."""
        model = self.publisher_config.robot_model
        configuration = {}
        for name, position in zip(state.name, state.position, strict=False):
            urdf_name = model.get_urdf_joint_name(name)
            if urdf_name in self._actuated_joint_names:
                configuration[urdf_name] = float(position)
        required = [model.get_urdf_joint_name(name) for name in model.joint_names]
        missing = [name for name in required if name not in configuration]
        if missing:
            # A partial state would place some links at a stale pose, which is
            # worse than publishing nothing: the self filter would trust it.
            logger.warning("Robot TF sample missing model joints: %s", missing)
            return False
        self._robot.update_cfg(configuration)

        base_pose = model.base_pose
        transforms = [
            Transform(
                translation=base_pose.position,
                rotation=base_pose.orientation,
                frame_id=self.publisher_config.fixed_frame,
                child_frame_id=model.base_link,
                ts=state.ts,
            )
        ]
        for parent, child in self._edges:
            matrix = np.asarray(self._robot.get_transform(child, parent), dtype=np.float64)
            transforms.append(
                Transform.from_matrix(
                    matrix,
                    ts=state.ts,
                    frame_id=parent,
                    child_frame_id=child,
                )
            )
        self.tf.publish(TFMessage(*transforms))
        return True

    @property
    def publisher_config(self) -> RobotTfPublisherConfig:
        return self.config  # type: ignore[return-value]


robot_tf_publisher = RobotTfPublisher.blueprint
