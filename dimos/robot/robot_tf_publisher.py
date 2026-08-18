# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Publish a robot model's complete TF tree from measured joint state."""

from __future__ import annotations

from pathlib import Path

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
    """Resolve one complete parent-child TF sample per measured joint state."""

    config: RobotTfPublisherConfig  # type: ignore[assignment]
    coordinator_joint_state: In[JointState]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        model = self.publisher_config.robot_model
        urdf_path = Path(
            prepare_urdf_for_drake(
                model.model_path,
                package_paths=model.package_paths,
                xacro_args=model.xacro_args,
            )
        )
        self._robot = yourdfpy.URDF.load(str(urdf_path), load_meshes=False)
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
        positions = dict(zip(state.name, state.position, strict=False))
        configuration = {
            model.get_urdf_joint_name(name): float(position) for name, position in positions.items()
        }
        required_joints = [model.get_urdf_joint_name(name) for name in model.joint_names]
        missing = [name for name in required_joints if name not in configuration]
        if missing:
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
