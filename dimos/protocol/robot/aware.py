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

"""A module mixin: `self.robot` to read the robot, `publish_robot` for the parts it owns.

The three topics are shared by every robot; robot names keep them apart on the bus.
Anyone may publish anything. Ports left unwired are simply not used.

SKETCH: signatures only.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

from pydantic import Field

from dimos.core.module import ModuleConfig
from dimos.core.stream import IO
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.robot.model import JointValues, RobotModel
from dimos.protocol.robot.source import LiveSource
from dimos.protocol.robot.state import Robot


class RobotAwareConfig(ModuleConfig):
    # The model to attach. None: follow what arrives on robot_model for robot_name.
    robot: RobotModel | None = None
    # Which robot on the bus; "" is a single robot with bare names.
    robot_name: str = ""
    # Parts this module published are re-sent this often for late joiners.
    model_hz: float = Field(default=0.2, gt=0.0)


class RobotAware:
    config: RobotAwareConfig

    tf: IO[TFMessage]
    joint_state: IO[JointState]
    robot_model: IO[RobotModel]

    @property
    def source(self) -> LiveSource:
        """This module's ports as a RobotSource; built once."""
        raise NotImplementedError

    @property
    def robot(self) -> Robot:
        """`config.robot` (or the received model) attached as `config.robot_name`."""
        raise NotImplementedError

    @property
    def robots(self) -> Mapping[str, Robot]:
        """Every robot the bus has sent a model for, by name, each its received model."""
        raise NotImplementedError

    def publish_robot(
        self,
        *,
        positions: JointValues | None = None,
        velocities: JointValues | None = None,
        efforts: JointValues | None = None,
        pose: Transform | None = None,
        frames: Iterable[Transform] = (),
        model: RobotModel | None = None,
        part: str | None = None,
        ts: float | None = None,
    ) -> None:
        """Publish whatever part of the robot this module has, as `config.robot_name`.

        positions/velocities/efforts go on joint_state; `pose` (the odometry edge into the
        robot, e.g. odom -> base_link) on tf; `model` (a whole robot or a subtree) and
        `frames` (mounts, as a one-edge model part) on robot_model. Mounts never go on tf.

        Model parts are kept under `part` (default: this module's name) and re-sent at
        `config.model_hz`; publishing the same part again replaces it.
        """
        raise NotImplementedError
