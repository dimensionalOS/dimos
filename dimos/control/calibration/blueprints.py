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

"""The ``<robot>-calibrate`` blueprints: a Damiao connection that takes ``set_config``, and the calibration module.

The module's ``position_command`` and the connection's ``joint_state`` connect by name;
``dimos arm calibrate <robot>`` runs these blueprints and answers the module, and reads the
robot they calibrate off them (``calibrated_robot``).
"""

from __future__ import annotations

from typing import Any

from dimos.control.calibration.module import CalibrationModule
from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.robot.manipulators.damiao.connection import DamiaoConnection
from dimos.robot.manipulators.damiao.damiao_blueprint import arm_connection
from dimos.robot.manipulators.damiao.robot import DamiaoRobot


def calibration_blueprint(
    connection: type[DamiaoConnection], robot: DamiaoRobot, **defaults: Any
) -> MetaBlueprint:
    """``connection`` with ``config_writes`` and a calibration module for an arm of ``robot``."""
    return autoconnect(
        arm_connection(connection, config_writes=True),
        CalibrationModule.blueprint(robot=robot, **defaults),
    )


def calibrated_robot(blueprint: Blueprint | MetaBlueprint) -> DamiaoRobot:
    """The robot a calibration blueprint hands its calibration module."""
    [atom] = [
        atom
        for atom in autoconnect(blueprint).solidify().blueprints
        if atom.module is CalibrationModule
    ]
    robot: DamiaoRobot = atom.kwargs["robot"]
    return robot
