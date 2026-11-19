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

"""The calibration blueprint of a Damiao arm: its connection, the coordinator that drives it, and the procedure.

Each robot package defines its ``<robot>-calibrate`` blueprint with
:func:`calibration_blueprint`. The connection is open for config sessions,
which change how the arm is driven; the arm is moved by a
:class:`~dimos.control.calibration.coordinator.CalibrationCoordinator`
whose pass-through task sends it the calibration module's targets unchanged,
and nothing once they stop. Recording is pinned to sqlite with the Python
engine and to the streams the procedure reads back, the commands the arm was
sent included, so the markers, joint states and commands land in
``recordings/<run-id>/memory.db`` where the fit reads them; the ``check`` step
verifies the recorder writes before the arm moves.
"""

from __future__ import annotations

from typing import Any

from dimos.control.calibration.coordinator import (
    CalibrationCoordinator,
    calibration_passthrough_task,
)
from dimos.control.calibration.module import CalibrationModule
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.damiao.connection import DamiaoConnection
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    arm_connection,
    arm_declaration,
    arm_remaps,
)

CALIBRATION_RECORDING = "sqlite"
# The engine that writes recordings/<run-id>/memory.db as SqliteRecording reads it.
CALIBRATION_RECORD_ENGINE = "python"
# What the reports read (joint_state, the commands and the three marker
# streams) and what the operator sees (every marker together, the status).
CALIBRATION_RECORD_TOPICS = ",".join(
    (
        "joint_state",
        "arm_position_command",
        "gravity_events",
        "step_events",
        "float_events",
        "calibration_events",
        "calibration_status",
    )
)


def calibration_blueprint(
    connection: type[DamiaoConnection], robot: str, **defaults: Any
) -> Blueprint:
    """``connection`` open for config sessions, driven by a calibration coordinator, and a calibration module for ``robot``.

    Args:
        connection: The robot's connection class.
        robot: Its ``robots.KNOWN_ROBOTS`` key.
        defaults: Calibration module settings, e.g. ``arm="left"``.
    """
    driver = arm_connection(connection, config_sessions=True)
    joints = [f"arm/{joint}" for joint in connection.robot.arm_joint_names]
    return (
        autoconnect(
            CalibrationCoordinator.blueprint(
                instance_name="ControlCoordinator",
                connections=[arm_declaration(connection)],
                tasks=[calibration_passthrough_task(joints)],
                zero_stiffness_joints=[],
            ),
            driver,
            CalibrationModule.blueprint(robot=robot, **defaults),
        )
        .remappings(arm_remaps(connection, CalibrationCoordinator))
        .global_config(
            record=CALIBRATION_RECORDING,
            record_engine=CALIBRATION_RECORD_ENGINE,
            record_topics=CALIBRATION_RECORD_TOPICS,
        )
    )
