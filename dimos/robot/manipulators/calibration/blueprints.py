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

"""The calibration blueprint of a Damiao arm: its connection, open for tuning, and the procedure.

Each robot package defines its ``<robot>-calibrate`` blueprint with
:func:`calibration_blueprint`. There is no control coordinator: the
calibration module owns the arm through its tuning session, and a coordinator
streaming position commands would only have them dropped. Recording is pinned
to sqlite, so the markers and joint states land in
``recordings/<run-id>/memory.db`` where the fit reads them back.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.calibration.module import CalibrationModule
from dimos.robot.manipulators.damiao.connection import DamiaoConnection
from dimos.robot.manipulators.damiao.damiao_blueprint import arm_connection

CALIBRATION_RECORDING = "sqlite"


def calibration_blueprint(
    connection: type[DamiaoConnection], robot: str, **defaults: Any
) -> Blueprint:
    """``connection`` deployed with ``tuning=True`` and a calibration module for ``robot``.

    Args:
        connection: The robot's connection class.
        robot: Its ``robots.KNOWN_ROBOTS`` key.
        defaults: Calibration module settings, e.g. ``arm="left"``.
    """
    return autoconnect(
        arm_connection(connection, tuning=True),
        CalibrationModule.blueprint(robot=robot, **defaults),
    ).global_config(record=CALIBRATION_RECORDING)
