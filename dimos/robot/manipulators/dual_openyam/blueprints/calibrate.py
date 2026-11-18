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

"""Dual OpenYAM calibration: the connection open for tuning and the calibration module.

    dimos run dual-openyam-calibrate --dualopenyamconnection.left-can-port can0 --dualopenyamconnection.right-can-port can1

The left arm is calibrated unless ``--calibrationmodule.arm right``; the other arm
is held where it is by the connection meanwhile. The bench guard of
``calibration_workspace`` keeps each arm on its own half of the table.
See ``dimos.robot.manipulators.calibration.blueprints``.
"""

from dimos.robot.manipulators.calibration.blueprints import calibration_blueprint
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection

dual_openyam_calibrate = calibration_blueprint(
    DualOpenYamConnection, "dual-openyam", arm="left"
).remappings([])
