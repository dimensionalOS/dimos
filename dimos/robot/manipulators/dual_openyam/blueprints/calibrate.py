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

"""Dual OpenYAM calibration: the connection open for config sessions, its coordinator and the calibration module.

    dimos run dual-openyam-calibrate --dualopenyamconnection.left-can-port can0 --dualopenyamconnection.right-can-port can1

The left arm is calibrated unless ``--calibrationmodule.arm right``; the other arm
is held where it is by the coordinator meanwhile. Poses need the table height and
the front line (``--calibrationmodule.floor-z-m``, ``--calibrationmodule.min-x-m``);
the midline between the two bases in the URDF keeps each arm on its own half.
See ``dimos.control.calibration.blueprints``.
"""

from dimos.control.calibration.blueprints import calibration_blueprint
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection

dual_openyam_calibrate = calibration_blueprint(
    DualOpenYamConnection, "dual-openyam", arm="left"
).remappings([])
