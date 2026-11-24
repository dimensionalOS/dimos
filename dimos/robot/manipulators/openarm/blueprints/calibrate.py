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

"""OpenArm calibration: its connection with ``config_writes`` and the calibration module.

    dimos run openarm-calibrate --calibrationmodule.arm left --calibrationmodule.floor-z-m Z --calibrationmodule.min-x-m X

Drive it from ``dimos shell``. See ``docs/capabilities/manipulation/arm_calibration.md``.
"""

from dimos.control.calibration.blueprints import calibration_blueprint
from dimos.robot.manipulators.openarm.connection import OpenArmConnection

openarm_calibrate = calibration_blueprint(OpenArmConnection).remappings([])
