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

"""MakerMods Metal calibration: the connection open for config sessions and the calibration module.

    dimos run metal-calibrate --metalconnection.can-port can0

The Metal does not compensate gravity, so its joints are never floated.
Without a workspace guard of its own the arm needs the table height and the
line its fingertip stays in front of: ``--calibrationmodule.floor-z-m`` and
``--calibrationmodule.min-x-m``, metres from its base.
See ``dimos.robot.manipulators.calibration.blueprints``.
"""

from dimos.robot.manipulators.calibration.blueprints import calibration_blueprint
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection

metal_calibrate = calibration_blueprint(MetalConnection, "metal").remappings([])
