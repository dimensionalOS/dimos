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

"""Physical dimensions of the R1 Pro, as the chassis fixes them.

Frame names are not here: they are configurable, so they live on the module
configs that publish and consume them.
"""

# First-pass clearances — tune on the robot.
CHASSIS_WIDTH_M = 0.65
ROTATION_DIAMETER_M = 0.9
# Torso-up height plus margin: anything lower than this cannot be driven under.
OVERHEAD_CLEARANCE_M = 1.9
# A wheeled base on flat floors only.
MAX_STEP_HEIGHT_M = 0.03
