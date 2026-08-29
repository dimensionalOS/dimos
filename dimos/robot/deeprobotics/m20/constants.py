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

"""Documented and provisional geometry values for the Lynx M20."""

import math

# Documented body dimensions in the vendor hardware guide.
BODY_LENGTH_M = 0.82
BODY_WIDTH_M = 0.506

# The official M20 locomotion SDK's stand_height_ default. Verify the PointLIO
# base origin and physical clearance on hardware before planner tuning.
BASE_LINK_HEIGHT_M = 0.48

# Conservative initial MLS clearance. This includes the body above base_link
# and remains a hardware-tuning value rather than a vendor specification.
PLANNING_HEIGHT_M = 0.65

ROTATION_DIAMETER_M = math.hypot(BODY_LENGTH_M, BODY_WIDTH_M)

# Conservative command bounds until direction signs, gait, latency, and
# stopping distance have been measured on the actual robot.
MAX_LINEAR_X_M_S = 0.3
MAX_LINEAR_Y_M_S = 0.3
MAX_ANGULAR_Z_RAD_S = 0.5
