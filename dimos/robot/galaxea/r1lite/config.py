# Copyright 2025-2026 Dimensional Inc.
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

"""R1 Lite model assets.

The arms are A1X units whose kinematics differ from the standalone A1Z
models (link lengths, joint limits, zero convention), so per-arm IK uses
chains extracted from the robot's own description. The extractor and the
source capture live in scripts/r1lite_test/; regenerate with
extract_arm_urdfs.py if a future firmware changes the description.
"""

from __future__ import annotations

from pathlib import Path

R1LITE_ARM_DOF = 6

_ASSETS = Path(__file__).parent / "assets"
R1LITE_LEFT_ARM_MODEL = _ASSETS / "r1lite_left_arm.urdf"
R1LITE_RIGHT_ARM_MODEL = _ASSETS / "r1lite_right_arm.urdf"
