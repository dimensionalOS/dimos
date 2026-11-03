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

"""Canonical names of the two Metal arms, matching the setup on main."""

from dimos.robot.manipulators.makermods_metal.config import METAL_ARM_JOINTS

SIDES = ("left", "right")
ARM_JOINTS = {side: tuple(f"{side}_{joint}" for joint in METAL_ARM_JOINTS) for side in SIDES}
ALL_ARM_JOINTS = (*ARM_JOINTS["left"], *ARM_JOINTS["right"])
