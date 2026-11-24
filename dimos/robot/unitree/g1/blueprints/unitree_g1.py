#!/usr/bin/env python3
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

"""Default G1 blueprint: onboard sensors + locomotion SDK, or the GR00T WBC stack under --simulation."""

from dimos.core.global_config import global_config

if global_config.simulation and global_config.simulation != "mujoco":
    raise ValueError("unitree-g1 only supports --simulation mujoco")

# imports are branched because the hardware stack needs the optional unitree-dds extra
if global_config.simulation == "mujoco":
    from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_groot_wbc import (
        unitree_g1_groot_wbc as _g1_stack,
    )
else:
    from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_onboard import (
        _unitree_g1_onboard_nav as _g1_stack,
    )

# the all_blueprints generator AST-scans for a top-level `.global_config(...)` call
unitree_g1 = _g1_stack.global_config(robot_model="unitree_g1")
