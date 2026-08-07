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

"""Default G1 blueprint. The ``--simulation`` flag picks the backend.

Real hardware (default):
    Onboard sensors + high-level locomotion SDK, raytracing costmap, and A*
    replanning.

Sim (``--simulation``, which normalizes to ``mujoco``):
    The GR00T WBC stack from ``unitree-g1-groot-wbc``. There is no simulated
    equivalent of the robot's onboard high-level walking controller, so the
    GR00T whole-body policy is what "the G1 walks" means in sim. Deferring to
    that blueprint also keeps the hardware DDS SDK (``unitree_sdk2py``, an
    optional ``unitree-dds`` extra) out of the sim import path.

Usage:
    dimos run unitree-g1                # real hardware
    dimos --simulation run unitree-g1   # mujoco
"""

from typing import Any

from dimos.core.global_config import global_config

if global_config.simulation and global_config.simulation != "mujoco":
    raise ValueError("unitree-g1 only supports --simulation mujoco")

_g1_global_config: dict[str, Any]

if global_config.simulation == "mujoco":
    from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_groot_wbc import (
        unitree_g1_groot_wbc,
    )

    _g1_stack = unitree_g1_groot_wbc
    # groot-wbc already picked a sim-appropriate worker count; don't clobber it.
    _g1_global_config = {}
else:
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_onboard import _unitree_g1_onboard
    from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_nav_simple import (
        _unitree_g1_nav_simple,
    )

    _g1_stack = autoconnect(_unitree_g1_onboard, _unitree_g1_nav_simple)
    _g1_global_config = {"n_workers": 10}

# Kept as a top-level `.global_config(...)` call: the all_blueprints generator
# AST-scans for exactly that shape, and a bare alias assignment is invisible
# to it (the blueprint would silently vanish from the registry).
unitree_g1 = _g1_stack.global_config(robot_model="unitree_g1", **_g1_global_config)
