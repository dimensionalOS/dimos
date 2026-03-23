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

"""Basic G1 sim stack: base sensors plus sim connection and planner."""

from dimos.core.blueprints import autoconnect
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.g1.legacy.blueprints.primitive.uintree_g1_primitive_no_nav import (
    uintree_g1_primitive_no_nav,
)
from dimos.robot.unitree.g1.legacy.sim import G1SimConnection

unitree_g1_basic_sim = autoconnect(
    uintree_g1_primitive_no_nav,
    G1SimConnection.blueprint(),
    ReplanningAStarPlanner.blueprint(),
)

__all__ = ["unitree_g1_basic_sim"]
